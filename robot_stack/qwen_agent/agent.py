#!/usr/bin/env python3
"""A local Qwen3-VL agent that drives the Go2 through the console's tools.

Loop: the model reads the task and what it has seen so far, writes a sentence of reasoning and one
tool call; the tool runs on the dog; its result (text and photos) goes back to the model; repeat until
the model calls finish or the step limit is reached. Tool calls use Qwen's native format:
<tool_call>{"name": ..., "arguments": {...}}</tool_call>.

  ./run.sh "find the red cup"          one task, then exit
  ./run.sh                             interactive: type tasks, /reset clears memory, /quit exits
Every run is logged under runs/<timestamp>/ (transcript.jsonl and the photos the model saw).
"""
import argparse
import copy
import json
import os
import re
import sys
import time

import go2_tools as tools

HERE = os.path.dirname(os.path.abspath(__file__))
TOOL_RE = re.compile(r"<tool_call>\s*(\{.*?\})\s*</tool_call>", re.S)

TOOLS_BLOCK = """

# Tools

You may call one function to assist with the user query.

You are provided with function signatures within <tools></tools> XML tags:
<tools>
{specs}
</tools>

For each function call, return a json object with function name and arguments within <tool_call></tool_call> XML tags:
<tool_call>
{{"name": <function-name>, "arguments": <args-json-object>}}
</tool_call>
Write one or two short sentences first (what you see, why this tool), then exactly one <tool_call>."""


def system_prompt():
    with open(os.path.join(HERE, "prompt.md")) as f:
        base = f.read().strip()
    specs = "\n".join(json.dumps({"type": "function", "function": t}) for t in tools.TOOLS)
    return base + TOOLS_BLOCK.format(specs=specs)


def _first_json(s):
    """The first balanced {...} object in s, or None."""
    start = s.find("{")
    while start != -1:
        depth = 0
        for i in range(start, len(s)):
            if s[i] == "{":
                depth += 1
            elif s[i] == "}":
                depth -= 1
                if depth == 0:
                    try:
                        return json.loads(s[start:i + 1]), start
                    except ValueError:
                        break
        start = s.find("{", start + 1)
    return None, -1


def parse_reply(reply):
    """(thought, name, arguments). name is None when the model did not call a tool."""
    m = TOOL_RE.search(reply)
    if m:
        thought = reply[:m.start()].strip()
        try:
            obj = json.loads(m.group(1))
        except ValueError:
            obj, _ = _first_json(m.group(1))
    else:
        obj, pos = _first_json(reply)
        if not (isinstance(obj, dict) and "name" in obj):
            return reply.strip(), None, None
        thought = re.sub(r"```(?:json)?\s*$", "", reply[:pos].strip()).strip()
    if not isinstance(obj, dict) or "name" not in obj:
        return reply.strip(), None, None
    args = obj.get("arguments", obj.get("parameters", {}))
    if isinstance(args, str):
        try:
            args = json.loads(args)
        except ValueError:
            args = {}
    return thought, str(obj["name"]), args if isinstance(args, dict) else {}


def prune_images(messages, keep):
    """A copy of messages with all but the last `keep` images replaced by a short text, so the context
    stays within the model's limit. The model's own log tells it what those photos showed."""
    out = copy.copy(messages)
    seen = 0
    for i in range(len(out) - 1, -1, -1):
        c = out[i]["content"]
        if not isinstance(c, list):
            continue
        parts = []
        for p in reversed(c):
            if p.get("type") == "image":
                seen += 1
                if seen > keep:
                    p = {"type": "text", "text": "[earlier photo omitted]"}
            parts.append(p)
        out[i] = dict(out[i], content=parts[::-1])
    return out


class Session:
    def __init__(self, vlm, max_steps, keep_images, run_dir):
        self.vlm, self.max_steps, self.keep_images, self.run_dir = vlm, max_steps, keep_images, run_dir
        self.system = system_prompt()
        self.reset()
        os.makedirs(run_dir, exist_ok=True)
        self.log = open(os.path.join(run_dir, "transcript.jsonl"), "a")
        self.photo_n = 0

    def reset(self):
        self.messages = [{"role": "system", "content": self.system}]

    def record(self, **ev):
        self.log.write(json.dumps({"t": round(time.time(), 2), **ev}) + "\n")
        self.log.flush()

    def save_images(self, name, imgs):
        paths = []
        for cap, img in imgs:
            self.photo_n += 1
            p = os.path.join(self.run_dir, f"{self.photo_n:03d}_{name}.jpg")
            img.save(p, "JPEG", quality=85)
            paths.append(p)
        return paths

    def run_task(self, task):
        print(f"\n=== task: {task}", flush=True)
        self.record(event="task", text=task)
        self.messages.append({"role": "user", "content": task})
        for step in range(1, self.max_steps + 1):
            t0 = time.time()
            reply = self.vlm.chat(prune_images(self.messages, self.keep_images))
            self.messages.append({"role": "assistant", "content": reply})
            thought, name, args = parse_reply(reply)
            if thought:
                print(thought, flush=True)
            if name is None:
                print(f"[no tool call, stopping after {step} steps]", flush=True)
                self.record(event="final", text=reply)
                tools.release_control()
                return
            print(f"  → {name} {json.dumps(args)}   ({time.time() - t0:.1f} s thinking)", flush=True)
            text, imgs = tools.run(name, args)
            paths = self.save_images(name, imgs)
            print("    " + text.replace("\n", " | ")[:400] + (f"  [{len(imgs)} image(s)]" if imgs else ""), flush=True)
            self.record(event="step", n=step, thought=thought, tool=name, args=args, result=text, images=paths)
            if name == "finish":
                print(f"[done in {step} steps]", flush=True)
                return
            content = [{"type": "text", "text": f"[result of {name}]\n{text}"}]
            for cap, img in imgs:
                content += [{"type": "text", "text": cap}, {"type": "image", "image": img}]
            self.messages.append({"role": "user", "content": content})
        print(f"[step limit {self.max_steps} reached; stopping the dog]", flush=True)
        tools.stop()
        tools.release_control()
        self.record(event="step_limit")


def main():
    ap = argparse.ArgumentParser(description="Local Qwen3-VL agent for the Go2")
    ap.add_argument("task", nargs="*", help="what to do; empty = interactive")
    ap.add_argument("--backend", choices=("local", "openai"), default=os.environ.get("QWEN_BACKEND", "local"))
    ap.add_argument("--model", default=os.environ.get("QWEN_MODEL", "Qwen/Qwen3-VL-8B-Instruct"))
    ap.add_argument("--url", default=os.environ.get("QWEN_URL"), help="OpenAI-compatible server (backend=openai)")
    ap.add_argument("--device", default="cuda:0")
    ap.add_argument("--max-steps", type=int, default=30)
    ap.add_argument("--keep-images", type=int, default=6, help="most recent photos kept in the model's context")
    ap.add_argument("--max-new-tokens", type=int, default=512)
    ap.add_argument("--runs", default=os.path.join(HERE, "runs"))
    a = ap.parse_args()

    if not tools.console_up():
        sys.exit(f"The console isn't running at {tools.CONSOLE}: start it with run_go2_keys.sh")
    from vlm import make_backend
    vlm = make_backend(a.backend, a.model, a.url, a.device, a.max_new_tokens)
    run_dir = os.path.join(a.runs, time.strftime("%Y%m%d_%H%M%S"))
    s = Session(vlm, a.max_steps, a.keep_images, run_dir)
    print(f"[log] {run_dir}", flush=True)
    try:
        if a.task:
            s.run_task(" ".join(a.task))
            return
        print("Type a task for the dog. /reset forgets the conversation, /quit exits.", flush=True)
        while True:
            try:
                line = input("\n> ").strip()
            except EOFError:
                break
            if not line:
                continue
            if line == "/quit":
                break
            if line == "/reset":
                s.reset()
                print("[memory cleared]")
                continue
            s.run_task(line)
    except KeyboardInterrupt:
        print("\n[interrupted: stopping the dog]", flush=True)
    finally:
        tools.stop()
        tools.release_control()


if __name__ == "__main__":
    main()
