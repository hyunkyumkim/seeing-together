#!/usr/bin/env python3
"""Vision backends for the navigation agent. Both take the same message list.

Messages are OpenAI-style: {"role": "system"|"user"|"assistant"|"tool", "content": str | [parts]},
where a part is {"type": "text", "text": ...} or {"type": "image", "image": PIL.Image}.

  LocalQwenVL   loads Qwen with transformers on this machine's GPU. The model writes
                <tool_call> XML, which the agent parses.
  OpenAIVL      calls /v1/chat/completions. api.openai.com uses native function tools
                (the reply carries the tool call). Any other URL, such as vLLM, is the
                same text channel as the local model.
"""
import base64
import io
import json
import os
import urllib.error
import urllib.request

MAX_PIXELS = 640 * 28 * 28        # ~640 visual tokens per image at most; raise for more detail
MIN_PIXELS = 128 * 28 * 28


def _jpeg_b64(img, quality=85):
    buf = io.BytesIO()
    img.convert("RGB").save(buf, "JPEG", quality=quality)
    return base64.b64encode(buf.getvalue()).decode()


class ModelReply:
    """One model turn. message is set when the history must store an OpenAI tool_calls object;
    otherwise the agent stores text and parses a <tool_call> itself."""

    def __init__(self, text, name=None, args=None, message=None):
        self.text = text or ""
        self.name = name
        self.args = args or {}
        self.message = message


def _text_of(content):
    if content is None:
        return ""
    if isinstance(content, str):
        return content
    if isinstance(content, list):
        return "\n".join(p.get("text", "") for p in content
                         if isinstance(p, dict) and p.get("type") in ("text", "output_text"))
    return str(content)


def reply_from_openai(message):
    """choices[0].message from chat/completions, with or without tool_calls."""
    text = _text_of(message.get("content")).strip()
    calls = message.get("tool_calls") or []
    stored = {"role": "assistant", "content": text}
    if not calls:
        return ModelReply(text, message=stored)
    call = calls[0]
    fn = call.get("function") or {}
    name = str(fn.get("name") or "")
    raw = fn.get("arguments") if fn.get("arguments") is not None else "{}"
    if isinstance(raw, str):
        try:
            args = json.loads(raw) if raw.strip() else {}
        except ValueError:
            args = {}
        raw_s = raw
    elif isinstance(raw, dict):
        args, raw_s = raw, json.dumps(raw)
    else:
        args, raw_s = {}, "{}"
    stored["tool_calls"] = [{
        "id": call.get("id") or "call_0",
        "type": "function",
        "function": {"name": name, "arguments": raw_s},
    }]
    return ModelReply(text, name or None, args if isinstance(args, dict) else {}, stored)


class LocalQwenVL:
    def __init__(self, model_id="Qwen/Qwen3-VL-8B-Instruct", device="cuda:0", max_new_tokens=512,
                 max_pixels=MAX_PIXELS):
        import torch
        from transformers import AutoModelForImageTextToText, AutoProcessor
        self.torch = torch
        self.max_new_tokens = max_new_tokens
        print(f"[vlm] loading {model_id} on {device} ...", flush=True)
        self.processor = AutoProcessor.from_pretrained(model_id, min_pixels=MIN_PIXELS, max_pixels=max_pixels)
        kw = dict(device_map={"": device}, attn_implementation="sdpa")
        try:
            self.model = AutoModelForImageTextToText.from_pretrained(model_id, dtype=torch.bfloat16, **kw)
        except TypeError:       # older transformers
            self.model = AutoModelForImageTextToText.from_pretrained(model_id, torch_dtype=torch.bfloat16, **kw)
        self.model.eval()
        mem = torch.cuda.memory_allocated(self.model.device) / 1e9 if torch.cuda.is_available() else 0
        print(f"[vlm] ready, {mem:.1f} GB of weights on {self.model.device}", flush=True)

    def chat(self, messages):
        # Qwen3-VL's processor packs the images itself. Qwen2.5-VL needs qwen_vl_utils.
        try:
            inputs = self.processor.apply_chat_template(
                messages, tokenize=True, add_generation_prompt=True, return_dict=True, return_tensors="pt")
        except TypeError:
            from qwen_vl_utils import process_vision_info
            text = self.processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
            images, videos = process_vision_info(messages)
            inputs = self.processor(text=[text], images=images, videos=videos, padding=True, return_tensors="pt")
        inputs = inputs.to(self.model.device)
        with self.torch.inference_mode():
            out = self.model.generate(**inputs, max_new_tokens=self.max_new_tokens, do_sample=False,
                                      temperature=None, top_p=None, top_k=None)
        new = out[:, inputs["input_ids"].shape[1]:]
        text = self.processor.batch_decode(new, skip_special_tokens=True, clean_up_tokenization_spaces=False)[0]
        return ModelReply(text)


class OpenAIVL:
    def __init__(self, base_url="https://api.openai.com/v1", model="gpt-4o", api_key="none",
                 max_new_tokens=512, timeout=600, tools=None, native_tools=False):
        self.base_url, self.model, self.api_key = base_url.rstrip("/"), model, api_key
        self.max_new_tokens, self.timeout = max_new_tokens, timeout
        self.tools, self.native_tools = tools, native_tools

    @staticmethod
    def _convert(messages):
        conv = []
        for m in messages:
            out = {"role": m["role"]}
            if m.get("tool_call_id"):
                out["tool_call_id"] = m["tool_call_id"]
            if m.get("name") and m["role"] == "tool":
                out["name"] = m["name"]
            if m.get("tool_calls"):
                out["tool_calls"] = m["tool_calls"]
            c = m.get("content")
            if isinstance(c, list):
                c = [{"type": "image_url", "image_url": {"url": "data:image/jpeg;base64," + _jpeg_b64(p["image"])}}
                     if p.get("type") == "image" else p for p in c]
            if c is not None:
                out["content"] = c
            elif "tool_calls" not in out:
                out["content"] = ""
            conv.append(out)
        return conv

    def chat(self, messages):
        body = {"model": self.model, "messages": self._convert(messages), "max_tokens": self.max_new_tokens,
                "temperature": 0}
        if self.native_tools and self.tools:
            body["tools"] = self.tools
            body["tool_choice"] = "auto"
        req = urllib.request.Request(self.base_url + "/chat/completions", data=json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json",
                                              "Authorization": f"Bearer {self.api_key}"}, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                data = json.loads(r.read())
        except urllib.error.HTTPError as e:
            detail = e.read().decode(errors="replace")[:500]
            raise RuntimeError(f"OpenAI API {e.code}: {detail}") from None
        reply = reply_from_openai(data["choices"][0]["message"])
        if not self.native_tools:
            return ModelReply(reply.text)
        return reply


def make_backend(kind, model, url=None, device="cuda:0", max_new_tokens=512, api_key=None, tools=None):
    """kind 'local' loads Qwen on this GPU. kind 'openai' calls /v1/chat/completions.
    api.openai.com gets native function tools; another URL (a local vLLM server) does not."""
    if kind == "openai":
        url = url or os.environ.get("QWEN_URL") or "https://api.openai.com/v1"
        key = api_key or os.environ.get("OPENAI_API_KEY") or os.environ.get("QWEN_API_KEY") or "none"
        native = "api.openai.com" in url
        if native and key == "none":
            raise SystemExit("Set OPENAI_API_KEY, or run with QWEN_BACKEND=local for the on-GPU model.")
        print(f"[vlm] {'OpenAI tools' if native else 'OpenAI-compatible'} {url}  model {model}", flush=True)
        backend = OpenAIVL(url, model, api_key=key, max_new_tokens=max_new_tokens,
                           tools=tools, native_tools=native)
        return backend
    backend = LocalQwenVL(model, device=device, max_new_tokens=max_new_tokens)
    backend.native_tools = False
    return backend
