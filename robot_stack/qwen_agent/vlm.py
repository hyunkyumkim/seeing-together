#!/usr/bin/env python3
"""Qwen-VL backends for the agent. Both take the same message list and return the model's text.

Messages are OpenAI-style: {"role": "system"|"user"|"assistant", "content": str | [parts]}, where a
part is {"type": "text", "text": ...} or {"type": "image", "image": PIL.Image}.

  LocalQwenVL   loads the weights with transformers on this machine's GPU (default).
  OpenAIVL      sends the same conversation to an OpenAI-compatible server, e.g. vLLM:
                  vllm serve Qwen/Qwen3-VL-8B-Instruct --max-model-len 16384
"""
import base64
import io
import json
import os
import urllib.request

MAX_PIXELS = 640 * 28 * 28        # ~640 visual tokens per image at most; raise for more detail
MIN_PIXELS = 128 * 28 * 28


def _jpeg_b64(img, quality=85):
    buf = io.BytesIO()
    img.convert("RGB").save(buf, "JPEG", quality=quality)
    return base64.b64encode(buf.getvalue()).decode()


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
        return self.processor.batch_decode(new, skip_special_tokens=True, clean_up_tokenization_spaces=False)[0]


class OpenAIVL:
    def __init__(self, base_url="http://127.0.0.1:8000/v1", model="Qwen/Qwen3-VL-8B-Instruct", api_key="none",
                 max_new_tokens=512, timeout=600):
        self.base_url, self.model, self.api_key = base_url.rstrip("/"), model, api_key
        self.max_new_tokens, self.timeout = max_new_tokens, timeout

    @staticmethod
    def _convert(messages):
        conv = []
        for m in messages:
            c = m["content"]
            if isinstance(c, list):
                c = [{"type": "image_url", "image_url": {"url": "data:image/jpeg;base64," + _jpeg_b64(p["image"])}}
                     if p.get("type") == "image" else p for p in c]
            conv.append({"role": m["role"], "content": c})
        return conv

    def chat(self, messages):
        body = {"model": self.model, "messages": self._convert(messages), "max_tokens": self.max_new_tokens,
                "temperature": 0}
        req = urllib.request.Request(self.base_url + "/chat/completions", data=json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json",
                                              "Authorization": f"Bearer {self.api_key}"}, method="POST")
        with urllib.request.urlopen(req, timeout=self.timeout) as r:
            return json.loads(r.read())["choices"][0]["message"]["content"] or ""


def make_backend(kind, model, url=None, device="cuda:0", max_new_tokens=512, api_key=None):
    """kind 'local' loads Qwen on this GPU. kind 'openai' calls /v1/chat/completions: a local vLLM
    server, or api.openai.com when OPENAI_API_KEY is set and --model is an OpenAI vision model."""
    if kind == "openai":
        url = url or os.environ.get("QWEN_URL") or "http://127.0.0.1:8000/v1"
        key = api_key or os.environ.get("OPENAI_API_KEY") or os.environ.get("QWEN_API_KEY") or "none"
        if "api.openai.com" in url and key == "none":
            raise SystemExit("Set OPENAI_API_KEY to call api.openai.com")
        print(f"[vlm] OpenAI-compatible {url}  model {model}", flush=True)
        return OpenAIVL(url, model, api_key=key, max_new_tokens=max_new_tokens)
    return LocalQwenVL(model, device=device, max_new_tokens=max_new_tokens)
