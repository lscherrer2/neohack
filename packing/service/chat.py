"""Terminal stand-in for the Vapi phone agent:  python -m packing.service chat

It plays both of Vapi's roles so the rest of the system can't tell the difference:
  * the ORDERING assistant (Claude Haiku) answers a "call" typed in the terminal and calls tools.
    Every tool call is wrapped in the exact Vapi `tool-calls` message and handled by the same code as
    POST /vapi/webhook (in-process by default, or over HTTP with --server).
  * the CALLBACK call: when an order is packed, the worker's notifier "rings" the terminal instead of
    POSTing to api.vapi.ai/call; type /answer to talk to the callback assistant, which gets the same
    variables Vapi would ({{order_id}}, {{items}}, {{customer_name}}).

Prompts live in packing/service/prompts/ and are meant to be pasted into Vapi unchanged.
Swapping to real Vapi = put the same prompts + tools in Vapi, run `serve`, set the VAPI_* keys.
"""

from __future__ import annotations

import os
import queue
import re
import threading
import uuid
from pathlib import Path

import anthropic
import httpx

from .orders import Order
from .vapi import Catalog, ToolHandler, VapiNotifier, VapiSettings, tool_definitions

MODEL = "claude-haiku-5-5"
PROMPTS = Path(__file__).parent / "prompts"
REPO_ROOT = Path(__file__).resolve().parents[2]

HELP = """commands:  /hangup  end this call (next line starts a new call)
           /answer  pick up a callback from the shop
           /orders  show the order queue
           /quit    exit"""


def load_api_key() -> None:
    """ANTHROPIC_API_KEY from the environment, else from <repo>/.env.local (never printed)."""
    if os.getenv("ANTHROPIC_API_KEY"):
        return
    f = REPO_ROOT / ".env.local"
    if f.exists():
        for line in f.read_text().splitlines():
            m = re.match(r'\s*(?:export\s+)?ANTHROPIC_API_KEY\s*=\s*["\']?([^"\'\s#]+)', line)
            if m:
                os.environ["ANTHROPIC_API_KEY"] = m.group(1)
                return
    raise SystemExit(f"ANTHROPIC_API_KEY not set and not found in {f}")


def render(template: str, variables: dict[str, str]) -> str:
    """Minimal stand-in for Vapi's {{variable}} substitution."""
    return re.sub(r"\{\{\s*(\w+)\s*\}\}", lambda m: str(variables.get(m.group(1), "")), template)


def anthropic_tools() -> list[dict]:
    """Same tool definitions we give Vapi, converted to the Messages API shape."""
    return [{"name": t["function"]["name"], "description": t["function"]["description"],
             "input_schema": t["function"]["parameters"]} for t in tool_definitions("local")]


# ---------------------------------------------------------------- Vapi-side transports
class InProcessVapi:
    """Delivers tool calls to the webhook handler directly (no server needed)."""

    def __init__(self, handler: ToolHandler):
        self.handler = handler

    def send(self, message: dict) -> dict:
        return self.handler.handle_message(message)


class HttpVapi:
    """Delivers tool calls to a running `serve` instance, exactly like Vapi would."""

    def __init__(self, url: str, secret: str | None):
        self.url = url.rstrip("/") + "/vapi/webhook"
        self.headers = {"X-Vapi-Secret": secret} if secret else {}
        self.client = httpx.Client(timeout=20)

    def send(self, message: dict) -> dict:
        r = self.client.post(self.url, json={"message": message}, headers=self.headers)
        r.raise_for_status()
        return r.json()


# ---------------------------------------------------------------- callback "phone line"
class TerminalCallbackNotifier:
    """Replaces the Vapi outbound call: queues the callback and rings the terminal."""

    def __init__(self, catalog: Catalog):
        self.vars_from = VapiNotifier(VapiSettings(), catalog)  # same variables as the real Vapi call
        self.pending: queue.Queue = queue.Queue()

    def notify(self, order: Order) -> str:
        variables = self.vars_from.payload(order)["assistantOverrides"]["variableValues"]
        self.pending.put((order, variables))
        print(f"\n\n  ☎  RING RING — the shop is calling {order.phone} about order {order.id}. "
              "Type /answer to pick up.\n> ", end="", flush=True)
        return f"terminal-{order.id}"


# ---------------------------------------------------------------- one phone call with Claude
class Call:
    def __init__(self, client: anthropic.Anthropic, system: str, tools: list[dict] | None, transport,
                 caller: str):
        self.client, self.system, self.tools, self.transport = client, system, tools, transport
        self.caller = caller
        self.call_id = f"local-{uuid.uuid4().hex[:8]}"
        self.messages: list = []

    def say_first(self, text: str) -> None:
        """The assistant speaks first (Vapi firstMessage)."""
        self.messages += [{"role": "user", "content": "(The call connects.)"},
                          {"role": "assistant", "content": text}]

    def _run_tools(self, blocks) -> list[dict]:
        message = {"type": "tool-calls",
                   "call": {"id": self.call_id, "customer": {"number": self.caller}},
                   "toolCallList": [{"id": b.id, "type": "function",
                                     "function": {"name": b.name, "arguments": b.input}} for b in blocks]}
        try:
            results = {r["toolCallId"]: r for r in self.transport.send(message).get("results", [])}
        except Exception as e:  # noqa: BLE001 - server down etc.: let the assistant handle it
            results = {b.id: {"error": f"order system unreachable: {e}"} for b in blocks}
        out = []
        for b in blocks:
            r = results.get(b.id, {"error": "no result returned"})
            print(f"    [tool] {b.name}({b.input}) -> {r.get('result') or r.get('error')}")
            out.append({"type": "tool_result", "tool_use_id": b.id,
                        "content": r.get("result") or r.get("error", ""), "is_error": "error" in r})
        return out

    def turn(self, user_text: str) -> str:
        self.messages.append({"role": "user", "content": user_text})
        while True:
            kwargs = {"tools": self.tools} if self.tools else {}
            response = self.client.messages.create(
                model=MODEL, max_tokens=4000, system=self.system, messages=self.messages,
                output_config={"effort": "low"}, **kwargs)
            self.messages.append({"role": "assistant", "content": response.content})
            if response.stop_reason == "refusal":
                return "(the assistant declined to respond)"
            if response.stop_reason == "tool_use":
                self.messages.append({"role": "user", "content": self._run_tools(
                    [b for b in response.content if b.type == "tool_use"])})
                continue
            if response.stop_reason == "pause_turn":
                continue
            return " ".join(b.text for b in response.content if b.type == "text").strip()


# ---------------------------------------------------------------- REPL
def run_chat(store, catalog: Catalog, packer, caller: str, server: str | None = None) -> None:
    from .worker import Worker

    load_api_key()
    client = anthropic.Anthropic()
    handler = ToolHandler(store, catalog)
    notifier = TerminalCallbackNotifier(catalog)
    worker = None
    if server:
        transport = HttpVapi(server, os.getenv("VAPI_SERVER_SECRET"))
        print(f"Tool calls go to {transport.url} (that server packs orders and places callbacks).")
    else:
        transport = InProcessVapi(handler)
        worker = Worker(store, packer, notifier, poll_s=0.5)
        handler.on_new_order = worker.wake
        worker.start()

    ordering_prompt = (PROMPTS / "ordering.md").read_text()
    callback_prompt = (PROMPTS / "callback.md").read_text()
    callback_first = (PROMPTS / "callback_first_message.txt").read_text().strip()
    tools = anthropic_tools()

    print(f"\nPhone-agent emulator ({MODEL}). You are calling from {caller}.\n{HELP}\n")
    call: Call | None = None
    try:
        while True:
            try:
                line = input("> ").strip()
            except EOFError:
                break
            if not line:
                continue
            if line == "/quit":
                break
            if line == "/help":
                print(HELP)
                continue
            if line == "/orders":
                for o in store.list(limit=10):
                    print(f"  #{o.id} {o.status:<11} {catalog.describe(o.items):<30} {o.phone} {o.customer_name or ''}")
                continue
            if line == "/hangup":
                print("  (call ended)\n")
                call = None
                continue
            if line == "/answer":
                try:
                    order, variables = notifier.pending.get_nowait()
                except queue.Empty:
                    print("  (nobody is calling)")
                    continue
                call = Call(client, render(callback_prompt, variables), None, transport, order.phone)
                first = render(callback_first, variables)
                call.say_first(first)
                print(f"  (callback connected)\nagent: {first}")
                continue
            if call is None:
                call = Call(client, ordering_prompt, tools, transport, caller)
                print("  (calling the shop...)")
            try:
                print(f"agent: {call.turn(line)}")
            except anthropic.APIError as e:
                print(f"  [API error: {e}]")
    finally:
        if worker:
            worker.stop()
