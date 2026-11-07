"""Terminal ordering agent: Claude Haiku using the shop server's tools over MCP.

    python -m packing.shop chat [--server http://localhost:8000]

Stands in for a future voice agent: it only knows the tools the MCP server lists, and calls them over
MCP. A background watcher prints a notice when an order changes to packed/failed.
"""

from __future__ import annotations

import asyncio
import os
import re
from pathlib import Path

import anthropic
import httpx
from mcp import Client

MODEL = "claude-haiku-5-5"
PROMPT = (Path(__file__).parent / "ordering_prompt.md").read_text()
REPO_ROOT = Path(__file__).resolve().parents[2]
HELP = "commands: /new (start a new conversation)  /orders  /quit"


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


async def watch_orders(server: str, stop: asyncio.Event) -> None:
    """Announce status changes of orders created after the chat started."""
    seen: dict[int, str] = {}
    first = True
    async with httpx.AsyncClient(base_url=server, timeout=5) as http:
        while not stop.is_set():
            try:
                for o in (await http.get("/orders", params={"limit": 50})).json():
                    old = seen.get(o["id"])
                    seen[o["id"]] = o["status"]
                    if not first and old != o["status"] and o["status"] in ("packed", "failed"):
                        word = "is packed and ready" if o["status"] == "packed" else "FAILED to pack"
                        print(f"\n  🔔 order {o['id']} ({o['customer_name']}) {word}\n> ", end="", flush=True)
                first = False
            except httpx.HTTPError:
                pass
            try:
                await asyncio.wait_for(stop.wait(), 2.0)
            except asyncio.TimeoutError:
                pass


class Conversation:
    def __init__(self, claude: anthropic.AsyncAnthropic, mcp: Client | None, tools: list[dict], verbose: bool = True):
        self.claude, self.mcp, self.tools, self.verbose = claude, mcp, tools, verbose
        self.messages: list = []
        self.last_calls: list[dict] = []  # tool calls made during the latest turn

    async def _call(self, block) -> dict:
        try:
            res = await self.mcp.call_tool(block.name, block.input)
            text = " ".join(getattr(c, "text", "") for c in res.content).strip()
            is_error = bool(res.is_error)
        except Exception as e:  # noqa: BLE001 - server down etc.; let the model handle it
            text, is_error = f"order system unreachable: {e}", True
        self.last_calls.append({"tool": block.name, "args": block.input, "result": text, "error": is_error})
        if self.verbose:
            print(f"    [mcp] {block.name}({block.input}) -> {text}")
        return {"type": "tool_result", "tool_use_id": block.id, "content": text, "is_error": is_error}

    async def turn(self, user_text: str) -> str:
        self.last_calls = []
        self.messages.append({"role": "user", "content": user_text})
        while True:
            response = await self.claude.messages.create(
                model=MODEL, max_tokens=4000, system=PROMPT, tools=self.tools, messages=self.messages,
                output_config={"effort": "low"})
            self.messages.append({"role": "assistant", "content": response.content})
            if response.stop_reason == "refusal":
                return "(the assistant declined to respond)"
            if response.stop_reason == "tool_use":
                calls = [b for b in response.content if b.type == "tool_use"]
                results = await asyncio.gather(*(self._call(b) for b in calls))
                self.messages.append({"role": "user", "content": list(results)})
                continue
            if response.stop_reason == "pause_turn":
                continue
            return " ".join(b.text for b in response.content if b.type == "text").strip()


async def chat(server: str) -> None:
    load_api_key()
    claude = anthropic.AsyncAnthropic()
    server = server.rstrip("/")
    async with Client(f"{server}/mcp") as mcp:
        listed = await mcp.list_tools()
        tools = [{"name": t.name, "description": t.description or "", "input_schema": t.input_schema}
                 for t in listed.tools]
        print(f"\nOrdering agent ({MODEL}) connected to {server}/mcp — tools: {', '.join(t['name'] for t in tools)}")
        print(HELP + "\n")
        stop = asyncio.Event()
        watcher = asyncio.create_task(watch_orders(server, stop))
        convo = Conversation(claude, mcp, tools)
        try:
            while True:
                try:
                    line = (await asyncio.to_thread(input, "> ")).strip()
                except EOFError:
                    break
                if not line:
                    continue
                if line == "/quit":
                    break
                if line == "/new":
                    convo = Conversation(claude, mcp, tools)
                    print("  (new conversation)")
                    continue
                if line == "/orders":
                    async with httpx.AsyncClient(base_url=server, timeout=5) as http:
                        for o in (await http.get("/orders", params={"limit": 10})).json():
                            print(f"  #{o['id']} {o['status']:<10} {o['customer_name'] or '':<10} {o['items']}")
                    continue
                if line == "/help":
                    print(HELP)
                    continue
                try:
                    print(f"agent: {await convo.turn(line)}")
                except anthropic.APIError as e:
                    print(f"  [API error: {e}]")
        finally:
            stop.set()
            await watcher
