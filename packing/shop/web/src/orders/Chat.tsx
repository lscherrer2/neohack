import { useEffect, useRef, useState, type FormEvent } from "react";
import type { ChatReply, ToolCall } from "../types";

const TOOL_WORDS: Record<string, string> = {
  list_products: "checked the products", place_order: "placed the order",
  check_order: "looked up the order", cancel_order: "cancelled the order",
};

type Msg =
  | { kind: "you" | "shop" | "note" | "err" | "typing"; text: string }
  | { kind: "calls"; calls: ToolCall[] };

function newSession(): string {
  const id = (crypto.randomUUID ? crypto.randomUUID() : String(Math.random()).slice(2)).replace(/-/g, "").slice(0, 16);
  sessionStorage.setItem("chat-session", id);
  return id;
}

const INTRO: Msg = { kind: "note", text: "You're the customer. Ask what's in stock or place an order; the robot packs it." };

/** Chat with the ordering agent (POST /api/chat). Messages live in sessionStorage so tab switches keep them. */
export default function Chat() {
  const [sessionId, setSessionId] = useState(() => sessionStorage.getItem("chat-session") || newSession());
  const [msgs, setMsgs] = useState<Msg[]>(() => {
    try { return JSON.parse(sessionStorage.getItem("chat-msgs") || "") as Msg[]; } catch { return [INTRO]; }
  });
  const [text, setText] = useState("");
  const [busy, setBusy] = useState(false);
  const box = useRef<HTMLDivElement>(null);
  const input = useRef<HTMLTextAreaElement>(null);

  useEffect(() => {
    sessionStorage.setItem("chat-msgs", JSON.stringify(msgs.filter((m) => m.kind !== "typing")));
    box.current?.scrollTo({ top: box.current.scrollHeight });
  }, [msgs]);

  const add = (...m: Msg[]) => setMsgs((list) => [...list.filter((x) => x.kind !== "typing"), ...m]);

  async function send(e?: FormEvent) {
    e?.preventDefault();
    const say = text.trim();
    if (!say || busy) return;
    setText("");
    add({ kind: "you", text: say }, { kind: "typing", text: "The shop is typing…" });
    setBusy(true);
    try {
      const r = await fetch("/api/chat", {
        method: "POST", headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ session_id: sessionId, message: say }),
      });
      const data: ChatReply = await r.json().catch(() => ({}));
      if (data.error || !r.ok) add({ kind: "err", text: data.error || `The server answered ${r.status}. Try again.` });
      else add(...(data.tool_calls?.length ? [{ kind: "calls", calls: data.tool_calls } as Msg] : []),
               { kind: "shop", text: data.reply ?? "" });
    } catch {
      add({ kind: "err", text: "Couldn't reach the shop server. Check that it's running, then send again." });
    } finally {
      setBusy(false);
      input.current?.focus();
    }
  }

  function reset() {
    fetch(`/api/chat/${sessionId}/reset`, { method: "POST" }).catch(() => {});
    setSessionId(newSession());
    setMsgs([{ kind: "note", text: "New conversation started." }]);
    input.current?.focus();
  }

  return (
    <section className="counter" aria-labelledby="chat-h">
      <header>
        <h2 id="chat-h">Order counter</h2>
        <button type="button" className="linkish" onClick={reset}>Start a new conversation</button>
      </header>
      <div className="msgs" ref={box} aria-live="polite">
        {msgs.map((m, i) => m.kind === "calls" ? (
          <details key={i} className="calls">
            <summary>The agent {m.calls.map((c) => TOOL_WORDS[c.tool] || c.tool).join(", then ")}</summary>
            <pre>{m.calls.map((c) => `${c.tool}(${JSON.stringify(c.args)})\n→ ${c.result}`).join("\n\n")}</pre>
          </details>
        ) : (
          <div key={i} className={`msg ${m.kind}`}>{m.text}</div>
        ))}
      </div>
      <form className="compose" onSubmit={send}>
        <label htmlFor="say" className="sr-only">Message</label>
        <textarea id="say" ref={input} rows={1} placeholder="Hi, what can I order?" autoComplete="off"
                  value={text} onChange={(e) => setText(e.target.value)}
                  onKeyDown={(e) => { if (e.key === "Enter" && !e.shiftKey) { e.preventDefault(); send(); } }} />
        <button type="submit" disabled={busy}>Send</button>
      </form>
    </section>
  );
}
