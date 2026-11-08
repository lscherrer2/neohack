import { clock } from "../api";
import type { ShopEvent } from "../types";

export default function Activity({ events }: { events: { ev: ShopEvent; fresh: boolean }[] }) {
  return (
    <section aria-labelledby="events-h">
      <header>
        <h2 id="events-h">Activity</h2>
        <span className="sub">Newest first. Click an entry for details.</span>
      </header>
      <ol className="events">
        {!events.length && <li className="system"><span /><span className="sub">Waiting for activity.</span></li>}
        {events.map(({ ev, fresh }) => {
          const d = ev.data || {};
          const declined = ev.kind === "tool" && d.ok === false;
          const result = d.result ? <div className="result">{String(d.result)}</div> : null;
          const tag = declined ? <span className="tag">declined</span> : null;
          return (
            // The kind is the class name (tool/robot/order/chat/error/system): don't reuse those names elsewhere.
            <li key={ev.seq} className={`${ev.kind}${declined ? " declined" : ""}${fresh ? " fresh" : ""}`}>
              <time>{clock(ev.ts)}</time>
              <div>
                {Object.keys(d).length ? (
                  <details>
                    <summary>{ev.text}{tag}{result}</summary>
                    <pre>{JSON.stringify(d, null, 2)}</pre>
                  </details>
                ) : <div>{ev.text}</div>}
              </div>
            </li>
          );
        })}
      </ol>
    </section>
  );
}
