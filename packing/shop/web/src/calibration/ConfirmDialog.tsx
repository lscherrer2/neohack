import { useEffect, useLayoutEffect, useRef, useState } from "react";

// The button that opened a dialog: the last button pressed (pointer or keyboard). Dialogs often open after an
// async step (plan first, then confirm), so remember it at press time rather than threading refs everywhere.
let lastTrigger: HTMLElement | null = null;
const remember = (e: Event) => {
  const el = (e.target as HTMLElement | null)?.closest?.("button, [role=button], [role=radio]") as HTMLElement | null;
  if (el) lastTrigger = el;
};
if (typeof window !== "undefined") {
  window.addEventListener("pointerdown", remember, true);
  window.addEventListener("keydown", (e) => { if (e.key === "Enter" || e.key === " ") remember(e); }, true);
}

const GAP = 8; // px between the button and the dialog
const EDGE = 12; // px kept free at the window edges

/** Fixed position just below `anchor`, right edges aligned; above it if there's no room below; inside the window. */
function placeNear(anchor: DOMRect, w: number, h: number): { top: number; left: number } {
  const vw = window.innerWidth, vh = window.innerHeight;
  let left = anchor.right - w;
  left = Math.min(Math.max(left, EDGE), vw - w - EDGE);
  let top = anchor.bottom + GAP;
  if (top + h > vh - EDGE) top = anchor.top - GAP - h; // flip above
  top = Math.min(Math.max(top, EDGE), Math.max(EDGE, vh - h - EDGE));
  return { top, left };
}

/**
 * Modal confirm (native <dialog>: backdrop, focus trap, Esc = cancel). Rendered only while open. It appears next
 * to the button that triggered it (centered if that button is gone).
 */
export default function ConfirmDialog({ title, body, confirmLabel, onConfirm, onCancel }: {
  title: string; body: string; confirmLabel: string; onConfirm: () => void; onCancel: () => void;
}) {
  const ref = useRef<HTMLDialogElement>(null);
  const anchor = useRef<HTMLElement | null>(lastTrigger);
  const [pos, setPos] = useState<{ top: number; left: number } | null>(null);

  useLayoutEffect(() => {
    const d = ref.current;
    if (!d) return;
    if (!d.open) d.showModal();
    const place = () => {
      const a = anchor.current;
      if (!a || !a.isConnected) { setPos(null); return; }
      const r = d.getBoundingClientRect();
      setPos(placeNear(a.getBoundingClientRect(), r.width, r.height));
    };
    place();
    window.addEventListener("resize", place);
    window.addEventListener("scroll", place, true);
    return () => {
      window.removeEventListener("resize", place);
      window.removeEventListener("scroll", place, true);
    };
  }, []);
  useEffect(() => () => { anchor.current?.focus?.(); }, []); // focus back on the trigger when closed

  return (
    <dialog ref={ref} className={`confirm${pos ? " anchored" : ""}`} style={pos ?? undefined}
            onCancel={(e) => { e.preventDefault(); onCancel(); }}>
      <h2>{title}</h2>
      <p>{body}</p>
      <div className="confirm-btns">
        <button type="button" className="linkish" onClick={onCancel}>Cancel</button>
        <button type="button" className="big warn" autoFocus onClick={onConfirm}>{confirmLabel}</button>
      </div>
    </dialog>
  );
}
