import { useEffect, useRef } from "react";

/** Modal confirm (native <dialog>). Rendered only while open. */
export default function ConfirmDialog({ title, body, confirmLabel, onConfirm, onCancel }: {
  title: string; body: string; confirmLabel: string; onConfirm: () => void; onCancel: () => void;
}) {
  const ref = useRef<HTMLDialogElement>(null);
  useEffect(() => {
    const d = ref.current;
    if (d && !d.open) d.showModal();
  }, []);
  return (
    <dialog ref={ref} className="confirm" onCancel={(e) => { e.preventDefault(); onCancel(); }}>
      <h2>{title}</h2>
      <p>{body}</p>
      <div className="confirm-btns">
        <button type="button" className="linkish" onClick={onCancel}>Cancel</button>
        <button type="button" className="big warn" autoFocus onClick={onConfirm}>{confirmLabel}</button>
      </div>
    </dialog>
  );
}
