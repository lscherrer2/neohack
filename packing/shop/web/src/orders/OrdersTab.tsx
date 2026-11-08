import type { TrayLoad } from "../App";
import type { ShopEvent, ShopState } from "../types";
import Activity from "./Activity";
import Chat from "./Chat";
import OrdersTable from "./OrdersTable";
import Shelf from "./Shelf";
import TrayMap from "./TrayMap";

interface Props {
  state: ShopState | null;
  refreshState: () => void;
  events: { ev: ShopEvent; fresh: boolean }[];
  trayLoad: TrayLoad;
}

export default function OrdersTab({ state, refreshState, events, trayLoad }: Props) {
  return (
    <main className="orders-grid">
      <Chat />
      <div className="stack">
        <Shelf stock={state?.stock ?? null} refreshState={refreshState} />
        <TrayMap trayLoad={trayLoad} />
        <OrdersTable state={state} refreshState={refreshState} />
        <Activity events={events} />
      </div>
    </main>
  );
}
