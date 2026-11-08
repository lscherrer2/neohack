import { ago } from "./api";
import type { ShopState } from "./types";

/** The machine nameplate: steel = idle, safety yellow = packing, alarm red = offline / failed. */
export default function Plate({ state }: { state: ShopState | null }) {
  let dataState = "loading";
  let title = "Connecting to the shop server";
  let sub = "Loading the robot's status.";

  if (state) {
    const { robot, counts, auto_dispatch } = state;
    dataState = robot.state;
    if (robot.state === "packing") {
      title = `Packing order ${robot.order.id}`;
      sub = `${robot.items} for ${robot.order.customer_name || "a customer"}. Last heard from the robot ${ago(robot.last_seen_s)}.`;
    } else if (robot.state === "idle") {
      title = "Idle, waiting for orders";
      sub = `Robot agent is connected (last check-in ${ago(robot.last_seen_s)}).`;
      const queued = counts.queued ?? 0;
      if (queued && !auto_dispatch)
        sub = `${queued} received order${queued > 1 ? "s are" : " is"} waiting. Click Action to send one to the robot.`;
      if (robot.last_failure) {
        dataState = "failed";
        title = `Order ${robot.last_failure.id} failed`;
        sub = `${robot.last_failure.error || "Unknown error"}. Check the arm, then restart the robot agent.`;
      }
    } else {
      title = "Robot agent offline";
      sub = robot.last_seen_s == null
        ? "It hasn't connected yet. Start it with python -m packing.shop robot"
        : `Last heard from it ${ago(robot.last_seen_s)}. Restart it with python -m packing.shop robot`;
    }
  }

  return (
    <div className="plate" data-state={dataState} role="status" aria-live="polite">
      <h1>{title}</h1>
      <p>{sub}</p>
    </div>
  );
}
