# neohack — SO-101 packing cell

A single SO-101 arm (Hiwonder kit, Feetech STS3215 servos, no leader arm) that packs parts into boxes,
tracks inventory, and takes orders. Talks to the servos directly over the Feetech SDK — no LeRobot.

## Setup
```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
source .venv/bin/activate
```
Arm on USB: `ls /dev/cu.usbmodem*` and set `robot.port` in `packing/cell.yaml`.

## Calibration (once per arm)
Torque stays off; you move the arm by hand. Values go to the servos and `packing/calibration/<robot id>.json`.
```bash
python -m packing.tools.calibrate scan      # read-only check: all 6 servos answer?
python -m packing.tools.calibrate backup
python -m packing.tools.calibrate center    # hold arm in the middle pose (L-shape, wrist straight)
python -m packing.tools.calibrate range 60  # move every joint end to end during the 60 s
```

## Use
```bash
python -m packing where            # joint values + tip position
python -m packing teach-all        # teach the cell's poses with the keyboard
python -m packing pack             # package boxes
python -m packing.shop start       # order server + dashboard (http://localhost:8000/dashboard)
python -m packing.remote           # MCP server for LLM control
```
Add `--sim` to run against a simulated arm. Tests: `python -m unittest discover -s packing/tests -t .`

Details: `packing/README.md`, `packing/shop/README.md`, `CLAUDE.md`.
