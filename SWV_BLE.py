#have python 3.9 or above 
# to see which version you have use 'which python'
#or use 'python --version'
#to download a specific version use pip install python==version u want 
# to delete old just do update or pip uninstall python 
# how to run the code (copy n paste the bottom into terminal !)
#install the following as well if u dont have it 
#pip install numpy matplotlib scipy bleak
##
#additional for windows
#pip install bleak[winrt]
##
#cd ~/desktop/SWV\ python\ +\ ino\ files\ +\ Data/
#python SWV_BLE.py run 
#written by me jose 
#below is written way nicer by chat lol 

"""
============================================================
BLE Square-Wave Voltammetry (SWV) Streaming & Analysis Script
============================================================

PURPOSE
-------
This script connects to an ESP32-based amperometric SWV board over Bluetooth Low Energy (BLE),
streams current data in real time, applies Excel/PalmSens-style filtering, and visualizes the
results live while logging everything to CSV files.

The script is designed for electrochemical experiments involving:
- A baseline measurement
- One or more solute additions
- Board resets between additions (via unplug / replug)

Each board reconnection is treated as a new experimental "segment".


EXPERIMENTAL WORKFLOW
---------------------
1. Start the script.
2. Plug in the board → BLE turns on → Segment 1 (baseline) begins.
3. Data streams continuously and is plotted live.
4. Unplug the board to reset it.
5. Add solute to the solution.
6. Plug the board back in → Segment 2 begins.
7. Repeat steps 4–6 for additional solute additions.
8. Stop the script with Ctrl+C when finished.

Segments are automatically labeled using `segment_id`:
- segment_id = 1 → baseline
- segment_id = 2 → first solute addition
- segment_id = 3 → second solute addition
- etc.


DATA FORMAT (BLE INPUT)
-----------------------
The firmware is expected to stream ASCII lines of the form:

    idx:<n>, step:<m>, phase:<F/R>, I_uA:<float>

Where:
- idx   : firmware sample index
- step  : SWV step index within a sweep
- phase : 'F' (forward) or 'R' (reverse)
- I_uA  : measured current in microamps


SIGNAL PROCESSING / FILTERING
-----------------------------

1) Differential current:
       Idiff = IF - IR

2) Average by sweep (NEW behavior):
   - A "sweep" is detected when `step` decreases (e.g., 512 → 0).
   - For each sweep, repeated measurements of the same step are averaged.
   - Averaging resets at the start of each new sweep.

3) Savitzky–Golay filtering:
   - Applied to the averaged Idiff vs step curve
   - Parameters:
         window_length = SG_WINDOW (odd)
         polyorder     = SG_ORDER
         mode          = 'interp'
   - SG is recomputed on the full averaged series for that sweep.


LIVE PLOTS
----------
Three persistent plots are shown and remain open for the entire session:

1) Raw IF / IR vs step
2) Averaged Idiff (by sweep) vs step
3) Savitzky–Golay filtered Idiff vs step

- Each experimental segment (baseline, solute additions, etc.) is overlaid on the same plots.
- X-axis is inverted to visually match PalmSens plotting direction.
- Axis limits are set dynamically:
      xlim = [min(step) - 0, max(step) + 0]
      ylim = [min(current) , max(current) ]


OUTPUT FILES (CSV)
------------------
Three CSV files are written continuously for the entire experiment:

1) RAW:
   ble_stream_RAW_ALL.csv
   Columns:
     segment_id, sweep_id, pc_timestamp, idx, step, phase, I_uA, raw_line

2) DIFFERENTIAL:
   ble_stream_DIFF_ALL.csv
   Columns:
     segment_id, sweep_id, pc_timestamp, step, IF_uA, IR_uA, Idiff_raw_uA

3) FILTERED (AVERAGED + SG):
   ble_stream_SG_ALL.csv
   Columns:
     segment_id, sweep_id, pc_timestamp, step, Idiff_avg_uA, Idiff_savgol_uA

These files can be directly loaded into Python, MATLAB, or Excel for post-analysis.


DEPENDENCIES
------------
Required Python packages:
    - numpy
    - matplotlib
    - scipy
    - bleak

Install with:
    pip install numpy matplotlib scipy bleak

Python version:
    Python 3.9 or newer recommended.


NOTES / ASSUMPTIONS
-------------------
- BLE disconnects are intentional (used to reset the board).
- Each BLE reconnection automatically starts a new experimental segment.
- Step values are assumed to monotonically increase during a sweep.
- SG filtering is centered.


AUTHOR / CONTEXT
----------------
Developed for real-time electrochemical biosensing experiments using an
ESP32-based amperometric SWV platform.

NBEEEEEEEE
============================================================
"""


import asyncio
import csv
import re
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from bleak import BleakScanner, BleakClient

# ============================================================
# USER SETTINGS
# ============================================================

TARGET_NAME = "ESP32_AMPEROMETRIC"
NOTIFY_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

# Single continuous CSVs for the whole experiment (tagged by segment_id and sweep_id)
CSV_RAW  = Path("ble_stream_RAW_ALL-td.csv")
CSV_DIFF = Path("ble_stream_DIFF_ALL-td.csv")
CSV_SG   = Path("ble_stream_SG_ALL-td.csv")

WINDOW_STEPS = 1500 
PLOT_UPDATE_HZ = 12 

# Excel-style SG params (applied to averaged series)
SG_WINDOW = 25   # must be odd
SG_ORDER  = 2

# Dynamic axis padding (your margins)
X_MARGIN_STEPS = 0
Y_MARGIN_UA = 0.0

SCAN_TIMEOUT_S = 5.0

# ============================================================
# Incoming firmware format
# ============================================================

LINE_RE = re.compile(
    r"idx\s*:\s*(\d+)\s*,\s*"
    r"step\s*:\s*(\d+)\s*,\s*"
    r"phase\s*:\s*([FR])\s*,\s*"
    r"I_uA\s*:\s*([+-]?\d*\.?\d+(?:[eE][+-]?\d+)?)"
)

def safe_decode(b: bytearray) -> str:
    return bytes(b).decode("utf-8", errors="ignore")

# ============================================================
# BLE scanning (reconnect loop)
# ============================================================

async def wait_for_target():
    """Keep scanning until the board appears (plugged in)."""
    round_i = 0
    while True:
        round_i += 1
        devices = await BleakScanner.discover(timeout=SCAN_TIMEOUT_S)
        for d in devices:
            if (d.name or "").strip() == TARGET_NAME:
                print(f"[FOUND] ✅ {TARGET_NAME} @ {d.address}")
                return d.address
        print(f"[SCAN] Not found (round {round_i}); waiting...")

# ============================================================
# Savitzky–Golay (Excel-style)
# ============================================================

def recompute_savgol(y_values):
    """Centered SG on entire averaged series (mode='interp')."""
    if len(y_values) < SG_WINDOW:
        return None
    return savgol_filter(np.asarray(y_values, dtype=float),
                         window_length=SG_WINDOW,
                         polyorder=SG_ORDER,
                         mode="interp")

# ============================================================
# Segment state (overlay + filtering state)
# ============================================================

def new_segment_state(seg_id: int):
    """
    All per-segment buffers are stored so overlays persist.
    Averaging is now PER SWEEP: we reset step_to_diffs on each new sweep.
    """
    return {
        "segment_id": seg_id,
        "label": f"seg {seg_id:04d}",

        # --- Plot buffers (rolling) ---
        "step": deque(maxlen=WINDOW_STEPS),
        "IF": deque(maxlen=WINDOW_STEPS),
        "IR": deque(maxlen=WINDOW_STEPS),

        "avg_step": deque(maxlen=WINDOW_STEPS),
        "avg_diff": deque(maxlen=WINDOW_STEPS),

        "sg_step": deque(maxlen=WINDOW_STEPS),
        "sg": deque(maxlen=WINDOW_STEPS),

        # --- Pairing F/R within a step ---
        "pending": {},  # step -> {"F":..., "R":...}

        # --- Sweep detection + per-sweep averaging ---
        "sweep_id": 1,
        "prev_step_seen": None,     # used to detect step reset
        "step_to_diffs": {},        # step -> list[Idiff] (RESET each sweep)

        # --- Line handles (set later) ---
        "lIF": None,
        "lIR": None,
        "lDiff": None,
        "lSG": None,
    }

def start_new_sweep(seg_state: dict):
    """Reset per-sweep averaging state (average-by-sweep behavior)."""
    seg_state["sweep_id"] += 1
    seg_state["step_to_diffs"].clear()

    # NOTE: we do NOT clear plot buffers; we want continuous traces.
    # If you want each sweep separated visually, we could add NaNs to break lines.

# ============================================================
# Dynamic axis limits (min/max across overlays + margins)
# ============================================================

def update_axes_limits(ax1, ax2, ax3, all_segments):
    # X: consider all step-like buffers
    x_min = None
    x_max = None
    for seg in all_segments:
        for buf in (seg["step"], seg["avg_step"], seg["sg_step"]):
            if not buf:
                continue
            mn = min(buf); mx = max(buf)
            x_min = mn if x_min is None else min(x_min, mn)
            x_max = mx if x_max is None else max(x_max, mx)

    if x_min is not None and x_max is not None:
        ax1.set_xlim(x_min - X_MARGIN_STEPS, x_max + X_MARGIN_STEPS)
        ax2.set_xlim(x_min - X_MARGIN_STEPS, x_max + X_MARGIN_STEPS)
        ax3.set_xlim(x_min - X_MARGIN_STEPS, x_max + X_MARGIN_STEPS)

    # Plot 1 Y: IF/IR
    y1_min = None
    y1_max = None
    for seg in all_segments:
        for buf in (seg["IF"], seg["IR"]):
            if not buf:
                continue
            mn = min(buf); mx = max(buf)
            y1_min = mn if y1_min is None else min(y1_min, mn)
            y1_max = mx if y1_max is None else max(y1_max, mx)
    if y1_min is not None and y1_max is not None:
        ax1.set_ylim(y1_min - Y_MARGIN_UA, y1_max + Y_MARGIN_UA)

    # Plot 2 Y: avg diff
    y2_min = None
    y2_max = None
    for seg in all_segments:
        buf = seg["avg_diff"]
        if not buf:
            continue
        mn = min(buf); mx = max(buf)
        y2_min = mn if y2_min is None else min(y2_min, mn)
        y2_max = mx if y2_max is None else max(y2_max, mx)
    if y2_min is not None and y2_max is not None:
        ax2.set_ylim(y2_min - Y_MARGIN_UA, y2_max + Y_MARGIN_UA)

    # Plot 3 Y: SG
    y3_min = None
    y3_max = None
    for seg in all_segments:
        buf = seg["sg"]
        if not buf:
            continue
        mn = min(buf); mx = max(buf)
        y3_min = mn if y3_min is None else min(y3_min, mn)
        y3_max = mx if y3_max is None else max(y3_max, mx)
    if y3_min is not None and y3_max is not None:
        ax3.set_ylim(y3_min - Y_MARGIN_UA, y3_max + Y_MARGIN_UA)

# ============================================================
# Main
# ============================================================

async def main():
    # Validate SG params
    if SG_WINDOW % 2 == 0:
        raise ValueError("SavGol window must be odd.")
    if SG_ORDER >= SG_WINDOW:
        raise ValueError("SavGol order must be < window length.")

    # -----------------------------
    # CSVs opened ONCE for whole experiment
    # -----------------------------
    raw_f  = open(CSV_RAW, "w", newline="")
    diff_f = open(CSV_DIFF, "w", newline="")
    sg_f   = open(CSV_SG, "w", newline="")

    raw_w  = csv.writer(raw_f)
    diff_w = csv.writer(diff_f)
    sg_w   = csv.writer(sg_f)

    # Added sweep_id columns so you can separate sweeps later if needed
    raw_w.writerow(["segment_id","sweep_id","pc_timestamp","idx","step","phase","I_uA","raw_line"])
    diff_w.writerow(["segment_id","sweep_id","pc_timestamp","step","IF_uA","IR_uA","Idiff_raw_uA"])
    sg_w.writerow(["segment_id","sweep_id","pc_timestamp","step","Idiff_avg_uA","Idiff_savgol_uA"])

    # -----------------------------
    # Persistent plots (overlay)
    # -----------------------------
    plt.ion()

    fig1, ax1 = plt.subplots()
    ax1.set_title("IF / IR (raw per completed step) — overlays by segment")
    ax1.set_xlabel("Step"); ax1.set_ylabel("Current (uA)")
    ax1.grid(True)

    fig2, ax2 = plt.subplots()
    ax2.set_title("Idiff averaged BY SWEEP (by step) — overlays by segment")
    ax2.set_xlabel("Step"); ax2.set_ylabel("Current (uA)")
    ax2.grid(True)

    fig3, ax3 = plt.subplots()
    ax3.set_title("SavGol on Idiff avg (BY SWEEP) — overlays by segment")
    ax3.set_xlabel("Step"); ax3.set_ylabel("Current (uA)")
    ax3.grid(True)

    # Option A: PalmSens visual direction uncommment for flipped version 
    #ax1.invert_xaxis()
    #ax2.invert_xaxis()
    #ax3.invert_xaxis()

    all_segments = []
    segment_id = 0

    print("[SYSTEM] Ready. Plug board in to start baseline (segment 1).")

    try:
        while True:
            # Wait for board to appear, then connect => new segment
            address = await wait_for_target()
            segment_id += 1

            seg_state = new_segment_state(segment_id)
            all_segments.append(seg_state)

            # Create overlay lines and store them in the segment state
            # (OPTIMIZE: switch "-o" -> "-" if it gets slow)
            seg_state["lIF"],  = ax1.plot([], [], "-o", markersize=3, linewidth=1.2,
                                         label=f"IF {seg_state['label']}")
            seg_state["lIR"],  = ax1.plot([], [], "-o", markersize=3, linewidth=1.2,
                                         label=f"IR {seg_state['label']}")
            seg_state["lDiff"],= ax2.plot([], [], "-o", markersize=3, linewidth=1.5,
                                         label=f"Idiff {seg_state['label']}")
            seg_state["lSG"],  = ax3.plot([], [], "-o", markersize=3, linewidth=1.8,
                                         label=f"SG {seg_state['label']}")

            ax1.legend(); ax2.legend(); ax3.legend()

            # Per-connection parsing buffer + disconnect flag
            text_buffer = ""
            disconnected = asyncio.Event()

            def on_disconnect(_client):
                disconnected.set()

            def on_notify(_handle, data: bytearray):
                nonlocal text_buffer
                text_buffer += safe_decode(data)

                while "\n" in text_buffer:
                    line, text_buffer = text_buffer.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue

                    ts = datetime.now().isoformat(timespec="milliseconds")
                    m = LINE_RE.search(line)

                    if not m:
                        raw_w.writerow([seg_state["segment_id"], seg_state["sweep_id"], ts,
                                        None, None, None, None, line])
                        raw_f.flush()
                        continue

                    idx = int(m.group(1))
                    step = int(m.group(2))
                    phase = m.group(3)
                    I = float(m.group(4))

                    # --- Detect new sweep (step reset) ---
                    prev = seg_state["prev_step_seen"]
                    if prev is not None and step < prev:
                        # step wrapped back -> new sweep starts
                        start_new_sweep(seg_state)
                        print(f"[SWEEP] {seg_state['label']} -> sweep {seg_state['sweep_id']} (step reset {prev} -> {step})")
                    seg_state["prev_step_seen"] = step

                    # RAW CSV
                    raw_w.writerow([seg_state["segment_id"], seg_state["sweep_id"], ts,
                                    idx, step, phase, I, line])
                    raw_f.flush()

                    # Pair F/R
                    pending = seg_state["pending"]
                    pending.setdefault(step, {})
                    if phase not in pending[step]:
                        pending[step][phase] = I

                    if "F" in pending[step] and "R" in pending[step]:
                        IF = pending[step]["F"]
                        IR = pending[step]["R"]
                        Idiff = IF - IR

                        # DIFF CSV
                        diff_w.writerow([seg_state["segment_id"], seg_state["sweep_id"], ts,
                                         step, IF, IR, Idiff])
                        diff_f.flush()

                        # Plot 1 buffers
                        seg_state["step"].append(step)
                        seg_state["IF"].append(IF)
                        seg_state["IR"].append(IR)

                        # ============================================================
                        # AVERAGE BY SWEEP (NEW)
                        # - Only average repeats within THIS sweep
                        # - step_to_diffs resets when a new sweep is detected
                        # ============================================================
                        step_to_diffs = seg_state["step_to_diffs"]
                        step_to_diffs.setdefault(step, []).append(Idiff)
                        Idiff_avg = float(np.mean(step_to_diffs[step]))

                        # Maintain averaged series buffers (one point per step)
                        if seg_state["avg_step"] and seg_state["avg_step"][-1] == step:
                            seg_state["avg_diff"][-1] = Idiff_avg
                        else:
                            seg_state["avg_step"].append(step)
                            seg_state["avg_diff"].append(Idiff_avg)

                        # SG on averaged series (centered, interp)
                        y_sg = recompute_savgol(list(seg_state["avg_diff"]))
                        Idiff_sg = float(y_sg[-1]) if y_sg is not None else None

                        if y_sg is not None:
                            seg_state["sg_step"].clear()
                            seg_state["sg"].clear()
                            seg_state["sg_step"].extend(seg_state["avg_step"])
                            seg_state["sg"].extend(y_sg.tolist())

                        # SG CSV
                        sg_w.writerow([seg_state["segment_id"], seg_state["sweep_id"], ts,
                                       step, Idiff_avg, Idiff_sg])
                        sg_f.flush()

                        pending.pop(step, None)

            # -----------------------------
            # Connect and stream until disconnect
            # -----------------------------
            print(f"[CONNECT] Connecting to {address} -> {seg_state['label']} (baseline/solute run)")
            async with BleakClient(address, disconnected_callback=on_disconnect) as client:
                print(f"[CONNECT] ✅ Connected ({seg_state['label']})")
                await client.start_notify(NOTIFY_CHAR_UUID, on_notify)
                print("[STREAM] Streaming... (unplug board to reset / start next segment)")

                dt = 1.0 / PLOT_UPDATE_HZ

                while not disconnected.is_set():
                    # Update each segment’s lines (true overlay)
                    for seg in all_segments:
                        if seg["step"]:
                            seg["lIF"].set_data(list(seg["step"]), list(seg["IF"]))
                            seg["lIR"].set_data(list(seg["step"]), list(seg["IR"]))
                        if seg["avg_step"]:
                            seg["lDiff"].set_data(list(seg["avg_step"]), list(seg["avg_diff"]))
                        if seg["sg_step"] and seg["sg"]:
                            seg["lSG"].set_data(list(seg["sg_step"]), list(seg["sg"]))

                    # Dynamic axis limits using your margins
                    update_axes_limits(ax1, ax2, ax3, all_segments)

                    for fig in (fig1, fig2, fig3):
                        fig.canvas.draw_idle()
                        fig.canvas.flush_events()

                    await asyncio.sleep(dt)

                # Stop notifications gracefully (may already be disconnected)
                try:
                    await client.stop_notify(NOTIFY_CHAR_UUID)
                except Exception:
                    pass

            print("[DISCONNECT] Board disconnected. Add solute, then plug back in for next segment.")

    finally:
        # Ensure CSVs close on exit
        raw_f.close()
        diff_f.close()
        sg_f.close()
        plt.ioff()
        plt.close("all")

# Entry point
if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[EXIT] Stopped")

