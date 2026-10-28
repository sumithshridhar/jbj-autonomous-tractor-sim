"""Collect results/*/metrics.json into results/SUMMARY.md.   python tools/summarize_results.py"""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RES = ROOT / "results"
ORDER = ["full_field", "cow", "person", "remote_estop", "remote_lost", "rtk_loss", "steering_fault"]
WHAT = {
    "full_field": "Plant the whole 3 ha field (26 passes + 3 headland laps)",
    "cow": "A cow is standing in the path 25 m ahead, later walks away",
    "person": "A person walks across the path 22 m ahead",
    "remote_estop": "Operator presses the handheld e-stop at full planting speed",
    "remote_lost": "The handheld remote's signal drops for 6 s",
    "rtk_loss": "RTK corrections lost for 15 s (fix drops to FLOAT)",
    "steering_fault": "Steering motor jams at 8 deg left while planting (fault injection)",
}


def ev(m, k):
    return next((e for e in m["events"] if e["event"] == k), None)


def row(name, m):
    stops = ", ".join(f"{k.replace('STOP_', '')} x{v}" for k, v in m.get("stops", {}).items()) or "none"
    x = m.get("xte_planting_cm", {})
    acc = f"{x['mean']} / {x['p95']} / {x['max']}" if x else "n/a"
    out = []
    if name == "full_field":
        out.append(f"coverage {m.get('coverage_pct')} % of the field, overlap {m.get('overlap_pct')} %, "
                   f"{m.get('sim_time_s', 0) / 60:.0f} min, {m.get('distance_m', 0) / 1000:.2f} km")
    if "min_gap_to_obstacle_m" in m:
        stop = ev(m, "stopped_for_obstacle") or ev(m, "slowed_for_obstacle")
        out.append(f"closest the bumper got: {m['min_gap_to_obstacle_m']} m (contact: {'YES' if m.get('contact') else 'no'})")
        if stop:
            out.append(f"{stop['event'].replace('_', ' ')} at t={stop['t']} s")
        if ev(m, "resumed"):
            out.append(f"resumed by itself at t={ev(m, 'resumed')['t']} s")
    if "time_to_standstill_s" in m:
        out.append(f"standstill {m['time_to_standstill_s']} s / {m['distance_to_standstill_m']} m after the event")
    if name == "steering_fault":
        out.append(f"max off-line {m.get('max_xte_m')} m, max outside field edge {m.get('max_outside_field_m')} m")
    return f"| {name} | {m['tractor']} | {WHAT.get(name, '')} | {stops} | {'; '.join(out)} | {acc} |"


def main():
    rows, tractor = [], None
    for name in ORDER:
        for d in sorted(RES.glob(f"{name}_*")):
            f = d / "metrics.json"
            if f.exists():
                m = json.loads(f.read_text())
                tractor = tractor or f"{m['tractor']} + {m['implement']}"
                rows.append(row(name, m) + f" [report]({d.name}/report.png) |")
    lines = ["# Results", "",
             "Planter: Kverneland Optima V, 6 rows. Ground truth comes from Gazebo, not from the tractor's own GNSS. "
             "Accuracy = cross-track error while planting, in cm: mean / 95th percentile / max.", "",
             "| Scenario | Tractor | What happens | Safety stops | Outcome | Accuracy (cm) | |",
             "|---|---|---|---|---|---|---|", *rows, ""]
    (RES / "SUMMARY.md").write_text("\n".join(lines), encoding="utf-8")
    print("\n".join(lines))


main()
