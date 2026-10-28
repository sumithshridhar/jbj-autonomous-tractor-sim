"""Plots for one run: top view (plan vs driven, obstacle), speed/state timeline, cross-track error histogram."""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

STATE_COLOURS = {"RUN": "#1a7f37", "SLOW": "#d4a017", "DONE": "#555", "WAIT": "#999"}


def plot_run(run_dir, path, field, rows, m):
    t = np.array([r[0] for r in rows])
    gx, gy, gv = (np.array([r[i] for r in rows], float) for i in (1, 2, 4))
    xte = np.array([r[17] for r in rows], float)
    work = np.array([r[14] for r in rows], float)
    states = [r[10] for r in rows]
    fig = plt.figure(figsize=(14, 8.5))
    ax = fig.add_axes([0.04, 0.08, 0.55, 0.84])
    fx, fy = field.exterior.xy
    ax.fill(fx, fy, color="#d9c3a5", zorder=0)
    ax.plot(path[:, 0], path[:, 1], color="#999", lw=0.6, ls="--", label="plan (rear axle)")
    pw = np.column_stack([gx, gy]).astype(float)
    pw[work < 0.5] = np.nan
    ax.plot(gx, gy, color="#555", lw=0.8, label="driven")
    ax.plot(pw[:, 0], pw[:, 1], color="#1a7f37", lw=1.3, label="driven, planter down")
    stop_idx = [i for i, s in enumerate(states) if s and s.startswith("STOP_")]
    if stop_idx:
        ax.plot(gx[stop_idx], gy[stop_idx], "x", color="#c00", ms=7, label="stopped")
    ox = [r[19] for r in rows if r[19] != ""]
    if ox:
        ax.plot(float(ox[-1]), float([r[20] for r in rows if r[20] != ""][-1]), "s", color="#7a3b12", ms=9, label="obstacle")
    if m["scenario"] != "full_field":
        pad = 25
        ax.set_xlim(np.nanmin(gx) - pad, np.nanmax(gx) + pad)
        ax.set_ylim(np.nanmin(gy) - pad, np.nanmax(gy) + pad)
    ax.set_aspect("equal")
    ax.legend(loc="upper right", fontsize=8)
    ax.set_title(f"{m['scenario']}: {m['tractor']} + {m['implement']}")
    ax.set_xlabel("east (m)")
    ax.set_ylabel("north (m)")
    a2 = fig.add_axes([0.65, 0.58, 0.32, 0.34])
    a2.plot(t, gv * 3.6, color="#222", lw=1)
    for i in range(len(t) - 1):
        s = states[i] or ""
        col = STATE_COLOURS.get(s, "#c00" if s.startswith("STOP_") else None)
        if col and s != "RUN":
            a2.axvspan(t[i], t[i + 1], color=col, alpha=0.25, lw=0)
    for e in m.get("events", []):
        a2.axvline(e["t"], color="#555", lw=0.6, ls=":")
        a2.text(e["t"], a2.get_ylim()[1] * 0.98 if a2.get_ylim()[1] > 0 else 1, e["event"], rotation=90, fontsize=6, va="top")
    a2.set_ylabel("speed (km/h)")
    a2.set_xlabel("time (s)   red = stopped, yellow = slow")
    a3 = fig.add_axes([0.65, 0.08, 0.32, 0.36])
    sel = (work > 0.5) & (gv > 0.3)
    if np.any(sel):
        a3.hist(100 * xte[sel], bins=60, color="#1a7f37")
        a3.set_xlabel("cross-track error while planting (cm, ground truth)")
        x = m.get("xte_planting_cm", {})
        a3.set_title(f"mean {x.get('mean')} cm, 95% < {x.get('p95')} cm, max {x.get('max')} cm", fontsize=9)
    fig.savefig(f"{run_dir}/report.png", dpi=100)
    plt.close(fig)
