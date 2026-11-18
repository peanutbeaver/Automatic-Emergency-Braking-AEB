#!/usr/bin/env python3
"""Plots the CSV time series written by `aeb_sil --csv <dir>`.

    python tools/plot_scenarios.py out/            # one PNG per scenario
    python tools/plot_scenarios.py out/ CCRs-50    # only selected scenarios
"""
import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

STATE_COLORS = {
    "Warning": "#f6c343",
    "PartialBrake": "#f08c3a",
    "FullBrake": "#d64545",
    "Hold": "#7a7a7a",
    "Fault": "#7d3cbf",
}
TTC_PLOT_LIMIT_S = 6.0


def load(path: Path) -> dict[str, list]:
    columns: dict[str, list] = {}
    with path.open() as f:
        for row in csv.DictReader(f):
            for key, value in row.items():
                columns.setdefault(key, []).append(value if key == "state" else float(value))
    return columns


def shade_states(ax, t, states):
    """Colour the background by controller state."""
    start = 0
    for i in range(1, len(states) + 1):
        if i == len(states) or states[i] != states[start]:
            color = STATE_COLORS.get(states[start])
            if color:
                ax.axvspan(t[start], t[i - 1], color=color, alpha=0.18, lw=0)
            start = i


def plot(path: Path) -> Path:
    d = load(path)
    t = d["time_s"]
    fig, (ax_gap, ax_speed, ax_brake) = plt.subplots(3, 1, sharex=True, figsize=(9, 7.5))
    fig.suptitle(f"AEB scenario {path.stem}", fontweight="bold")

    ax_gap.plot(t, d["gap_m"], label="gap [m]", color="#1f5fa8")
    ax_gap.set_ylabel("gap [m]")
    ax_ttc = ax_gap.twinx()
    ax_ttc.plot(t, [min(v, TTC_PLOT_LIMIT_S) for v in d["ttc_s"]], color="#888", lw=1, label="TTC [s]")
    ax_ttc.set_ylabel("TTC [s] (clipped)")
    ax_ttc.set_ylim(0, TTC_PLOT_LIMIT_S)

    ax_speed.plot(t, [v * 3.6 for v in d["ego_mps"]], label="ego", color="#1f5fa8")
    ax_speed.plot(t, [v * 3.6 for v in d["lead_mps"]], label="target", color="#2e9e5b")
    ax_speed.set_ylabel("speed [km/h]")
    ax_speed.legend(loc="upper right")

    ax_brake.plot(t, d["brake_req_mps2"], label="AEB request", color="#d64545")
    ax_brake.plot(t, d["brake_act_mps2"], label="actual (lagged)", color="#222", ls="--")
    ax_brake.set_ylabel("deceleration [m/s²]")
    ax_brake.set_xlabel("time [s]")
    ax_brake.legend(loc="upper right")

    for ax in (ax_gap, ax_speed, ax_brake):
        shade_states(ax, t, d["state"])
        ax.grid(alpha=0.3)

    handles = [plt.Rectangle((0, 0), 1, 1, color=c, alpha=0.4) for c in STATE_COLORS.values()]
    fig.legend(handles, STATE_COLORS.keys(), loc="lower center", ncol=len(STATE_COLORS), frameon=False)
    fig.tight_layout(rect=(0, 0.04, 1, 1))

    out = path.with_suffix(".png")
    fig.savefig(out, dpi=110)
    plt.close(fig)
    return out


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    directory = Path(sys.argv[1])
    wanted = set(sys.argv[2:])
    files = sorted(p for p in directory.glob("*.csv") if not wanted or p.stem in wanted)
    if not files:
        print(f"no matching CSV files in {directory}")
        return 1
    for path in files:
        print("wrote", plot(path))
    return 0


if __name__ == "__main__":
    sys.exit(main())
