#!/usr/bin/env python3

import argparse
import re
import shutil
import subprocess
from pathlib import Path


def build_parser() -> argparse.ArgumentParser:
    repo_root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(
        description=(
            "Run RunMultiAircraft tuning and update convergence plots while it executes."
        ),
    )
    parser.add_argument(
        "--binary",
        type=Path,
        default=repo_root / "output/UNIX/bin/RunMultiAircraft",
        help="Path to RunMultiAircraft binary.",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=repo_root / "options_tune.json",
        help="Path to options JSON.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=repo_root / "tmp/tuning_convergence_plots",
        help="Directory for logs and plots.",
    )
    parser.add_argument(
        "--log-file",
        type=Path,
        default=repo_root / "tmp/tuning_convergence_plots/live_tuning.log",
        help="Log file to write streamed process output.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Show interactive plot window while running.",
    )
    parser.add_argument(
        "--snapshot-every",
        type=int,
        default=1,
        help="Save snapshot every N progress updates.",
    )
    return parser


def setup_matplotlib(show: bool):
    import matplotlib

    if not show:
        matplotlib.use("Agg")

    import matplotlib.pyplot as plt

    return plt


def make_plot_state(plt):
    fig, axes = plt.subplots(2, 1, figsize=(11, 9), sharex=True)

    ax0 = axes[0]
    (line_meas,) = ax0.plot([], [], label="Measurement samples", linewidth=2)
    (line_proc,) = ax0.plot([], [], label="Process samples", linewidth=2)
    (line_init,) = ax0.plot([], [], label="Initial-state samples", linewidth=2)
    ax0.set_ylabel("Cumulative samples")
    ax0.set_title("Covariance Tuning Progress")
    ax0.grid(True, alpha=0.3)
    ax0.legend()

    ax1 = axes[1]
    (line_dmeas,) = ax1.plot([], [], label="delta measurement", marker="o")
    (line_dproc,) = ax1.plot([], [], label="delta process", marker="o")
    (line_dinit,) = ax1.plot([], [], label="delta initial", marker="o")
    ax1.set_ylabel("Per-update increment")
    ax1.set_xlabel("Tuning elapsed time (minutes)")
    ax1.grid(True, alpha=0.3)
    ax1.legend()

    fig.tight_layout()

    return {
        "fig": fig,
        "ax0": ax0,
        "ax1": ax1,
        "line_meas": line_meas,
        "line_proc": line_proc,
        "line_init": line_init,
        "line_dmeas": line_dmeas,
        "line_dproc": line_dproc,
        "line_dinit": line_dinit,
    }


def update_plot(plt, state, x_min, meas, proc, init, dmeas, dproc, dinit):
    state["line_meas"].set_data(x_min, meas)
    state["line_proc"].set_data(x_min, proc)
    state["line_init"].set_data(x_min, init)

    state["line_dmeas"].set_data(x_min, dmeas)
    state["line_dproc"].set_data(x_min, dproc)
    state["line_dinit"].set_data(x_min, dinit)

    if x_min:
        xmin = min(x_min)
        xmax = max(x_min)
        if xmax <= xmin:
            xmax = xmin + 1e-3
        state["ax0"].set_xlim(xmin, xmax)
        state["ax1"].set_xlim(xmin, xmax)

    state["ax0"].relim()
    state["ax0"].autoscale_view(scalex=False, scaley=True)
    state["ax1"].relim()
    state["ax1"].autoscale_view(scalex=False, scaley=True)

    plt.pause(0.001)


def main() -> int:
    args = build_parser().parse_args()
    repo_root = Path(__file__).resolve().parent.parent
    binary = args.binary.resolve()
    config = args.config.resolve()
    output_dir = args.output_dir.resolve()
    log_file = args.log_file.resolve()

    output_dir.mkdir(parents=True, exist_ok=True)
    log_file.parent.mkdir(parents=True, exist_ok=True)

    plt = setup_matplotlib(args.show)
    if args.show:
        plt.ion()

    state = make_plot_state(plt)

    progress_png = output_dir / "convergence_live_progress.png"
    final_png = output_dir / "convergence_live_progress_final.png"

    flight_progress_re = re.compile(
        r"\[cov-tune\] flight-progress(?: pass=\d+)?\s+\d+/\d+.*\+meas=(\d+)\s+\+proc=(\d+)\s+\+init=(\d+).*elapsed_ms=(\d+)"
    )
    done_re = re.compile(
        r"\[cov-tune\] done elapsed_ms=(\d+) measurement_samples=(\d+) process_samples=(\d+) initial_state_samples=(\d+)"
    )

    x_min = []
    meas = []
    proc = []
    init = []
    dmeas = []
    dproc = []
    dinit = []

    cum_meas = 0
    cum_proc = 0
    cum_init = 0

    command = [str(binary), str(config)]
    if shutil.which("stdbuf") is not None:
        command = ["stdbuf", "-oL"] + command

    print(f"Running: {' '.join(command)}")
    print(f"Logging to: {log_file}")

    updates = 0
    with log_file.open("w", encoding="utf-8") as log_handle:
        process = subprocess.Popen(
            command,
            cwd=repo_root,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )

        done_line = None
        assert process.stdout is not None
        for line in process.stdout:
            log_handle.write(line)
            log_handle.flush()

            m = flight_progress_re.search(line)
            if m:
                dm = int(m.group(1))
                dp = int(m.group(2))
                di = int(m.group(3))
                elapsed_ms = int(m.group(4))

                cum_meas += dm
                cum_proc += dp
                cum_init += di

                x_min.append(elapsed_ms / 60000.0)
                meas.append(cum_meas)
                proc.append(cum_proc)
                init.append(cum_init)
                dmeas.append(dm)
                dproc.append(dp)
                dinit.append(di)

                updates += 1
                update_plot(
                    plt, state, x_min, meas, proc, init, dmeas, dproc, dinit
                )

                if updates % max(1, args.snapshot_every) == 0:
                    state["fig"].savefig(progress_png, dpi=140)

            m_done = done_re.search(line)
            if m_done:
                done_line = line.strip()

        return_code = process.wait()

    state["fig"].savefig(final_png, dpi=160)

    if args.show:
        plt.ioff()
        plt.show()

    print(f"Exit code: {return_code}")
    if done_line is not None:
        print(done_line)
    print(f"Live progress plot: {progress_png}")
    print(f"Final progress plot: {final_png}")

    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
