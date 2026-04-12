#!/usr/bin/env python3

import argparse
import json
import shutil
import subprocess
from pathlib import Path

import matplotlib

matplotlib.use("Agg")


def build_parser() -> argparse.ArgumentParser:
    repo_root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(
        description=(
            "Compare RunMultiAircraft vignette output for different "
            "reconstruction_pre_buffer values."
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
        default=repo_root / "optionsv.json",
        help="Base JSON config with vignette settings.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=repo_root / "tmp/reconstruction_pre_buffer_plots",
        help="Directory for generated configs, JSON outputs, and plots.",
    )
    parser.add_argument(
        "--pre-buffer",
        dest="pre_buffers",
        action="append",
        type=float,
        help=(
            "Pre-buffer value in seconds. Repeat for multiple values. "
            "Defaults to 0, 3, 5."
        ),
    )
    parser.add_argument(
        "--max-time",
        type=float,
        default=20.0,
        help="Only plot samples with t <= this value (seconds).",
    )
    return parser


def load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as file_handle:
        return json.load(file_handle)


def save_json(path: Path, payload: dict) -> None:
    with path.open("w", encoding="utf-8") as file_handle:
        json.dump(payload, file_handle, indent=2)


def normalize_config(base_config: dict, repo_root: Path) -> dict:
    config = json.loads(json.dumps(base_config))
    config["igc_files"] = [
        str(repo_root / Path(path)) for path in config["igc_files"]
    ]
    if "root" in config:
        config["root"] = str(Path(config["root"]).resolve())
    return config


def vignette_filename(config: dict) -> str:
    vignette = config["vignette"]
    return (
        f"vignette-{vignette['subject']}-"
        f"{vignette['start_time']}-{vignette['end_time']}.json"
    )


def run_case(
    binary: Path,
    repo_root: Path,
    output_dir: Path,
    base_config: dict,
    pre_buffer: float,
) -> Path:
    case_config = json.loads(json.dumps(base_config))
    case_config["reconstruction_pre_buffer"] = pre_buffer

    config_path = output_dir / f"options-pre-buffer-{pre_buffer:g}.json"
    save_json(config_path, case_config)

    expected_name = vignette_filename(case_config)
    working_output = repo_root / expected_name
    archived_output = (
        output_dir
        / f"{Path(expected_name).stem}-pre-buffer-{pre_buffer:g}.json"
    )
    if working_output.exists():
        working_output.unlink()
    if archived_output.exists():
        archived_output.unlink()

    subprocess.run(
        [str(binary), str(config_path)],
        cwd=repo_root,
        check=True,
    )

    if not working_output.exists():
        raise FileNotFoundError(
            f"Expected vignette output was not created: {working_output}"
        )

    shutil.move(working_output, archived_output)
    return archived_output


def get_subject_trace(vignette_json: dict) -> list[dict]:
    subject = vignette_json["subject"]
    for aircraft in vignette_json["aircraft"]:
        if aircraft["id"] == subject:
            return aircraft["trace"]
    raise KeyError(f"Subject {subject!r} not found in vignette output")


def filter_trace_by_time(trace: list[dict], max_time: float) -> list[dict]:
    return [step for step in trace if float(step["t"]) <= max_time]


def plot_track(
    case_traces: dict[float, list[dict]],
    output_dir: Path,
    max_time: float,
) -> Path:
    import matplotlib.pyplot as plt

    figure, axis = plt.subplots(figsize=(10, 8))
    for pre_buffer, trace in case_traces.items():
        clipped_trace = filter_trace_by_time(trace, max_time)
        if not clipped_trace:
            continue
        xs = [step["x"] for step in clipped_trace]
        ys = [step["y"] for step in clipped_trace]
        axis.plot(xs, ys, label=f"pre_buffer={pre_buffer:g}")
    axis.set_title(
        f"Subject XY Track by reconstruction_pre_buffer (t <= {max_time:g}s)"
    )
    axis.set_xlabel("x")
    axis.set_ylabel("y")
    axis.legend()
    axis.grid(True, alpha=0.3)
    output_path = output_dir / "reconstruction_pre_buffer-track.png"
    figure.tight_layout()
    figure.savefig(output_path, dpi=160)
    plt.close(figure)
    return output_path


def plot_timeseries(
    case_traces: dict[float, list[dict]],
    output_dir: Path,
    max_time: float,
) -> Path:
    import matplotlib.pyplot as plt

    fields = [
        ("alt_gps", "GPS Altitude"),
        ("v_tas", "True Airspeed"),
        ("bank", "Bank Angle"),
        ("yaw", "Yaw Angle"),
    ]
    figure, axes = plt.subplots(len(fields), 1, figsize=(12, 12), sharex=True)
    for axis, (field, title) in zip(axes, fields):
        for pre_buffer, trace in case_traces.items():
            clipped_trace = filter_trace_by_time(trace, max_time)
            if not clipped_trace:
                continue
            times = [step["t"] for step in clipped_trace]
            values = [step[field] for step in clipped_trace]
            axis.plot(times, values, label=f"pre_buffer={pre_buffer:g}")
        axis.set_title(title)
        axis.grid(True, alpha=0.3)
        axis.legend()
    axes[-1].set_xlabel(f"t [s], clipped to <= {max_time:g}")
    output_path = output_dir / "reconstruction_pre_buffer-timeseries.png"
    figure.tight_layout()
    figure.savefig(output_path, dpi=160)
    plt.close(figure)
    return output_path


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parent.parent
    binary = args.binary.resolve()
    config_path = args.config.resolve()
    output_dir = args.output_dir.resolve()
    max_time = args.max_time
    pre_buffers = args.pre_buffers or [0.0, 3.0, 5.0]

    output_dir.mkdir(parents=True, exist_ok=True)
    base_config = normalize_config(load_json(config_path), repo_root)
    if "vignette" not in base_config:
        raise KeyError("Config must contain a vignette section")

    case_outputs: dict[float, Path] = {}
    case_traces: dict[float, list[dict]] = {}
    for pre_buffer in pre_buffers:
        output_json = run_case(
            binary, repo_root, output_dir, base_config, pre_buffer
        )
        vignette_json = load_json(output_json)
        case_outputs[pre_buffer] = output_json
        case_traces[pre_buffer] = get_subject_trace(vignette_json)

    track_plot = plot_track(case_traces, output_dir, max_time)
    timeseries_plot = plot_timeseries(case_traces, output_dir, max_time)

    print("Generated comparison cases:")
    for pre_buffer, output_json in sorted(case_outputs.items()):
        print(f"  pre_buffer={pre_buffer:g}: {output_json}")
    print(f"Generated plots:\n  {track_plot}\n  {timeseries_plot}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
