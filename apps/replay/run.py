#!/usr/bin/env python3
"""Single entry point for paired recording, C++ playback and input checks."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys

import yaml
import record

ROOT = record.ROOT
NATIVE = Path(__file__).resolve().parent


def validate_noise(noise):
    if not isinstance(noise, dict) or set(noise) - {"v1", "v2", "q_h", "r_scale"}:
        raise ValueError("ekf supports only v1, v2, q_h and r_scale")
    for key in ("v1", "v2", "q_h"):
        if key in noise:
            v = noise[key]
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v < 0:
                raise ValueError(f"ekf.{key} must be finite and nonnegative")
    if "r_scale" in noise:
        values = noise["r_scale"]
        if not isinstance(values, list) or len(values) != 4 or any(
            isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0
            for v in values
        ):
            raise ValueError("ekf.r_scale needs four finite positive numbers")


def load_settings(path):
    path = path.resolve()
    settings = record.load_yaml(path).get("replay")
    if not isinstance(settings, dict):
        raise ValueError("Missing replay mapping")
    settings = dict(settings)
    for key in ("video", "frames_csv", "auto_aim_yaml", "camera_yaml", "output_dir"):
        if not isinstance(settings.get(key), str) or not settings[key].strip():
            raise ValueError(f"Missing replay.{key}")
        settings[key] = (path.parent / settings[key]).resolve()
        if key != "output_dir" and not settings[key].is_file():
            raise ValueError(f"Missing input: {settings[key]}")
    speed = settings.get("speed", 1.0)
    if isinstance(speed, bool) or not isinstance(speed, (int, float)) or not math.isfinite(speed) or speed < 0:
        raise ValueError("speed must be finite and nonnegative")
    settings["speed"] = float(speed)
    if not isinstance(settings.get("display", True), bool):
        raise ValueError("display must be true or false")
    settings.setdefault("display", True)
    validate_noise(settings.get("ekf", {}))
    return settings


def build_inputs():
    paths = {ROOT / "auto_aim/armor_types.hpp", ROOT / "hardware/common_def.hpp"}
    for directory in ("apps/replay", "auto_aim/armor_detector", "auto_aim/armor_track",
                      "rm_utils/algorithm", "rm_utils/display", "rm_utils/mas_log",
                      "rm_utils/plotter", "rm_utils/thread_pool", "rm_utils/SPSCQueue"):
        paths.update(p for p in (ROOT / directory).rglob("*") if p.is_file() and
                     (p.suffix in (".cpp", ".hpp", ".h", ".py", ".cmake") or p.name == "CMakeLists.txt"))
    return {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(paths)}


def build_backend(workspace):
    workspace.mkdir(parents=True, exist_ok=True)
    build = workspace / "build"
    executable = build / ("replay.exe" if sys.platform == "win32" else "replay")
    stamp = workspace / "build_inputs.json"
    inputs = json.dumps(build_inputs(), sort_keys=True)
    if executable.is_file() and (build / "CMakeCache.txt").is_file() and stamp.is_file() and stamp.read_text(encoding="utf-8") == inputs:
        print("Replay build unchanged; reusing cached application.", flush=True)
        return executable
    cmake = shutil.which("cmake")
    if not cmake:
        raise ValueError("cmake not found; build the replay app in the Linux vision environment")
    print("Building apps/replay (incremental when available).", flush=True)
    subprocess.run([cmake, "-S", str(NATIVE), "-B", str(build), "-DCMAKE_BUILD_TYPE=Release"], check=True)
    subprocess.run([cmake, "--build", str(build), "--target", "replay", "-j", "2"], check=True)
    if not executable.is_file():
        raise ValueError("Build completed without the replay executable")
    stamp.write_text(inputs, encoding="utf-8")
    return executable


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    recording = commands.add_parser("record", help="Record paired frames using an isolated copy of the live app")
    recording.add_argument("--config", type=Path, default=ROOT / "config/record.yaml")
    recording.add_argument("--prepare", action="store_true", help="Prepare the copy only; no build or hardware access")
    playback = commands.add_parser("replay", help="Build/reuse and launch the C++ replay app")
    playback.add_argument("--config", type=Path, default=ROOT / "config/replay.yaml")
    playback.add_argument("--headless", action="store_true")
    check = commands.add_parser("check", help="Validate the whole video and paired CSV; no C++ build required")
    check.add_argument("--config", type=Path, default=ROOT / "config/replay.yaml")
    args = parser.parse_args(argv)
    try:
        if args.command == "record":
            return record.run(args.config, prepare_only=args.prepare)
        settings = load_settings(args.config)
        if args.command == "check":
            count = record.check_video(settings["video"], record.load_frames(settings["frames_csv"]))
            print(f"Pairing check passed: {count} frames")
            return 0
        project_id = hashlib.sha256(str(ROOT.resolve()).encode("utf-8")).hexdigest()[:16]
        workspace = Path.home() / ".cache/mas_vision_tools" / ("replay_app_" + project_id)
        workspace.mkdir(parents=True, exist_ok=True)
        with (workspace / "build.lock").open("a") as lock:
            if sys.platform == "linux":
                import fcntl
                fcntl.flock(lock, fcntl.LOCK_EX)
            executable = build_backend(workspace)
        command = [str(executable), "--config", str(args.config.resolve())]
        if args.headless:
            command.append("--headless")
        return subprocess.run(command, cwd=ROOT).returncode
    except (ValueError, OSError, KeyError, TypeError, yaml.YAMLError, subprocess.SubprocessError) as error:
        print(f"Replay error: {error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())


