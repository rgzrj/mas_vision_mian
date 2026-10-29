#!/usr/bin/env python3
"""Record paired camera/IMU data using an isolated copy of the existing application."""
import csv
import math
import hashlib
import json
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time

import yaml

ROOT = Path(__file__).resolve().parents[2]
FIELDS = ("frame_index", "timestamp_ns", "t_recv_ns", "q_w", "q_x", "q_y", "q_z", "mode")


def load_yaml(path):
    with path.open(encoding="utf-8-sig") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"Expected YAML mapping: {path}")
    return data


def load_frames(path):
    frames = []
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames or set(FIELDS) - set(reader.fieldnames):
            raise ValueError("frames.csv needs columns: " + ",".join(FIELDS))
        for line, row in enumerate(reader, 2):
            try:
                index = int(row["frame_index"])
                ns, recv = int(row["timestamp_ns"]), int(row["t_recv_ns"])
                mode = int(row["mode"])
                q = [float(row[k]) for k in ("q_w", "q_x", "q_y", "q_z")]
                if index != len(frames):
                    raise ValueError("frame_index must be contiguous from zero, in video decode order")
                if not 0 <= ns <= recv <= 2**63 - 1 or (frames and ns <= frames[-1]["timestamp_ns"]):
                    raise ValueError("invalid timestamp order or receipt time")
                norm = math.hypot(*q)
                if mode not in range(6) or not all(math.isfinite(v) for v in q) or not math.isfinite(norm) or norm <= 1e-6:
                    raise ValueError("invalid quaternion or mode")
                frames.append({"frame_index": index, "timestamp_ns": ns, "t_recv_ns": recv,
                               "q": q, "mode": mode})
            except (ValueError, TypeError) as error:
                raise ValueError(f"{path}:{line}: {error}") from error
    if not frames:
        raise ValueError("frames.csv is empty")
    return frames


def check_video(path, frames):
    import cv2
    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise ValueError(f"Cannot open video: {path}")
    count = 0
    try:
        while capture.read()[0]:
            count += 1
    finally:
        capture.release()
    if count != len(frames):
        raise ValueError(f"Video has {count} frames but CSV has {len(frames)} rows")
    return count


SOURCE_DIRS = ("apps", "auto_aim", "hardware", "rm_utils", "threads", "config")


def write_changed(path, content):
    if not path.is_file() or path.read_bytes() != content:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(content)


def instrument_thread(source):
    include = '#include "user_serial.hpp"'
    start = '    MAS_LOG_INFO("Armor processing thread started");'
    if source.count(include) != 1 or source.count(start) != 1:
        raise ValueError("Unknown armor_thread layout; refusing to guess the recording hook")
    source = source.replace(include, include + '\n#include "recorder.hpp"', 1)
    source = source.replace(start, start + '\n    replay_app::PairedRecorder recorder("config/recording.yaml");', 1)
    pattern = r'(?m)^([ \t]*)Eigen::Quaterniond\s+pos\s*=\s*serial\.q\(frame\.timestamp\);'
    source, count = re.subn(pattern, lambda m: m[0] + '\n' + m[1] +
                            'recorder.push(frame, pos, static_cast<int>(current_mode));', source)
    if count != 1:
        raise ValueError("Expected one paired image/quaternion call; no original file was changed")
    return source


def load_settings(path):
    path = path.resolve()
    config = load_yaml(path).get("recording")
    if not isinstance(config, dict):
        raise ValueError("Missing recording mapping")
    config = dict(config)
    for key in ("auto_aim_yaml", "camera_yaml", "serial_yaml", "output_dir"):
        if not isinstance(config.get(key), str) or not config[key]:
            raise ValueError(f"Missing recording.{key}")
        config[key] = (path.parent / config[key]).resolve()
        if key != "output_dir" and not config[key].is_file():
            raise ValueError(f"Missing input: {config[key]}")
    fps, size, codec = config.get("fps", 60.0), config.get("queue_size", 32), config.get("codec", "MJPG")
    max_record_fps = config.get("max_record_fps", 60.0)
    if isinstance(fps, bool) or not isinstance(fps, (int, float)) or not math.isfinite(fps) or fps <= 0:
        raise ValueError("fps must be finite and positive")
    if type(size) is not int or size <= 0 or not isinstance(codec, str) or len(codec) != 4:
        raise ValueError("queue_size must be positive; codec needs exactly four characters")
    if (isinstance(max_record_fps, bool) or not isinstance(max_record_fps, (int, float)) or
            not math.isfinite(max_record_fps) or not 0 <= max_record_fps <= 1.0e9 or
            0 < max_record_fps < 1.0e-9):
        raise ValueError("max_record_fps must be finite and within [0, 1e9]; zero records every frame")
    config.update(fps=fps, queue_size=size, codec=codec, max_record_fps=max_record_fps)
    return config


def prepare(settings, output, workspace):
    source = workspace / "source"
    source.mkdir(parents=True, exist_ok=True)
    paths = [ROOT / "CMakeLists.txt"]
    for directory in SOURCE_DIRS:
        paths.extend(p for p in (ROOT / directory).rglob("*")
                     if p.is_file() and "__pycache__" not in p.parts and p.suffix != ".pyc")
    manifest = {}
    for file in sorted(paths):
        relative = file.relative_to(ROOT)
        content = file.read_bytes()
        manifest[str(relative)] = hashlib.sha256(content).hexdigest()
        if relative.as_posix() == "threads/armor_thread/armor_thread.cpp":
            content = instrument_thread(content.decode("utf-8")).encode("utf-8")
        write_changed(source / relative, content)
    for file in paths:
        if hashlib.sha256(file.read_bytes()).hexdigest() != manifest[str(file.relative_to(ROOT))]:
            raise ValueError("Source changed during copy; retry after editing is complete")
    header = Path("threads/armor_thread/recorder.hpp")
    write_changed(source / header, Path(__file__).with_name("recorder.hpp").read_bytes())
    # Remove stale copied inputs only; retain source/build and its object files.
    managed = set(manifest) | {str(header), str(Path("config/recording.yaml"))}
    for directory in SOURCE_DIRS:
        for file in (source / directory).rglob("*"):
            if file.is_file() and str(file.relative_to(source)) not in managed:
                file.unlink()
    for key, name in (("auto_aim_yaml", "auto_aim.yaml"), ("camera_yaml", "hikcamera.yaml"),
                      ("serial_yaml", "serial_config.yaml")):
        shutil.copy2(settings[key], source / "config" / name)
        shutil.copy2(settings[key], output / name)
    # Preserve the classifier alongside the recording so the session can be moved.
    auto_yaml = source / "config/auto_aim.yaml"
    auto = load_yaml(auto_yaml)
    numbers = auto["auto_aim"]["armor_detector"]["number_params"]
    model_dir = output / "model"
    model_dir.mkdir()
    for key, name in (("model_path", "model.onnx"), ("label_path", "label.txt")):
        original_path = (settings["auto_aim_yaml"].parent / numbers[key]).resolve()
        if not original_path.is_file():
            raise ValueError(f"Missing classifier input: {original_path}")
        shutil.copy2(original_path, model_dir / name)
        numbers[key] = "model/" + name
    (output / "auto_aim.yaml").write_text(
        yaml.safe_dump(auto, allow_unicode=True, sort_keys=False), encoding="utf-8")
    for key in ("model_path", "label_path"):
        numbers[key] = str((output / numbers[key]).resolve())
    auto_yaml.write_text(yaml.safe_dump(auto, allow_unicode=True, sort_keys=False), encoding="utf-8")
    (source / "config/recording.yaml").write_text(yaml.safe_dump(
        {"output_dir": str(output), "fps": settings["fps"], "codec": settings["codec"],
         "queue_size": settings["queue_size"], "max_record_fps": settings["max_record_fps"]}), encoding="utf-8")
    (output / "source_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    (output / "native_workspace.txt").write_text(str(workspace), encoding="utf-8")
    replay_config = {"replay": {"video": "video.avi", "frames_csv": "frames.csv",
                               "auto_aim_yaml": "auto_aim.yaml", "camera_yaml": "hikcamera.yaml",
                               "output_dir": "../../replay" if output.parent.resolve() == (ROOT / "data/recordings").resolve() else "replay_output",
                               "speed": 1.0, "display": True}}
    (output / "replay.yaml").write_text(yaml.safe_dump(replay_config, sort_keys=False), encoding="utf-8")
    return source


def build_recording(source, workspace):
    build = source / "build"
    executable = build / "base"
    inputs = {}
    for directory in SOURCE_DIRS:
        if directory == "config":
            continue
        for file in (source / directory).rglob("*"):
            if file.is_file() and (file.suffix.lower() in
                    (".cpp", ".cc", ".c", ".cxx", ".h", ".hpp", ".hh", ".hxx", ".cmake", ".a", ".so")
                    or file.name == "CMakeLists.txt"):
                inputs[str(file.relative_to(source))] = hashlib.sha256(file.read_bytes()).hexdigest()
    inputs["CMakeLists.txt"] = hashlib.sha256((source / "CMakeLists.txt").read_bytes()).hexdigest()
    manifest = json.dumps(inputs, sort_keys=True)
    stamp = workspace / "build_inputs.json"
    configured = (build / "CMakeCache.txt").is_file()
    if configured and executable.is_file() and stamp.is_file() and stamp.read_text(encoding="utf-8") == manifest:
        print("Recording build unchanged; reusing cached executable.", flush=True)
        return executable
    cmake = shutil.which("cmake")
    if not cmake:
        raise ValueError("cmake not found; recording requires the Linux vision environment and MVS SDK")
    # Refresh recursive CMake discovery too, including added/deleted subdirectories.
    subprocess.run([cmake, "-S", str(source), "-B", str(build), "-DCMAKE_BUILD_TYPE=Release"], check=True)
    print("Building recording application (incremental when available).", flush=True)
    subprocess.run([cmake, "--build", str(build), "--target", "base", "-j", "2"], check=True)
    # A failed build must never mark the old executable as current.
    stamp.write_text(manifest, encoding="utf-8")
    return executable


def run(config_path, prepare_only=False):
    process = None
    cache_lock = None
    try:
        settings = load_settings(config_path)
        project_id = hashlib.sha256(str(ROOT.resolve()).encode("utf-8")).hexdigest()[:16]
        workspace = Path.home() / ".cache/mas_vision_tools" / ("record_" + project_id)
        workspace.mkdir(parents=True, exist_ok=True)
        if sys.platform == "linux":
            import fcntl
            cache_lock = (workspace / "record.lock").open("a")
            try:
                fcntl.flock(cache_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                raise ValueError("This project's recording cache is in use; stop the other recording first")
        output = settings["output_dir"] / (time.strftime("session_%Y%m%d_%H%M%S_") + str(time.time_ns() % 1000000000))
        output.mkdir(parents=True, exist_ok=False)
        shutil.copy2(config_path, output / "record.yaml")
        source = prepare(settings, output, workspace)
        print("Recording output:", output, flush=True)
        print(f"Recording settings: codec={settings['codec']} queue_size={settings['queue_size']} "
              f"fps_metadata={settings['fps']} max_record_fps={settings['max_record_fps']}", flush=True)
        if settings["max_record_fps"] > 0:
            print("Recording is sampled; online tracking still processes its original input stream.", flush=True)
        if prepare_only:
            print("Prepared source only:", source)
            return 0
        build = source / "build"
        executable = build_recording(source, workspace)
        print("Stop any existing base process before recording. Ctrl+C stops and finalizes this recording.", flush=True)
        process = subprocess.Popen([str(executable)], cwd=build)
        try:
            code = process.wait()
        except KeyboardInterrupt:
            # Python and the application normally share a POSIX foreground process group.
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            code = process.wait(timeout=30)
        if code not in (0, -signal.SIGINT):
            raise ValueError(f"Recording application exited with code {code}")
        summary_path = output / "recording_summary.json"
        if not summary_path.is_file():
            raise ValueError("No recording summary: application did not finish recorder shutdown")
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        print("Recording summary:\n" + json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
        if summary.get("failed"):
            raise ValueError("Recording is incomplete: " + summary.get("error", "unknown error"))
        frames = load_frames(output / "frames.csv")
        count = check_video(output / "video.avi", frames)
        print(f"Paired recording verified: {count} frames\nReplay with:\n"
              f'python3 apps/replay/run.py replay --config "{output / "replay.yaml"}"')
        return 0
    except (ValueError, OSError, KeyError, TypeError, yaml.YAMLError, subprocess.SubprocessError) as error:
        print(f"Recording error: {error}", file=sys.stderr)
        return 1
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        if cache_lock is not None:
            cache_lock.close()
