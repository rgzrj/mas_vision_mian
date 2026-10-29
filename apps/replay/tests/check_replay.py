"""Check replay app preparation, metadata, build cache and recording compatibility."""
import csv
import hashlib
import json
import os
import re
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import run as replay
import record


class ReplayChecks(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.frames = [{"frame_index": i, "timestamp_ns": ns, "t_recv_ns": ns + 12960000,
                        "q": [0.9983, 0, 0, 0], "mode": i % 2}
                       for i, ns in enumerate((1000000000, 1006000000, 1030000000))]
        self.csv = self.root / "frames.csv"
        self.write_csv(self.frames)
        self.video = self.root / "video.avi"
        writer = cv2.VideoWriter(str(self.video), cv2.VideoWriter_fourcc(*"MJPG"), 30, (64, 48))
        self.assertTrue(writer.isOpened())
        for i in range(3):
            writer.write(np.full((48, 64, 3), i * 60, dtype=np.uint8))
        writer.release()

    def tearDown(self):
        self.temp.cleanup()

    def write_csv(self, frames):
        with self.csv.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=record.FIELDS)
            writer.writeheader()
            for frame in frames:
                row = {k: frame[k] for k in ("frame_index", "timestamp_ns", "t_recv_ns", "mode")}
                row.update(zip(("q_w", "q_x", "q_y", "q_z"), frame["q"]))
                writer.writerow(row)

    def test_pairing_checks_order_and_quaternion(self):
        self.assertEqual(record.load_frames(self.csv), self.frames)
        self.assertEqual(record.check_video(self.video, self.frames), 3)
        with self.assertRaises(ValueError):
            record.check_video(self.video, self.frames[:-1])
        for field, value in [("timestamp_ns", self.frames[0]["timestamp_ns"]),
                             ("frame_index", 8), ("q", [0, 0, 0, 0])]:
            frames = [dict(f) for f in self.frames]
            frames[1][field] = value
            self.write_csv(frames)
            with self.assertRaises(ValueError):
                record.load_frames(self.csv)

    def test_replay_target_generation_never_changes_originals(self):
        source = replay.ROOT / "auto_aim/armor_track/armor_target.cpp"
        before = source.read_bytes()
        destination = self.root / "replay_target.cpp"
        subprocess.run([sys.executable, str(replay.NATIVE / "prepare_target.py"), str(source), str(destination)], check=True)
        self.assertEqual(source.read_bytes(), before)
        copied = destination.read_text(encoding="utf-8")
        for key in ("v1", "v2", "q_h"):
            self.assertIn(f"replay_noise(\"{key}\",", copied)
        self.assertIn("R_diag[replay_i] *= replay_r_scale(replay_i);", copied)
        self.assertEqual(copied.count("const Eigen::VectorXd replay_prior = ekf_.x;"), 1)
        self.assertEqual(copied.count("replay_observation_update(armor, *this,"), 2)
        self.assertIn("    constrainState();\n    replay_observation_update", copied)
        self.assertTrue(destination.with_name("replay_instrumentation.patch").is_file())
        bad = self.root / "unknown.cpp"
        bad.write_text("unknown source", encoding="utf-8")
        failed = subprocess.run([sys.executable, str(replay.NATIVE / "prepare_target.py"), str(bad), str(destination)], capture_output=True)
        self.assertNotEqual(failed.returncode, 0)
        self.assertEqual(destination.read_text(encoding="utf-8"), copied)

    def test_recording_preparation_preserves_originals_and_models(self):
        thread = replay.ROOT / "threads/armor_thread/armor_thread.cpp"
        before = thread.read_bytes()
        modified = record.instrument_thread(before.decode("utf-8"))
        self.assertEqual(modified.count("recorder.push(frame, pos,"), 1)
        self.assertGreater(modified.index("recorder.push(frame, pos,"), modified.index("serial.q(frame.timestamp)"))
        with self.assertRaises(ValueError):
            record.instrument_thread("unknown layout")
        settings = record.load_settings(replay.ROOT / "config/record.yaml")
        self.assertEqual(settings["codec"], "MJPG")
        self.assertEqual(settings["fps"], 60)
        self.assertEqual(settings["output_dir"], replay.ROOT / "data/recordings")
        output = self.root / "recording"
        output.mkdir()
        source = record.prepare(settings, output, self.root / "workspace")
        self.assertEqual(thread.read_bytes(), before)
        self.assertIn("recorder.push(frame, pos,", (source / "threads/armor_thread/armor_thread.cpp").read_text(encoding="utf-8"))
        config = record.load_yaml(output / "auto_aim.yaml")
        numbers = config["auto_aim"]["armor_detector"]["number_params"]
        for key in ("model_path", "label_path"):
            self.assertTrue((output / numbers[key]).is_file())
        copied = record.load_yaml(source / "config/auto_aim.yaml")
        for key in ("model_path", "label_path"):
            self.assertTrue(Path(copied["auto_aim"]["armor_detector"]["number_params"][key]).is_absolute())
        self.assertTrue((output / "replay.yaml").is_file())
        cpp = source / "threads/armor_thread/armor_thread.cpp"
        header = cpp.with_name("recorder.hpp")
        timestamps = [p.stat().st_mtime_ns for p in (cpp, header)]
        stale = source / "auto_aim/deleted_source.cpp"
        stale.write_text("old copy", encoding="utf-8")
        next_output = self.root / "next_recording"
        next_output.mkdir()
        record.prepare(dict(settings, codec="FFV1", queue_size=8), next_output, self.root / "workspace")
        self.assertEqual([p.stat().st_mtime_ns for p in (cpp, header)], timestamps)
        self.assertFalse(stale.exists())
        self.assertEqual(record.load_yaml(source / "config/recording.yaml")["output_dir"], str(next_output))
        self.assertEqual(record.load_yaml(source / "config/recording.yaml")["max_record_fps"], 60)

    @unittest.skipUnless(shutil.which("g++"), "C++ compiler needed for the production sampler check")
    def test_production_recording_sampler_limits_rate_without_catchup_bursts(self):
        header = (replay.NATIVE / "recorder.hpp").read_text(encoding="utf-8")
        function = header.split("inline bool recording_frame_due", 1)[1].split("\nclass PairedRecorder", 1)[0]
        source = self.root / "sampling.cpp"
        source.write_text('#include <cstdint>\n#include <cassert>\ninline bool recording_frame_due' + function + r'''
int main() {
    int64_t next = -1;
    int kept = 0;
    for (int64_t i = 0; i < 1680; ++i)
        kept += recording_frame_due(1000000000 + i * 1000000000 / 168, 16666667, next);
    assert(kept == 600);
    assert(recording_frame_due(100000000000, 16666667, next));
    assert(!recording_frame_due(100000000001, 16666667, next));
    next = -1;
    for (int64_t i = 0; i < 100; ++i)
        assert(recording_frame_due(i, 0, next));
    next = -1;
    for (int64_t i = 0; i < 100; ++i)
        assert(recording_frame_due(i * 33333334, 16666667, next));
}
''', encoding="utf-8")
        executable = self.root / "sampling.exe"
        env = os.environ.copy()
        env["PATH"] = str(Path(shutil.which("g++")).parent) + os.pathsep + env.get("PATH", "")
        subprocess.run([shutil.which("g++"), "-std=c++17", str(source), "-o", str(executable)],
                       check=True, capture_output=True, text=True, env=env)
        subprocess.run([str(executable)], check=True, capture_output=True, text=True, env=env)

    def test_recording_build_cache_skips_runtime_changes_and_retries_failure(self):
        workspace = self.root / "cached"
        source = workspace / "source"
        cpp = source / "apps/main.cpp"
        record.write_changed(cpp, b"int main() { return 0; }")
        record.write_changed(source / "CMakeLists.txt", b"project(record_test)")
        build = source / "build"
        def run(command, **kwargs):
            build.mkdir(exist_ok=True)
            (build / "CMakeCache.txt").write_text("configured", encoding="utf-8")
            if "--build" in command:
                (build / "base").write_bytes(b"test executable")
        with mock.patch.object(record.shutil, "which", return_value="cmake"), \
                mock.patch.object(record.subprocess, "run", side_effect=run) as commands:
            executable = record.build_recording(source, workspace)
            self.assertTrue(executable.is_file())
            self.assertEqual(commands.call_count, 2)
            commands.reset_mock()
            record.write_changed(source / "config/recording.yaml", b"codec: FFV1\nqueue_size: 8\n")
            record.build_recording(source, workspace)
            commands.assert_not_called()
            record.write_changed(cpp, b"int main() { return 1; }")
            record.build_recording(source, workspace)
            self.assertEqual(commands.call_count, 2)
            self.assertIn("--build", commands.call_args.args[0])
            commands.reset_mock()
            previous_stamp = (workspace / "build_inputs.json").read_bytes()
            record.write_changed(cpp, b"invalid source")
            commands.side_effect = subprocess.CalledProcessError(1, ["cmake", "--build"])
            with self.assertRaises(subprocess.CalledProcessError):
                record.build_recording(source, workspace)
            self.assertEqual((workspace / "build_inputs.json").read_bytes(), previous_stamp)
            commands.side_effect = run
            commands.reset_mock()
            record.build_recording(source, workspace)
            self.assertEqual(commands.call_count, 2)
            commands.reset_mock()
            record.build_recording(source, workspace)
            commands.assert_not_called()

    def test_rejects_bad_noise(self):
        for noise in ({"v1": -1}, {"v2": float("nan")}, {"q_h": True},
                      {"r_scale": [1, 0, 1, 1]}, {"p0": [1]}):
            with self.assertRaises(ValueError):
                replay.validate_noise(noise)

    def test_settings_paths_are_relative_to_yaml(self):
        cfg = self.root / "config.yaml"
        cfg.write_text("replay:\n  video: video.avi\n  frames_csv: frames.csv\n"
                       "  auto_aim_yaml: auto.yaml\n  camera_yaml: camera.yaml\n"
                       "  output_dir: output\n  speed: 0\n", encoding="utf-8")
        for name in ("auto.yaml", "camera.yaml"):
            (self.root / name).write_text("{}", encoding="utf-8")
        settings = replay.load_settings(cfg)
        self.assertEqual(settings["video"], self.video)
        self.assertEqual(settings["output_dir"], self.root / "output")

    def test_replay_build_cache_uses_runtime_configuration_without_rebuilding(self):
        workspace = self.root / "replay_cache"
        build = workspace / "build"
        executable = build / ("replay.exe" if sys.platform == "win32" else "replay")
        def run(command, **kwargs):
            build.mkdir(parents=True, exist_ok=True)
            (build / "CMakeCache.txt").write_text("configured", encoding="utf-8")
            if "--build" in command:
                executable.write_bytes(b"mock executable")
        with mock.patch.object(replay, "build_inputs", return_value={"source": "a"}) as inputs, \
                mock.patch.object(replay.shutil, "which", return_value="cmake"), \
                mock.patch.object(replay.subprocess, "run", side_effect=run) as commands:
            self.assertEqual(replay.build_backend(workspace), executable)
            self.assertEqual(commands.call_count, 2)
            commands.reset_mock()
            replay.build_backend(workspace)
            commands.assert_not_called()
            previous_stamp = (workspace / "build_inputs.json").read_bytes()
            inputs.return_value = {"source": "b"}
            commands.side_effect = subprocess.CalledProcessError(1, ["cmake"])
            with self.assertRaises(subprocess.CalledProcessError):
                replay.build_backend(workspace)
            self.assertEqual((workspace / "build_inputs.json").read_bytes(), previous_stamp)
            commands.side_effect = run
            commands.reset_mock()
            replay.build_backend(workspace)
            self.assertEqual(commands.call_count, 2)
        self.assertFalse(any(key.startswith("config") for key in replay.build_inputs()))
        self.assertIn(str(Path("apps/replay/prepare_target.py")), replay.build_inputs())

    def test_single_entry_dispatches_record_check_and_replay_from_other_directory(self):
        with mock.patch.object(record, "run", return_value=0) as recording:
            self.assertEqual(replay.main(["record", "--prepare"]), 0)
            recording.assert_called_once_with(replay.ROOT / "config/record.yaml", prepare_only=True)
        cfg = self.root / "replay.yaml"
        cfg.write_text("replay:\n  video: video.avi\n  frames_csv: frames.csv\n"
                       "  auto_aim_yaml: auto.yaml\n  camera_yaml: camera.yaml\n"
                       "  output_dir: output\n  speed: 0\n", encoding="utf-8")
        for name in ("auto.yaml", "camera.yaml"):
            (self.root / name).write_text("{}", encoding="utf-8")
        command = [sys.executable, str(replay.NATIVE / "run.py"), "check", "--config", str(cfg)]
        result = subprocess.run(command, cwd=self.root, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Pairing check passed: 3 frames", result.stdout)
        executable = self.root / "replay"
        with mock.patch.object(replay.Path, "home", return_value=self.root), \
                mock.patch.object(replay, "build_backend", return_value=executable), \
                mock.patch.object(replay.subprocess, "run", return_value=mock.Mock(returncode=7)) as native:
            self.assertEqual(replay.main(["replay", "--config", str(cfg), "--headless"]), 7)
            native.assert_called_once_with([str(executable), "--config", str(cfg), "--headless"], cwd=replay.ROOT)
        self.write_csv(self.frames[:-1])
        result = subprocess.run(command, cwd=self.root, capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Video has 3 frames but CSV has 2 rows", result.stderr)

    def test_record_prepare_through_single_entry_keeps_original_thread_untouched(self):
        settings = record.load_settings(replay.ROOT / "config/record.yaml")
        config = self.root / "record.yaml"
        config.write_text(record.yaml.safe_dump({"recording": {
            **{key: str(value) if isinstance(value, Path) else value for key, value in settings.items()},
            "output_dir": str(self.root / "recordings")}}), encoding="utf-8")
        original = replay.ROOT / "threads/armor_thread/armor_thread.cpp"
        before = original.read_bytes()
        with mock.patch.object(record.Path, "home", return_value=self.root), \
                mock.patch.object(record, "build_recording") as build:
            self.assertEqual(replay.main(["record", "--config", str(config), "--prepare"]), 0)
            build.assert_not_called()
        self.assertEqual(original.read_bytes(), before)
        sessions = list((self.root / "recordings").glob("session_*"))
        self.assertEqual(len(sessions), 1)
        self.assertTrue((sessions[0] / "replay.yaml").is_file())
        self.assertFalse((sessions[0] / "video.avi").exists())
        copied = next((self.root / ".cache").glob("**/source/threads/armor_thread/armor_thread.cpp"))
        self.assertIn("replay_app::PairedRecorder", copied.read_text(encoding="utf-8"))
        self.assertTrue(copied.with_name("recorder.hpp").is_file())

    @unittest.skipUnless(shutil.which("g++"), "C++ compiler needed for the actual metadata reader")
    def test_cpp_metadata_reader_preserves_pairing_and_rejects_bad_rows(self):
        # Compile the real stdlib-only reader; OpenCV/Eigen playback still needs Linux validation.
        cpp = (replay.NATIVE / "player.cpp").read_text(encoding="utf-8").split("Player::Player", 1)[0]
        header = (replay.NATIVE / "player.hpp").read_text(encoding="utf-8")
        metadata = "struct Metadata" + header.split("struct Metadata", 1)[1].split("std::vector<Metadata> read_metadata", 1)[0]
        cpp = cpp.replace('#include "player.hpp"', '#include <filesystem>\n#include <cstdint>\n#include <array>\n#include <vector>\n#include <iostream>')
        cpp = cpp.replace("namespace replay_app\n{", "namespace replay_app\n{\n" + metadata, 1)
        cpp += r'''
}
int main(int argc, char **argv) {
    try {
        const auto frames = replay_app::read_metadata(argv[1]);
        std::cout << frames.size() << ' ' << frames.front().timestamp_ns << ' '
                  << frames.back().timestamp_ns << ' ' << frames.back().t_recv_ns;
        return 0;
    } catch (const std::exception &error) { std::cerr << error.what(); return 1; }
}
'''
        source = self.root / "reader.cpp"
        source.write_text(cpp, encoding="utf-8")
        executable = self.root / "reader.exe"
        env = os.environ.copy()
        env["PATH"] = str(Path(shutil.which("g++")).parent) + os.pathsep + env.get("PATH", "")
        subprocess.run([shutil.which("g++"), "-std=c++17", str(source), "-o", str(executable)],
                       check=True, capture_output=True, text=True, env=env)
        def read():
            return subprocess.run([str(executable), str(self.csv)], capture_output=True, text=True, env=env)
        result = read()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, "3 1000000000 1030000000 1042960000")
        original = self.csv.read_text(encoding="utf-8")
        self.csv.write_text("\ufeff" + original.replace("frame_index", '"frame_index"'), encoding="utf-8")
        self.assertEqual(read().returncode, 0)
        for key, value in (("timestamp_ns", self.frames[0]["timestamp_ns"]), ("frame_index", 99),
                           ("mode", 6), ("q", [0, 0, 0, 0]), ("q", [float("nan"), 0, 0, 0])):
            frames = [dict(f) for f in self.frames]
            frames[1][key] = value
            self.write_csv(frames)
            result = read()
            self.assertNotEqual(result.returncode, 0, key)
            self.assertIn(":3:", result.stderr)

    @unittest.skipUnless(shutil.which("g++"), "C++ compiler needed for generated parameter checks")
    def test_generated_parameters_preserve_defaults_and_other_target_branches(self):
        copied = self.root / "replay_target.cpp"
        subprocess.run([sys.executable, str(replay.NATIVE / "prepare_target.py"),
                        str(replay.ROOT / "auto_aim/armor_track/armor_target.cpp"), str(copied)], check=True)
        text = copied.read_text(encoding="utf-8")
        declarations = "\n".join(line for line in text.splitlines() if re.match(r"\s*double (v1|v2|q_h) =", line))
        self.assertEqual(len(declarations.splitlines()), 3)
        scales = re.search(r"    if \(armor_num == 4\)\n        for \(int replay_i[^\n]+", text)[0]
        source = self.root / "parameters.cpp"
        source.write_text(r'''
#include <array>
#include <string>
#include <cassert>
bool enabled = false;
int calls = 0;
double replay_noise(const char *key, double fallback) {
    ++calls;
    if (!enabled) return fallback;
    return std::string(key) == "v1" ? 12 : std::string(key) == "v2" ? 34 : 0;
}
double replay_r_scale(int i) { ++calls; return enabled ? i + 2 : 1; }
std::array<double, 7> parameters(int armor_num, const std::string &name) {
''' + declarations + r'''
    std::array<double, 4> R_diag{1, 2, 3, 4};
''' + scales + r'''
    return {v1, v2, q_h, R_diag[0], R_diag[1], R_diag[2], R_diag[3]};
}
int main() {
    assert((parameters(4, "3") == std::array<double, 7>{100, 400, 1e-6, 1, 2, 3, 4}));
    enabled = true;
    calls = 0;
    assert((parameters(4, "3") == std::array<double, 7>{12, 34, 0, 2, 6, 12, 20}));
    assert(calls == 7);
    calls = 0;
    assert((parameters(3, "outpost") == std::array<double, 7>{10, .1, 0, 1, 2, 3, 4}));
    assert((parameters(1, "base") == std::array<double, 7>{100, 400, 1e-6, 1, 2, 3, 4}));
    assert(calls == 0);
}
''', encoding="utf-8")
        executable = self.root / "parameters.exe"
        env = os.environ.copy()
        env["PATH"] = str(Path(shutil.which("g++")).parent) + os.pathsep + env.get("PATH", "")
        subprocess.run([shutil.which("g++"), "-std=c++17", str(source), "-o", str(executable)],
                       check=True, capture_output=True, text=True, env=env)
        subprocess.run([str(executable)], check=True, capture_output=True, text=True, env=env)



if __name__ == "__main__":
    unittest.main()
