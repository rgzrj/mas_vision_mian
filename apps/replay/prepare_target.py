"""Generate replay-only runtime parameters and observation calls; originals stay unchanged."""
import difflib
from pathlib import Path
import re
import sys


def instrument(text):
    namespace = "namespace auto_aim\n{\n"
    update = ("    update_ypda(armor, best_id);\n"
              "    if (!ekf_.last_update_applied) return false;\n"
              "    constrainState();")
    if text.count(namespace) != 1 or text.count(update) != 1:
        raise ValueError("Unknown Target update layout; original files were not changed")
    text = text.replace(namespace, namespace +
        "// Replay-only definitions are supplied by apps/replay/main.cpp.\n"
        "double replay_noise(const char *, double);\n"
        "double replay_r_scale(int);\n"
        "void replay_observation_update(const Armor &, const Target &, const Eigen::VectorXd &,\n"
        "                               const Eigen::Vector4d &, bool);\n", 1)
    for key in ("v1", "v2", "q_h"):
        pattern = rf"(?m)^(\s*double[ \t]+{key}[ \t]*=[ \t]*)([^;\n]+)(;)"
        text, count = re.subn(pattern, lambda m: m[1] +
            f'(armor_num == 4 ? replay_noise("{key}", ({m[2]})) : ({m[2]}))' + m[3], text)
        if count != 1:
            raise ValueError(f"Cannot locate unique {key}; original files were not changed")
    pattern = r"(?m)^([ \t]*)Eigen::MatrixXd R = R_diag\.asDiagonal\(\);"
    text, count = re.subn(pattern, lambda m:
        m[1] + "if (armor_num == 4)\n" + m[1] +
        "    for (int replay_i = 0; replay_i < 4; ++replay_i) R_diag[replay_i] *= replay_r_scale(replay_i);\n" + m[0], text)
    if count != 1:
        raise ValueError("Cannot locate unique R_diag; original files were not changed")
    return text.replace(update,
        "    const Eigen::VectorXd replay_prior = ekf_.x;\n"
        "    update_ypda(armor, best_id);\n"
        "    if (!ekf_.last_update_applied)\n"
        "    {\n"
        "        replay_observation_update(armor, *this, replay_prior, predicted[best_id], false);\n"
        "        return false;\n"
        "    }\n"
        "    constrainState();\n"
        "    replay_observation_update(armor, *this, replay_prior, predicted[best_id], true);", 1)


def prepare(source, destination):
    before = source.read_text(encoding="utf-8")
    after = instrument(before)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(after, encoding="utf-8")
    destination.with_name("replay_instrumentation.patch").write_text("".join(difflib.unified_diff(
        before.splitlines(True), after.splitlines(True), fromfile="original/armor_target.cpp",
        tofile="replay-build/armor_target.cpp")), encoding="utf-8")


if __name__ == "__main__":
    prepare(Path(sys.argv[1]), Path(sys.argv[2]))
