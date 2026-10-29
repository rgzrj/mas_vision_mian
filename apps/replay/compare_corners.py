"""Offline corner comparison; Python reproduction, not the native tracking pipeline.

Usage: python apps/replay/compare_corners.py VIDEO --roi 0 140 1440 180
Outputs are isolated under data/replay/corner_compare_*. No camera calibration is assumed.
"""
import argparse
import csv
import hashlib
import itertools
import json
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
METHODS = ("A_raw", "B_gradient", "C_axis", "D_shi_pair", "E_shi_peak", "F_harris_peak")
COLORS = ((255, 180, 0), (0, 220, 255), (0, 220, 0), (255, 0, 230), (255, 220, 160), (80, 120, 255))


def bar(rect):
    corners = cv2.boxPoints(rect)
    corners = corners[np.argsort(corners[:, 1], kind="stable")]
    ends = np.array([corners[:2].mean(0), corners[2:].mean(0)], np.float32)
    length = float(np.linalg.norm(ends[1] - ends[0]))
    width = float(np.linalg.norm(corners[1] - corners[0]))
    return dict(rect=rect, ends=ends, center=ends.mean(0), length=length, width=width)


def lights(image, roi, threshold):
    x, y, w, h = roi
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    mask = cv2.threshold(gray[y:y+h, x:x+w], threshold, 255, cv2.THRESH_BINARY)[1]
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    found = []
    for contour in contours:
        if len(contour) < 4 or cv2.contourArea(contour) > 30000:
            continue
        contour = contour + np.array([x, y], np.int32)
        b = bar(cv2.minAreaRect(contour))
        delta = b["ends"][1] - b["ends"][0]
        if (b["width"] > 0 and b["length"] >= 8
                and 1.5 <= b["length"] / b["width"] <= 20
                and abs(np.arctan2(delta[1], delta[0]) - np.pi/2) <= np.pi/4):
            found.append(b)
    return gray, sorted(found, key=lambda b: b["center"][0])


def classify(image, left, right, net, labels):
    # Same perspective crop and preprocessing as NumberClassifier::extractNumber.
    lt, lb = left["ends"]
    rt, rb = right["ends"]
    ratio = np.linalg.norm(right["center"]-left["center"])/max(left["length"], right["length"])
    width = 54 if ratio > 3 else 32
    src = np.float32([left["center"]+(lb-lt)*1.125,
                      left["center"]-(lb-lt)*1.125,
                      right["center"]-(rb-rt)*1.125,
                      right["center"]+(rb-rt)*1.125])
    dst = np.float32([[0, 27], [0, 0], [width-1, 0], [width-1, 27]])
    number = cv2.warpPerspective(image, cv2.getPerspectiveTransform(src, dst), (width, 28))
    number = cv2.cvtColor(number[:, (width-20)//2:(width+20)//2], cv2.COLOR_BGR2GRAY)
    number = cv2.threshold(number, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]
    number = cv2.resize(number, (28, 28)).astype(np.float32)/255
    net.setInput(cv2.dnn.blobFromImage(number))
    scores = net.forward().ravel()
    i = int(np.argmax(scores))
    return labels[i], float(scores[i])


def select_board(image, found, net, labels, number):
    candidates = []
    for left, right in itertools.combinations(found, 2):
        length = max(left["length"], right["length"])
        ratio = np.linalg.norm(right["center"]-left["center"])/length
        if not 1 <= ratio <= 5 or length/min(left["length"], right["length"]) > 1.5:
            continue
        roll = np.arctan2(*(right["center"]-left["center"])[::-1])
        angles = [np.arctan2(*(b["ends"][1]-b["ends"][0])[::-1]) for b in (left, right)]
        if max(abs(np.cos(a-roll)) for a in angles) > 25/57.3:
            continue
        name, confidence = classify(image, left, right, net, labels)
        if name == number and confidence >= .7:
            candidates.append((confidence, left, right))
    return max(candidates, key=lambda c: c[0]) if candidates else None


def bilinear(gray, points):
    x, y = np.floor(points).astype(np.int32).T
    valid = (x >= 0) & (x < gray.shape[1]-1) & (y >= 0) & (y < gray.shape[0]-1)
    values = np.zeros(len(points), np.float32)
    u, v = (points[valid]-np.column_stack((x[valid], y[valid]))).T
    x, y = x[valid], y[valid]
    values[valid] = ((1-u)*(1-v)*gray[y, x] + u*(1-v)*gray[y, x+1]
                     + (1-u)*v*gray[y+1, x] + u*v*gray[y+1, x+1])
    return values


def gradient(gray, b):
    # Reproduce light_corner_corrector.cpp, including ROI truncation and 0.25px difference.
    x, y, w, h = cv2.boundingRect(cv2.boxPoints(b["rect"]))
    x -= int(w*.1)
    y -= int(h*.1)
    w += int(2*w*.1)
    h += int(2*h*.1)
    x0, y0, x1, y1 = max(0, x), max(0, y), min(gray.shape[1], x+w), min(gray.shape[0], y+h)
    roi = gray[y0:y1, x0:x1]
    if roi.size == 0:
        return b["ends"].copy(), b["ends"].copy(), False
    binary = cv2.threshold(roi, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return b["ends"].copy(), b["ends"].copy(), False
    rect = cv2.minAreaRect(max(contours, key=cv2.contourArea))
    center = np.float32(rect[0])+np.float32([x0, y0])
    rw, rh = rect[1]
    if min(rw, rh) <= 0:
        return b["ends"].copy(), b["ends"].copy(), False
    angle = np.deg2rad(rect[2]+(90 if rw < rh else 0))
    axis = np.float32([np.cos(angle), np.sin(angle)])
    if axis[1] < 0:
        axis = -axis
    normal = np.float32([-axis[1], axis[0]])
    length, width = max(rw, rh), min(rw, rh)
    mean = float(roi.mean())
    ends, constrained, success = [], [], True
    for direction in (-1, 1):
        delta = axis*direction
        candidates = []
        for offset in range(-max(1, int(width/2)), max(1, int(width/2))+1):
            start = center+length*.4*delta+normal*offset
            current = start+np.arange(0, length*.2, .5, dtype=np.float32)[:, None]*delta
            inside = ((current[:, 0] >= 0) & (current[:, 0] < gray.shape[1])
                      & (current[:, 1] >= 0) & (current[:, 1] < gray.shape[0]))
            invalid = np.flatnonzero(~inside)
            if len(invalid):
                current = current[:invalid[0]]
            if not len(current):
                continue
            previous = current-delta*.25
            pv, cv = bilinear(gray, previous), bilinear(gray, current)
            difference = pv-cv
            difference[pv <= mean] = 0
            i = int(np.argmax(difference))
            if difference[i] > 0:
                candidates.append(previous[i])
        point = np.mean(candidates, axis=0) if candidates else center+axis*length*.5*direction
        success &= bool(candidates)
        ends.append(point)
        constrained.append(center+axis*np.dot(point-center, axis))
    return np.float32(ends), np.float32(constrained), success


def shi(gray, b, pair=True, harris=False):
    axis = (b["ends"][1]-b["ends"][0])/b["length"]
    normal = np.array([-axis[1], axis[0]])
    center, length, width = b["center"], b["length"], b["width"]
    result = []
    for direction in (-1, 1):
        expected = center+axis*length*.5*direction
        along_radius, across_radius = max(3., length*.2), max(3., width*.9)
        region = np.array([expected+axis*a+normal*n for a in (-along_radius, along_radius)
                           for n in (-across_radius, across_radius)])
        x, y = np.floor(region.min(0)).astype(int)
        x1, y1 = np.ceil(region.max(0)).astype(int)+1
        x, y, x1, y1 = max(0, x), max(0, y), min(gray.shape[1], x1), min(gray.shape[0], y1)
        roi = gray[y:y1, x:x1]
        if min(roi.shape, default=0) < 5:
            return b["ends"].copy(), False
        corners = cv2.goodFeaturesToTrack(roi, 12, .03, max(1., width*.25), blockSize=3,
                                         useHarrisDetector=harris, k=.04)
        if corners is None:
            return b["ends"].copy(), False
        candidates = corners.reshape(-1, 2)+np.float32([x, y])
        window = max(1, min(3, int((width-1)/3 if pair else width*.5)))
        margin = window+1
        candidates = candidates[(candidates[:, 0] >= margin) & (candidates[:, 0] < gray.shape[1]-margin)
                                & (candidates[:, 1] >= margin) & (candidates[:, 1] < gray.shape[0]-margin)]
        if len(candidates) < (2 if pair else 1):
            return b["ends"].copy(), False
        before = candidates.copy()
        cv2.cornerSubPix(gray, candidates, (window, window), (-1, -1),
                         (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 20, .01))
        candidates = candidates[np.linalg.norm(candidates-before, axis=1) <= max(1., width*.5)]
        if not pair:
            good = [p for p in candidates if abs(np.dot(p-expected, axis)) <= along_radius
                    and abs(np.dot(p-center, normal)) <= width*.5]
            if not good:
                return b["ends"].copy(), False
            result.append(min(good, key=lambda p: abs(np.dot(p-center, normal))
                              +.5*abs(np.dot(p-expected, axis))))
            continue
        pairs = []
        for p, q in itertools.combinations(candidates, 2):
            p_off, q_off = np.dot(p-center, normal), np.dot(q-center, normal)
            midpoint = (p+q)*.5
            axial = abs(np.dot(midpoint-expected, axis))
            lateral = abs(np.dot(midpoint-center, normal))
            span = abs(p_off-q_off)
            if (p_off*q_off < 0 and .3*width <= span <= 2*width
                    and axial <= along_radius and lateral <= width*.5
                    and abs(np.dot(p-q, axis)) <= max(2., length*.15)):
                score = axial+2*lateral+abs(span-width)*.5
                pairs.append((score, midpoint))
        if not pairs:
            return b["ends"].copy(), False
        result.append(min(pairs, key=lambda p: p[0])[1])
    return np.float32(result), True


def features(points):
    # Points: LT, RT, RB, LB. Relative points remove common image translation only.
    center = points.mean(0)
    l, r = points[3]-points[0], points[2]-points[1]
    connection = (points[1]+points[2]-points[0]-points[3])*.5
    roll = np.arctan2(connection[1], connection[0])
    angles = np.array([np.arctan2(l[1], l[0]), np.arctan2(r[1], r[0])])
    error = np.rad2deg(np.arcsin(np.clip(max(abs(np.cos(angles-roll))), 0, 1)))
    result = dict(center_x=float(center[0]), center_y=float(center[1]),
                  length_px=float((np.linalg.norm(l)+np.linalg.norm(r))*.5),
                  left_angle_deg=float(np.rad2deg(angles[0])), right_angle_deg=float(np.rad2deg(angles[1])),
                  rectangular_error_deg=float(error))
    for i, p in enumerate(points):
        result.update({f"p{i}_x": float(p[0]), f"p{i}_y": float(p[1]),
                       f"relative_p{i}_x": float(p[0]-center[0]), f"relative_p{i}_y": float(p[1]-center[1])})
    return result


def residual(values):
    # Same fixed 11-frame quadratic trend for all methods; not used to alter observations.
    if len(values) < 11:
        return np.array([])
    x = np.arange(-5, 6, dtype=float)
    coefficients = np.linalg.pinv(np.column_stack([np.ones(11), x, x*x]))[0]
    return values[5:-5]-np.convolve(values, coefficients, mode="valid")


def summarize(rows, fps):
    summaries = {}
    for method in METHODS:
        data = [r for r in rows if r["method"] == method]
        ids = np.array([r["frame"] for r in data])
        cuts = np.r_[0, np.flatnonzero(np.diff(ids) != 1)+1, len(ids)]
        segments = [data[a:b] for a, b in zip(cuts[:-1], cuts[1:]) if b-a >= 2]
        summary = dict(frames=len(data), success_frames=sum(r["success"] for r in data),
                       success_rate=float(np.mean([r["success"] for r in data])) if data else 0,
                       left_success_frames=sum(r["left_success"] for r in data),
                       right_success_frames=sum(r["right_success"] for r in data),
                       pixel_only=True, trend_window_seconds=11/fps,
                       residual_samples=sum(max(0, len(s)-10) for s in segments))
        for name in ("length_px", "left_angle_deg", "right_angle_deg", "rectangular_error_deg"):
            values = np.array([r[name] for r in data])
            errors = [residual(np.array([r[name] for r in s])) for s in segments]
            errors = np.concatenate(errors) if errors else np.array([])
            summary[name] = dict(mean=float(values.mean()) if len(values) else None,
                                 trend_residual_rms=float(np.sqrt(np.mean(errors**2))) if len(errors) else None,
                                 trend_residual_p95=float(np.percentile(abs(errors), 95)) if len(errors) else None)
        jumps = []
        for segment in segments:
            p = np.array([[[r[f"relative_p{i}_{axis}"] for axis in ("x", "y")] for i in range(4)] for r in segment])
            jumps.extend(np.linalg.norm(np.diff(p, axis=0), axis=2).ravel())
        summary["shape_step_p95_px"] = float(np.percentile(jumps, 95)) if jumps else None
        summary["shape_step_max_px"] = float(max(jumps)) if jumps else None
        summary["mean_method_ms"] = float(np.mean([r["method_ms"] for r in data])) if data else None
        summaries[method] = summary
    return summaries


def self_check():
    gray = np.zeros((100, 100), np.uint8)
    gray[25:76, 45:56] = 240
    b = bar(cv2.minAreaRect(np.array([[[45, 25]], [[55, 25]], [[55, 75]], [[45, 75]]], np.int32)))
    raw, axis, ok = gradient(gray, b)
    assert ok and raw[0, 1] < raw[1, 1]
    assert np.max(abs(axis[:, 0]-50)) < 1e-4
    d, ok = shi(gray, b)
    assert ok and np.max(abs(d[:, 0]-50)) < .1 and d[0, 1] < d[1, 1]
    empty = np.zeros_like(gray)
    a, c, ok = gradient(empty, b)
    assert not ok and np.all(np.isfinite(a)) and np.all(np.isfinite(c))
    d, ok = shi(empty, b)
    assert not ok and np.allclose(d, b["ends"])
    narrow = np.zeros_like(gray)
    narrow[25:76, 49:52] = 240
    narrow = cv2.GaussianBlur(narrow, (5, 5), 1)
    contour = cv2.findContours(cv2.threshold(narrow, 90, 255, 0)[1],
                               cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0][0]
    narrow_bar = bar(cv2.minAreaRect(contour))
    for harris in (False, True):
        d, ok = shi(narrow, narrow_bar, pair=False, harris=harris)
        assert ok and np.max(abs(d[:, 0]-50)) < .1 and d[0, 1] < d[1, 1]
    assert np.max(abs(residual(np.arange(50.)**2))) < 1e-9
    print("Self-check passed: endpoint order, common axis, Shi pairing, fallback, motion detrending.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("video", type=Path, nargs="?")
    parser.add_argument("--roi", nargs=4, type=int, help="x y width height, in original image pixels")
    parser.add_argument("--number", default="2")
    parser.add_argument("--threshold", type=int, default=90)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--self-check", action="store_true")
    args = parser.parse_args()
    if args.self_check:
        self_check()
        if args.video is None:
            return
    if args.video is None:
        parser.error("video is required")
    cv2.setNumThreads(1)
    cap = cv2.VideoCapture(str(args.video))
    if not cap.isOpened():
        raise ValueError(f"Cannot open {args.video}")
    fps = cap.get(cv2.CAP_PROP_FPS)
    advertised_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    w, h = int(cap.get(3)), int(cap.get(4))
    roi = args.roi or (0, 0, w, h)
    if roi[0] < 0 or roi[1] < 0 or min(roi[2:]) <= 0 or roi[0]+roi[2] > w or roi[1]+roi[3] > h:
        raise ValueError("ROI is outside the image")
    output = args.output or ROOT/"data/replay"/datetime.now().strftime("corner_compare_%Y%m%d_%H%M%S")
    output.mkdir(parents=True, exist_ok=False)
    net = cv2.dnn.readNetFromONNX(str(ROOT/"auto_aim/armor_detector/model/lenet.onnx"))
    labels = (ROOT/"auto_aim/armor_detector/model/label.txt").read_text().splitlines()
    rows, coverage = [], []
    writer = cv2.VideoWriter(str(output/"comparison.avi"), cv2.VideoWriter_fourcc(*"MJPG"), fps, (960, 900))
    if not writer.isOpened():
        raise ValueError("Cannot create comparison video")
    source_paths = [Path(__file__), ROOT/"auto_aim/armor_detector/light_corner_corrector.cpp",
                    ROOT/"auto_aim/armor_detector/armor_detector.cpp", ROOT/"auto_aim/armor_detector/number_classifier.cpp"]
    manifest = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in source_paths}
    frame_id = 0
    started = time.perf_counter()
    while not args.max_frames or frame_id < args.max_frames:
        ok, image = cap.read()
        if not ok:
            break
        gray, found = lights(image, roi, args.threshold)
        selected = select_board(image, found, net, labels, args.number)
        coverage.append(dict(frame=frame_id, time_s=frame_id/fps, lights=len(found), selected=selected is not None))
        canvas = np.zeros((900, 960, 3), np.uint8)
        if selected is not None:
            confidence, left, right = selected
            results = {m: [] for m in METHODS}
            success = {m: True for m in METHODS}
            per_bar_success = {m: [] for m in METHODS}
            timings = {m: 0. for m in METHODS}
            for b in (left, right):
                results["A_raw"].append(b["ends"])
                per_bar_success["A_raw"].append(True)
                begin = time.perf_counter()
                bp, cp, good = gradient(gray, b)
                elapsed = (time.perf_counter()-begin)*1000
                for m, points in (("B_gradient", bp), ("C_axis", cp)):
                    results[m].append(points)
                    success[m] &= good
                    per_bar_success[m].append(good)
                    timings[m] += elapsed
                for m, pair, harris in (("D_shi_pair", True, False), ("E_shi_peak", False, False),
                                        ("F_harris_peak", False, True)):
                    begin = time.perf_counter()
                    dp, good = shi(gray, b, pair=pair, harris=harris)
                    results[m].append(dp)
                    success[m] &= good
                    per_bar_success[m].append(good)
                    timings[m] += (time.perf_counter()-begin)*1000
            center = (left["center"]+right["center"])*.5
            x, y = np.maximum(0, np.floor(center-[60, 35])).astype(int)
            x, y = min(x, w-120), min(y, h-70)
            for index, method in enumerate(METHODS):
                le, re = results[method]
                points = np.array([le[0], re[0], re[1], le[1]])
                row = dict(frame=frame_id, time_s=frame_id/fps, method=method, confidence=confidence,
                           success=success[method], left_success=per_bar_success[method][0],
                           right_success=per_bar_success[method][1], raw_left_width_px=left["width"],
                           raw_right_width_px=right["width"], method_ms=timings[method], **features(points))
                rows.append(row)
                tile = cv2.resize(image[y:y+70, x:x+120], (480, 280), interpolation=cv2.INTER_NEAREST)
                pp = (points-[x, y])*4
                cv2.polylines(tile, [np.rint(pp).astype(np.int32)], True, COLORS[index], 1)
                for p in pp:
                    cv2.drawMarker(tile, tuple(np.rint(p).astype(int)), COLORS[index], cv2.MARKER_CROSS, 9, 1)
                panel = np.zeros((300, 480, 3), np.uint8)
                panel[:280] = tile
                label = f'{method} len={row["length_px"]:.2f} err={row["rectangular_error_deg"]:.2f} {"OK" if success[method] else "FALLBACK"}'
                cv2.putText(panel, label, (5, 295), 0, .45, COLORS[index], 1)
                yy, xx = index//2*300, index%2*480
                canvas[yy:yy+300, xx:xx+480] = panel
            if frame_id in (0, 232, 464, 773, 1082, 1450):
                cv2.imwrite(str(output/f"frame_{frame_id:04d}.png"), canvas)
        else:
            cv2.putText(canvas, "NO MATCHED BOARD", (20, 90), 0, 1, (0, 0, 255), 2)
        cv2.putText(canvas, f"frame {frame_id} t={frame_id/fps:.3f}s", (8, 20), 0, .5, (255, 255, 255), 1)
        writer.write(canvas)
        frame_id += 1
        if frame_id % 200 == 0:
            print(f"Processed {frame_id}; board frames={len(rows)//len(METHODS)}", flush=True)
    cap.release()
    writer.release()
    if not rows:
        raise ValueError("No matched board; inspect ROI, threshold and classifier results")
    for name, data in (("corners.csv", rows), ("coverage.csv", coverage)):
        with (output/name).open("w", newline="", encoding="utf-8-sig") as f:
            writer_csv = csv.DictWriter(f, list(data[0]))
            writer_csv.writeheader()
            writer_csv.writerows(data)
    summary = dict(video=str(args.video.resolve()), fps=fps, advertised_frames=advertised_frames, decoded_frames=frame_id,
                   selected_frames=len(rows)//len(METHODS), roi=list(roi), threshold=args.threshold, number=args.number,
                   implementation="Python reproduction, shared raw detections, existing ONNX classifier",
                   calibration=None, pose_evaluation=False, elapsed_s=time.perf_counter()-started,
                   warning="Moving target: high-pass residual is a proxy, not ground-truth corner error. Fallbacks included.",
                   source_sha256=manifest, methods=summarize(rows, fps))
    summary["settings"] = dict(shi_quality=.03, corner_block_size=3, harris_k=.04,
                               gradient_step_px=.5, gradient_difference_px=.25,
                               trend_window_frames=11, trend_polynomial_degree=2,
                               fallbacks="Per lightbar: retain both raw endpoints when that bar fails",
                               timing="Python additional refinement only; B/C share one timed gradient call")
    # Also compare only common frames where both gradient endpoints and both Shi bars succeeded.
    common = {r["frame"] for r in rows if r["method"] == "D_shi_pair" and r["success"]}
    common &= {r["frame"] for r in rows if r["method"] == "B_gradient" and r["success"]}
    summary["common_success_frames"] = len(common)
    summary["common_success_methods"] = summarize([r for r in rows if r["frame"] in common], fps)
    common = {r["frame"] for r in rows if r["method"] == "E_shi_peak" and r["success"]}
    common &= {r["frame"] for r in rows if r["method"] == "F_harris_peak" and r["success"]}
    common &= {r["frame"] for r in rows if r["method"] == "B_gradient" and r["success"]}
    summary["peak_common_success_frames"] = len(common)
    summary["peak_common_success_methods"] = summarize([r for r in rows if r["frame"] in common], fps)
    (output/"summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
    for m in METHODS:
        records = [r for r in rows if r["method"] == m]
        t = [r["time_s"] for r in records]
        for ax, name in zip(axes, ("length_px", "left_angle_deg", "rectangular_error_deg")):
            ax.plot(t, [r[name] for r in records], label=m, linewidth=.7, alpha=.8)
            ax.set_ylabel(name)
            ax.grid(alpha=.25)
    axes[0].legend(ncol=4)
    axes[-1].set_xlabel("Video time (s); actual translation retained")
    fig.tight_layout()
    fig.savefig(output/"comparison.png", dpi=150)
    plt.close(fig)
    assert all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest() == digest for p, digest in manifest.items())
    print(json.dumps({"output": str(output), "decoded": frame_id, "selected": len(rows)//len(METHODS),
                      "peak_common_success": len(common)}, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
