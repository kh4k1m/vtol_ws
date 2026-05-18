#!/usr/bin/env python3
"""Offline helpers for AprilTag + GPS sanity checks.

* ``video`` — rough OpenCV demo on an MP4 (legacy playground).
* ``analyze-session`` — read a flight ``session_*`` bag and estimate the
  scale factor between tag-localizer Z and ArduPilot ``/ap/relative_alt``.
  A slope >> 1 means printed ``size`` in ``config/markers.yaml`` is too
  small for the physical tag edge used in ``tag_detector_node`` PnP.

Run from the workspace root, for example::

    python visual_positioning_demo.py analyze-session logs/session_20260506-195925
"""

from __future__ import annotations

import argparse
import math
import os
import sys


def _ensure_workspace_on_path():
    root = os.environ.get('VTOL_WS_ROOT', os.path.expanduser('~/vtol_ws'))
    if root not in sys.path:
        sys.path.insert(0, root)


def _analyze_session(session_dir: str, expected_depth_scale: float | None = None) -> int:
    _ensure_workspace_on_path()
    import numpy as np

    from tools.flight_viz import readers

    session_dir = os.path.abspath(session_dir)
    bag_dir = readers.find_bag_dir(session_dir)
    if not bag_dir:
        print(f'No bag under {session_dir}', file=sys.stderr)
        return 1

    topics = ['/ap/relative_alt', '/tag_localizer/pose']
    try:
        bag = readers.read_bag(bag_dir, topics)
    except Exception as exc:
        print(f'Bag read failed: {exc}', file=sys.stderr)
        return 1

    rel = bag.get('/ap/relative_alt')
    pose = bag.get('/tag_localizer/pose')
    if rel is None or rel.is_empty():
        print('Bag has no /ap/relative_alt samples.', file=sys.stderr)
        return 1
    if pose is None or pose.is_empty():
        print('Bag has no /tag_localizer/pose samples.', file=sys.stderr)
        return 1

    rt = np.array(rel.times_ns, dtype=np.int64)
    rv = np.array([float(getattr(m, 'data')) for m in rel.values], dtype=float)
    tt, tx, ty, tz = readers.extract_pose_xyz(pose)
    tt = np.array(tt, dtype=np.int64)
    tz = np.asarray(tz, dtype=float)

    t0 = max(int(rt[0]), int(tt[0]))
    t1 = min(int(rt[-1]), int(tt[-1]))
    if t1 <= t0:
        print('Tag poses and relative_alt do not overlap in time.', file=sys.stderr)
        return 1

    def near_rel_alt(t_ns: int) -> float:
        j = int(np.argmin(np.abs(rt - t_ns)))
        return float(rv[j])

    pairs = []
    for t_ns, z in zip(tt, tz):
        if t_ns < t0 or t_ns > t1:
            continue
        if not math.isfinite(z):
            continue
        pairs.append((near_rel_alt(int(t_ns)), float(z)))
    if len(pairs) < 30:
        print(f'Only {len(pairs)} overlapping samples; need more tag poses.', file=sys.stderr)
        return 1

    pairs = np.asarray(pairs, dtype=float)
    rel_alt = pairs[:, 0]
    tag_z = pairs[:, 1]
    A = np.c_[np.ones(len(tag_z)), tag_z]
    coef, *_ = np.linalg.lstsq(A, rel_alt, rcond=None)
    intercept, slope = float(coef[0]), float(coef[1])
    corr = float(np.corrcoef(rel_alt, tag_z)[0, 1])
    pred = intercept + slope * tag_z
    resid = rel_alt - pred
    rmse = float(np.sqrt(np.mean(resid * resid)))
    mae = float(np.mean(np.abs(resid)))
    ss_res = float(np.sum(resid * resid))
    ss_tot = float(np.sum((rel_alt - np.mean(rel_alt)) ** 2))
    r_sq = float(1.0 - ss_res / ss_tot) if ss_tot > 1e-12 else float('nan')

    print(f'Session: {session_dir}')
    print(f'Overlapping samples: {len(pairs)}')
    print(f'Linear fit: relative_alt ≈ {intercept:.3f} m + ({slope:.4f}) * tag_z')
    print(f'Pearson corr(rel_alt, tag_z): {corr:.4f}')
    print(f'Fit quality: R²={r_sq:.4f}, RMSE={rmse:.3f} m, MAE={mae:.3f} m')
    if expected_depth_scale is not None and expected_depth_scale > 0:
        delta_pct = 100.0 * (slope - expected_depth_scale) / expected_depth_scale
        print(
            f'Expected depth scale (e.g. from marker size ratio): '
            f'{expected_depth_scale:.4f}; fitted slope differs by {delta_pct:+.2f}%.'
        )
    if corr < 0.9:
        print(
            'Warning: low correlation — slope is not a reliable scale hint '
            '(clipped flight, baro drift, or map frame issues).',
            file=sys.stderr,
        )
    else:
        print()
        print(
            'Interpretation: if the physical AprilTag outer edge length is L '
            'metres but config/markers.yaml lists a smaller size for that id, '
            'depth (and tag_z) is underestimated. Multiply the affected '
            f'"size" entries by ≈ {slope:.3f} (and default_marker_size if '
            'unknown ids use it), then clear or regenerate a bad '
            'discovered_map.yaml before the next flight.'
        )
        print()
        print(
            'After editing sizes, re-run tag_detector with the same '
            'camera.yaml intrinsics; optional: delete bogus learned markers '
            '(large |Z| in the persisted map) so online learning can rebuild.'
        )

    print()
    print(
        'Averaging vs systematic error (why "more frames" is not always '
        'more absolute truth):'
    )
    print(
        '  * Random frame-to-frame noise (PnP scatter, discretisation) '
        'averages down if you fuse many independent views — your tagDB '
        'sliding window / Kalman already does part of this.'
    )
    print(
        '  * Biases do NOT average to zero: wrong focal length, distortion, '
        'baro offset/drift vs geometric height, anchor datum, time skew. '
        'Those look like slope/intercept or structured residuals; more '
        'samples only estimate them more tightly, they do not disappear.'
    )
    print(
        '  * VO: monocular scale is weak without IMU/gravity/tags; '
        'improving shared camera intrinsics + time sync helps both tags '
        'and VO.'
    )

    manifest_path = os.path.join(session_dir, 'manifest.yaml')
    if os.path.isfile(manifest_path):
        import yaml

        with open(manifest_path, 'r', encoding='utf-8') as f:
            man = yaml.safe_load(f) or {}
        mp = man.get('marker_detector_profile')
        if mp:
            print()
            print('Recorded at flight (manifest marker_detector_profile):')
            print(f"  default_marker_size: {mp.get('default_marker_size')}")
            n = len(mp.get('markers') or [])
            print(f'  markers list: {n} entries (see manifest for ids/sizes)')

    return 0


def _run_video_demo(video_path: str) -> None:
    import cv2
    import numpy as np

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print('Error opening video')
        return

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    focal_length = 3000.0
    camera_matrix = np.array(
        [[focal_length, 0, width / 2], [0, focal_length, height / 2], [0, 0, 1]],
        dtype=float,
    )
    dist_coeffs = np.zeros((5, 1))

    marker_size = 0.8

    anchor_id = 4
    anchor_lat, anchor_lon = 65.988106, 117.622628

    relative_positions = {
        4: np.array([0.0, 0.0, 0.0]),
        3: np.array([-5.0, 0.0, 0.0]),
        5: np.array([5.0, 0.0, 0.0]),
    }

    def get_gps_from_offset(lat0, lon0, dx, dy):
        r_earth = 6378137.0
        d_lat = dy / r_earth
        d_lon = dx / (r_earth * math.cos(math.pi * lat0 / 180.0))
        return lat0 + (d_lat * 180.0 / math.pi), lon0 + (d_lon * 180.0 / math.pi)

    frame_count = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        frame_count += 1
        if frame_count % 10 != 0:
            continue

        corners, ids, _rejected = detector.detectMarkers(frame)
        if ids is None:
            continue

        obj_points = np.array(
            [
                [-marker_size / 2, marker_size / 2, 0],
                [marker_size / 2, marker_size / 2, 0],
                [marker_size / 2, -marker_size / 2, 0],
                [-marker_size / 2, -marker_size / 2, 0],
            ],
            dtype=np.float32,
        )

        for i, marker_id in enumerate(ids.flatten()):
            if marker_id not in relative_positions:
                continue
            success, rvec, tvec = cv2.solvePnP(
                obj_points, corners[i][0], camera_matrix, dist_coeffs
            )
            if not success:
                continue
            rmat, _ = cv2.Rodrigues(rvec)
            camera_pos_marker_frame = -np.dot(rmat.T, tvec).flatten()
            marker_offset = relative_positions[marker_id]
            dx = camera_pos_marker_frame[0] + marker_offset[0]
            dy = camera_pos_marker_frame[1] + marker_offset[1]
            dz = camera_pos_marker_frame[2]
            drone_lat, drone_lon = get_gps_from_offset(anchor_lat, anchor_lon, dx, dy)
            print(
                f'Frame {frame_count} | ID {marker_id} | '
                f'rel m X={dx:.2f} Y={dy:.2f} Z={dz:.2f} | '
                f'GPS {drone_lat:.6f}, {drone_lon:.6f}'
            )
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.drawFrameAxes(
                frame, camera_matrix, dist_coeffs, rvec, tvec, marker_size / 2
            )

    cap.release()
    print('Processing complete.')


def main() -> int:
    parser = argparse.ArgumentParser(description='AprilTag / GPS offline tools')
    sub = parser.add_subparsers(dest='cmd', required=True)

    p_vid = sub.add_parser('video', help='OpenCV demo on MP4 (hardcoded paths).')
    p_vid.add_argument(
        '--video',
        default='logs/session_20260506-195925/video/flight_video_000.mp4',
        help='Path to MP4',
    )

    p_an = sub.add_parser(
        'analyze-session',
        help='Estimate tag depth scale vs /ap/relative_alt from a session bag.',
    )
    p_an.add_argument('session_dir', help='Path to session_* directory')
    p_an.add_argument(
        '--expected-depth-scale',
        type=float,
        default=None,
        metavar='K',
        help='Optional theoretical depth ratio (e.g. 2.0 if tag size in YAML '
        'was half the printed edge). Prints %% difference vs fitted slope.',
    )

    args = parser.parse_args()
    if args.cmd == 'analyze-session':
        return _analyze_session(
            args.session_dir,
            expected_depth_scale=args.expected_depth_scale,
        )
    _run_video_demo(args.video)
    return 0


if __name__ == '__main__':
    sys.exit(main())
