#!/usr/bin/env python3
"""Evaluate FAST-LIVO2 IMU poses against Oxford Spires base-frame TUM GT.

Uses timestamp interpolation and rigid SE(3) alignment (no scale). The
published Oxford calibration defines LiDAR->base as [0, 0, .124] with a
180-degree yaw; the mapper YAML supplies LiDAR->IMU.
"""

import argparse
import pathlib

import numpy as np
import yaml
from scipy.spatial.transform import Rotation, Slerp


def load_tum(path):
    poses = np.loadtxt(path, comments="#")
    if poses.ndim == 1:
        poses = poses[None, :]
    if poses.shape[1] != 8:
        raise ValueError(f"Expected 8 TUM columns in {path}")
    poses = poses[np.argsort(poses[:, 0])]
    if np.any(np.diff(poses[:, 0]) <= 0):
        raise ValueError(f"Non-increasing timestamps in {path}")
    return poses


def rigid_align(source, target):
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    u, _, vt = np.linalg.svd((source - source_mean).T @ (target - target_mean))
    sign = np.linalg.det(u @ vt)
    correction = np.diag([1.0, 1.0, sign])
    rotation = vt.T @ correction @ u.T
    translation = target_mean - rotation @ source_mean
    return rotation, translation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("estimated", type=pathlib.Path)
    parser.add_argument("ground_truth", type=pathlib.Path)
    parser.add_argument("--config", type=pathlib.Path, required=True)
    parser.add_argument("--rpe-seconds", type=float, default=1.0)
    parser.add_argument("--duration", type=float, default=None,
                        help="Evaluate only this many seconds from the first estimated pose")
    parser.add_argument("--start-time", type=float, default=None,
                        help="Absolute timestamp lower bound (inclusive)")
    parser.add_argument("--end-time", type=float, default=None,
                        help="Absolute timestamp upper bound (inclusive)")
    args = parser.parse_args()

    estimated = load_tum(args.estimated)
    if args.duration is not None:
        estimated = estimated[estimated[:, 0] <= estimated[0, 0] + args.duration]
    if args.start_time is not None:
        estimated = estimated[estimated[:, 0] >= args.start_time]
    if args.end_time is not None:
        estimated = estimated[estimated[:, 0] <= args.end_time]
    ground_truth = load_tum(args.ground_truth)
    with args.config.open(encoding="utf-8") as stream:
        calibration = yaml.safe_load(stream)["extrin_calib"]

    r_imu_lidar = np.asarray(calibration["extrinsic_R"], dtype=float).reshape(3, 3)
    t_imu_lidar = np.asarray(calibration["extrinsic_T"], dtype=float)
    # T_base_lidar from Oxford Spires configs/sensor.yaml.
    r_base_lidar = np.diag([-1.0, -1.0, 1.0])
    t_base_lidar = np.array([0.0, 0.0, 0.124])
    r_imu_base = r_imu_lidar @ r_base_lidar.T
    t_imu_base = t_imu_lidar - r_imu_base @ t_base_lidar

    times = estimated[:, 0]
    valid = (times >= ground_truth[0, 0]) & (times <= ground_truth[-1, 0])
    estimated = estimated[valid]
    times = estimated[:, 0]
    if len(times) < 3:
        raise ValueError("Fewer than three time-overlapping estimated poses")

    r_est_imu = Rotation.from_quat(estimated[:, 4:8]).as_matrix()
    p_est_base = estimated[:, 1:4] + np.einsum("nij,j->ni", r_est_imu, t_imu_base)
    r_est_base = r_est_imu @ r_imu_base

    p_gt = np.column_stack(
        [np.interp(times, ground_truth[:, 0], ground_truth[:, i]) for i in range(1, 4)]
    )
    r_gt = Slerp(ground_truth[:, 0], Rotation.from_quat(ground_truth[:, 4:8]))(times).as_matrix()

    r_align, t_align = rigid_align(p_est_base, p_gt)
    p_aligned = (r_align @ p_est_base.T).T + t_align
    r_aligned = r_align @ r_est_base
    translation_error = np.linalg.norm(p_aligned - p_gt, axis=1)
    rotation_error = Rotation.from_matrix(np.transpose(r_gt, (0, 2, 1)) @ r_aligned).magnitude()

    rpe_translation = []
    rpe_rotation = []
    for i, timestamp in enumerate(times):
        target_time = timestamp + args.rpe_seconds
        insertion = np.searchsorted(times, target_time)
        neighbors = [j for j in (insertion - 1, insertion) if i < j < len(times)]
        if not neighbors:
            continue
        j = min(neighbors, key=lambda index: abs(times[index] - target_time))
        if abs(times[j] - target_time) > 0.05:
            continue
        delta_est = r_aligned[i].T @ (p_aligned[j] - p_aligned[i])
        delta_gt = r_gt[i].T @ (p_gt[j] - p_gt[i])
        rpe_translation.append(np.linalg.norm(delta_est - delta_gt))
        delta_rotation = (r_gt[i].T @ r_gt[j]).T @ (r_aligned[i].T @ r_aligned[j])
        rpe_rotation.append(np.rad2deg(Rotation.from_matrix(delta_rotation).magnitude()))

    rmse = lambda values: float(np.sqrt(np.mean(np.square(values))))
    print(f"Matched poses: {len(times)}; duration: {times[-1] - times[0]:.2f} s")
    print(f"ATE translation RMSE: {rmse(translation_error):.4f} m; median: {np.median(translation_error):.4f} m")
    print(f"ATE orientation RMSE: {np.rad2deg(rmse(rotation_error)):.3f} deg")
    if rpe_translation:
        print(f"RPE {args.rpe_seconds:g} s translation RMSE: {rmse(rpe_translation):.4f} m")
        print(f"RPE {args.rpe_seconds:g} s rotation RMSE: {rmse(rpe_rotation):.3f} deg")


if __name__ == "__main__":
    main()
