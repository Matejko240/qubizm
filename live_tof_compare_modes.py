import copy
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import pyvista as pv
import serial
from PyQt6 import QtCore, QtGui, QtWidgets
from pyvistaqt import QtInteractor

from live_tof_heatmap import (
    BAUD,
    CALIBRATION_ACC_STILL_G,
    CALIBRATION_DURATION_MS,
    CALIBRATION_EXPECTED_ACC_G,
    CALIBRATION_FRONT_UP_DOT_MIN,
    CALIBRATION_GYRO_STILL_DPS,
    CALIBRATION_MIN_STILL_RATIO,
    DEFAULT_IMU_AXIS_ORDER,
    DEFAULT_IMU_AXIS_SIGN,
    DEVICE_HALF_DEPTH_M,
    DEVICE_HALF_HEIGHT_M,
    DEVICE_HALF_WIDTH_M,
    DEVICE_SENSOR_FACE_LOCAL,
    MAP_DECAY_TAU_S,
    MAP_MIN_WEIGHT,
    PORT,
    PoseTracker,
    RECORDINGS_DIR,
    TOF_AXIS_HALF_RAD,
    TOF_MAX_RANGE_M,
    TOF_MIN_RANGE_M,
    TOF_SIZE,
    project_tof_to_local_points,
    robust_mean_and_std,
    rotation_matrix,
)


SERIAL_MAX_BYTES_PER_TICK = 65536
SERIAL_BACKLOG_DROP_BYTES = 262144
MAX_READS_PER_TICK = 100
UI_TIMER_MS = 50
RENDER_INTERVAL_MS = 120
FPV_RENDER_INTERVAL_MS = 160
STATUS_INTERVAL_MS = 500
PRUNE_INTERVAL_MS = 600
SCENE_MAX_RENDER_VOXELS = 2200
FPV_MAX_RENDER_VOXELS = 1200
GRID_BOUND_QUANT_M = 0.25
FPV_TILE_PX = 8
MIN_RENDER_STREAK = 2
TRANSLATION_ACC_LP = 0.97
TRANSLATION_VEL_DAMP = 0.55
TRANSLATION_POS_DAMP = 0.985
TRANSLATION_GAIN = 0.18
TRANSLATION_DEADBAND_G = 0.10
TRANSLATION_TRIGGER_G = 0.22
TRANSLATION_MAX_SPEED_MPS = 0.25
TRANSLATION_MAX_POS_M = 1.5


def transform_axes(values):
    if values is None or len(values) != 3:
        return None
    arr = np.asarray(values, dtype=float)
    arr = arr[list(DEFAULT_IMU_AXIS_ORDER)] * DEFAULT_IMU_AXIS_SIGN
    return arr


class SparseVoxelMap:
    def __init__(self, voxel_size=0.06, decay_tau=MAP_DECAY_TAU_S, min_weight=MAP_MIN_WEIGHT, merge_range_scale=1.0):
        self.voxel_size = float(voxel_size)
        self.decay_tau = float(decay_tau)
        self.min_weight = float(min_weight)
        self.merge_range_scale = float(merge_range_scale)
        self.cells = {}
        self._last_prune_ms = None
        self._footprint_cache = {}

    def clear(self):
        self.cells.clear()
        self._last_prune_ms = None

    def set_voxel_size(self, voxel_size):
        voxel_size = float(voxel_size)
        if abs(voxel_size - self.voxel_size) > 1e-9:
            self.voxel_size = voxel_size
            self._footprint_cache.clear()
            self.clear()

    def _key(self, point):
        return tuple(np.floor(np.asarray(point, dtype=float) / self.voxel_size).astype(int).tolist())

    def _center(self, key):
        key_arr = np.asarray(key, dtype=float)
        return (key_arr + 0.5) * self.voxel_size

    def _decayed(self, cell, tick_ms):
        if tick_ms is None:
            return float(cell["weight"])
        dt_s = max(0.0, (float(tick_ms) - float(cell["last_seen"])) / 1000.0)
        return float(cell["weight"]) * math.exp(-dt_s / max(self.decay_tau, 1e-6))

    def prune(self, tick_ms, force=False):
        if tick_ms is None:
            return
        if not force and self._last_prune_ms is not None and (float(tick_ms) - self._last_prune_ms) < PRUNE_INTERVAL_MS:
            return
        remove = []
        for key, cell in self.cells.items():
            w = self._decayed(cell, tick_ms)
            if w < self.min_weight:
                remove.append(key)
            else:
                cell["weight"] = w
                cell["last_seen"] = float(tick_ms)
        for key in remove:
            self.cells.pop(key, None)
        self._last_prune_ms = float(tick_ms)

    def _trace_keys(self, origin, endpoint):
        origin = np.asarray(origin, dtype=float)
        endpoint = np.asarray(endpoint, dtype=float)
        delta = endpoint - origin
        length = float(np.linalg.norm(delta))
        if length < 1e-9:
            return []
        steps = max(2, int(math.ceil(length / max(self.voxel_size * 0.5, 1e-4))))
        ts = np.linspace(0.0, 1.0, steps, endpoint=False)
        keys = []
        seen = set()
        for t in ts:
            key = self._key(origin + delta * t)
            if key not in seen:
                seen.add(key)
                keys.append(key)
        return keys

    def _merge_hit(self, key, distance, add_weight, tick_ms, frame_seq):
        existing = self.cells.get(key)
        if existing is None:
            self.cells[key] = {
                "weight": min(1.0, add_weight),
                "distance": float(distance),
                "last_seen": float(tick_ms),
                "last_frame_seq": int(frame_seq),
                "streak": 1,
            }
            return

        old_w = self._decayed(existing, tick_ms)
        total = min(1.0, old_w + add_weight)
        if (old_w + add_weight) > 1e-9:
            existing["distance"] = ((existing["distance"] * old_w) + (float(distance) * add_weight)) / (old_w + add_weight)
        else:
            existing["distance"] = float(distance)
        existing["weight"] = total
        existing["last_seen"] = float(tick_ms)
        last_frame_seq = int(existing.get("last_frame_seq", -1000000))
        if int(frame_seq) == last_frame_seq:
            pass
        elif int(frame_seq) == last_frame_seq + 1:
            existing["streak"] = int(existing.get("streak", 1)) + 1
        else:
            existing["streak"] = 1
        existing["last_frame_seq"] = int(frame_seq)

    def _footprint_offsets(self, radius_vox):
        radius_vox = int(max(0, radius_vox))
        cached = self._footprint_cache.get(radius_vox)
        if cached is not None:
            return cached
        offsets = []
        for dx in range(-radius_vox, radius_vox + 1):
            for dy in range(-radius_vox, radius_vox + 1):
                for dz in range(-radius_vox, radius_vox + 1):
                    offsets.append((dx, dy, dz))
        arr = np.asarray(offsets, dtype=int) if offsets else np.zeros((0, 3), dtype=int)
        self._footprint_cache[radius_vox] = arr
        return arr

    def add_hits(self, origin_world, hit_points_world, tick_ms, frame_seq):
        self.prune(tick_ms)
        if hit_points_world is None or len(hit_points_world) == 0:
            return
        origin_world = np.asarray(origin_world, dtype=float)
        points = np.asarray(hit_points_world, dtype=float)
        distances = np.linalg.norm(points - origin_world[None, :], axis=1)
        valid = (distances >= TOF_MIN_RANGE_M) & (distances <= TOF_MAX_RANGE_M)
        points = points[valid]
        distances = distances[valid]
        if len(points) == 0:
            return

        adjacent_ray_spacing = 2.0 * math.tan(TOF_AXIS_HALF_RAD) / max(1, TOF_SIZE - 1)

        for point, distance in zip(points, distances):
            ray_keys = self._trace_keys(origin_world, point)
            end_key = self._key(point)
            for key in ray_keys:
                if key != end_key:
                    self.cells.pop(key, None)

            norm = np.clip((distance - TOF_MIN_RANGE_M) / max(TOF_MAX_RANGE_M - TOF_MIN_RANGE_M, 1e-6), 0.0, 1.0)
            base_weight = 0.35 + 0.55 * (1.0 - math.sqrt(norm))
            footprint_radius = self.merge_range_scale * max(self.voxel_size * 0.55, 0.45 * distance * adjacent_ray_spacing)
            radius_vox = max(0, int(math.ceil(footprint_radius / max(self.voxel_size, 1e-6))))
            px, py, pz = end_key
            offsets = self._footprint_offsets(radius_vox)
            if offsets.size == 0:
                self._merge_hit(end_key, distance, base_weight, tick_ms, frame_seq)
                continue
            keys_arr = offsets + np.asarray(end_key, dtype=int)[None, :]
            centers = (keys_arr.astype(float) + 0.5) * self.voxel_size
            dists = np.linalg.norm(centers - point[None, :], axis=1)
            valid_mask = dists <= footprint_radius
            if not np.any(valid_mask):
                self._merge_hit(end_key, distance, base_weight, tick_ms, frame_seq)
                continue
            valid_keys = keys_arr[valid_mask]
            valid_dists = dists[valid_mask]
            weights = base_weight * np.exp(-(valid_dists * valid_dists) / max(2.0 * footprint_radius * footprint_radius, 1e-9))
            for key_vals, local_weight in zip(valid_keys, weights):
                key = (int(key_vals[0]), int(key_vals[1]), int(key_vals[2]))
                if key == end_key:
                    local_weight = max(float(local_weight), base_weight)
                self._merge_hit(key, distance, float(local_weight), tick_ms, frame_seq)

    def snapshot(self, tick_ms, max_range, min_prob):
        centers = []
        weights = []
        distances = []
        keys = []
        sizes = []
        for key, cell in self.cells.items():
            weight = self._decayed(cell, tick_ms) if tick_ms is not None else float(cell["weight"])
            if weight < float(min_prob):
                continue
            if int(cell.get("streak", 0)) < MIN_RENDER_STREAK:
                continue
            distance = float(cell["distance"])
            if distance > float(max_range):
                continue
            keys.append(key)
            centers.append(self._center(key))
            weights.append(weight)
            distances.append(distance)
            sizes.append(self.voxel_size)
        if not centers:
            return (
                np.empty((0, 3), dtype=float),
                np.empty((0,), dtype=float),
                np.empty((0,), dtype=float),
                [],
                np.empty((0,), dtype=float),
            )
        return (
            np.asarray(centers, dtype=float),
            np.asarray(weights, dtype=float),
            np.asarray(distances, dtype=float),
            keys,
            np.asarray(sizes, dtype=float),
        )

    def merged_snapshot(self, tick_ms, max_range, min_prob, merge_scale=1.0):
        centers, weights, distances, keys, sizes = self.snapshot(tick_ms, max_range, min_prob)
        merge_scale = float(max(1.0, merge_scale))
        if centers.size == 0 or merge_scale <= 1.0:
            return centers, weights, distances, keys, sizes

        merged = {}
        range_span = max(TOF_MAX_RANGE_M - TOF_MIN_RANGE_M, 1e-6)
        for center, weight, distance, fine_key in zip(centers, weights, distances, keys):
            dist_norm = np.clip((float(distance) - TOF_MIN_RANGE_M) / range_span, 0.0, 1.0)
            local_merge_scale = 1.0 + (merge_scale - 1.0) * math.sqrt(dist_norm)
            local_merge_size = self.voxel_size * local_merge_scale
            coarse_key = np.floor(np.asarray(center, dtype=float) / local_merge_size).astype(int)
            coarse_key_t = tuple(int(v) for v in coarse_key.tolist())
            bucket = merged.get(coarse_key_t)
            if bucket is None:
                merged[coarse_key_t] = {
                    "sum_w": float(weight),
                    "sum_dist": float(distance) * float(weight),
                    "fine_keys": [fine_key],
                    "merge_size_wsum": float(local_merge_size) * float(weight),
                    "merge_weight_sum": float(weight),
                }
            else:
                bucket["sum_w"] += float(weight)
                bucket["sum_dist"] += float(distance) * float(weight)
                bucket["fine_keys"].append(fine_key)
                bucket["merge_size_wsum"] += float(local_merge_size) * float(weight)
                bucket["merge_weight_sum"] += float(weight)

        out_centers = []
        out_weights = []
        out_distances = []
        out_keys = []
        out_sizes = []
        for coarse_key_t, bucket in merged.items():
            sum_w = max(bucket["sum_w"], 1e-9)
            merge_weight_sum = max(bucket["merge_weight_sum"], 1e-9)
            merge_size = bucket["merge_size_wsum"] / merge_weight_sum
            snapped_center = (np.asarray(coarse_key_t, dtype=float) + 0.5) * merge_size
            out_centers.append(snapped_center)
            out_distances.append(bucket["sum_dist"] / sum_w)
            out_weights.append(min(1.0, sum_w))
            out_keys.append(coarse_key_t)
            out_sizes.append(merge_size)

        bridge_range = max(1, int(math.floor(merge_scale)))
        if bridge_range > 1 and out_keys:
            coarse_data = {
                key: {
                    "weight": float(weight),
                    "distance": float(distance),
                }
                for key, weight, distance in zip(out_keys, out_weights, out_distances)
            }
            base_keys = list(coarse_data.keys())
            for key in base_keys:
                kx, ky, kz = key
                src = coarse_data[key]
                for dx in range(-bridge_range, bridge_range + 1):
                    for dy in range(-bridge_range, bridge_range + 1):
                        for dz in range(-bridge_range, bridge_range + 1):
                            if dx == 0 and dy == 0 and dz == 0:
                                continue
                            nbr = (kx + dx, ky + dy, kz + dz)
                            if nbr <= key:
                                continue
                            dst = coarse_data.get(nbr)
                            if dst is None:
                                continue
                            step_count = max(abs(dx), abs(dy), abs(dz))
                            if step_count <= 1:
                                continue
                            w_avg = min(1.0, 0.5 * (src["weight"] + dst["weight"]))
                            d_avg = 0.5 * (src["distance"] + dst["distance"])
                            prev_key = key
                            for step in range(1, step_count):
                                t = step / step_count
                                interp = (
                                    int(round(kx + dx * t)),
                                    int(round(ky + dy * t)),
                                    int(round(kz + dz * t)),
                                )
                                if interp == prev_key or interp == nbr:
                                    continue
                                prev_key = interp
                                existing = coarse_data.get(interp)
                                if existing is None:
                                    coarse_data[interp] = {"weight": w_avg, "distance": d_avg}
                                else:
                                    existing["weight"] = min(1.0, max(existing["weight"], w_avg))
                                    existing["distance"] = 0.5 * (existing["distance"] + d_avg)

            out_keys = list(coarse_data.keys())
            out_weights = [float(coarse_data[key]["weight"]) for key in out_keys]
            out_distances = [float(coarse_data[key]["distance"]) for key in out_keys]
            merge_size = self.voxel_size * merge_scale
            out_centers = [
                (np.asarray(key, dtype=float) + 0.5) * merge_size
                for key in out_keys
            ]
            out_sizes = [merge_size for _ in out_keys]

        return (
            np.asarray(out_centers, dtype=float),
            np.asarray(out_weights, dtype=float),
            np.asarray(out_distances, dtype=float),
            out_keys,
            np.asarray(out_sizes, dtype=float),
        )

    @staticmethod
    def connected_components(keys, connectivity=26, neighbor_range=1):
        if not keys:
            return []
        r = max(1, int(neighbor_range))
        if connectivity == 6:
            offsets = []
            for step in range(1, r + 1):
                offsets.extend(
                    [
                        (step, 0, 0), (-step, 0, 0),
                        (0, step, 0), (0, -step, 0),
                        (0, 0, step), (0, 0, -step),
                    ]
                )
        elif connectivity == 18:
            offsets = []
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    for dz in range(-r, r + 1):
                        if dx == 0 and dy == 0 and dz == 0:
                            continue
                        if max(abs(dx), abs(dy), abs(dz)) <= r and (abs(dx) + abs(dy) + abs(dz)) <= (2 * r):
                            offsets.append((dx, dy, dz))
        else:
            offsets = [
                (dx, dy, dz)
                for dx in range(-r, r + 1)
                for dy in range(-r, r + 1)
                for dz in range(-r, r + 1)
                if not (dx == 0 and dy == 0 and dz == 0)
            ]

        key_set = set(keys)
        visited = set()
        components = []
        for key in keys:
            if key in visited:
                continue
            stack = [key]
            visited.add(key)
            comp = []
            while stack:
                cur = stack.pop()
                comp.append(cur)
                cx, cy, cz = cur
                for dx, dy, dz in offsets:
                    nxt = (cx + dx, cy + dy, cz + dz)
                    if nxt in key_set and nxt not in visited:
                        visited.add(nxt)
                        stack.append(nxt)
            components.append(comp)
        return components


class TranslationEstimator:
    def __init__(self):
        self.pos = np.zeros(3, dtype=float)
        self.vel = np.zeros(3, dtype=float)
        self.acc_lp = np.zeros(3, dtype=float)
        self.last_tick_ms = None

    def reset(self, tick_ms=None):
        self.pos[:] = 0.0
        self.vel[:] = 0.0
        self.acc_lp[:] = 0.0
        self.last_tick_ms = tick_ms

    def update(self, linear_acc_world_g, stationary, tick_ms, accel_deadband_g=TRANSLATION_DEADBAND_G):
        if linear_acc_world_g is None or tick_ms is None:
            return self.pos
        if self.last_tick_ms is None:
            self.last_tick_ms = tick_ms
            return self.pos

        dt = max(0.001, min((tick_ms - self.last_tick_ms) / 1000.0, 0.15))
        self.last_tick_ms = tick_ms
        acc_g = np.asarray(linear_acc_world_g, dtype=float)
        self.acc_lp = TRANSLATION_ACC_LP * self.acc_lp + (1.0 - TRANSLATION_ACC_LP) * acc_g
        acc_eff = self.acc_lp.copy()
        deadband = max(float(accel_deadband_g), TRANSLATION_DEADBAND_G)
        acc_eff[np.abs(acc_eff) < deadband] = 0.0
        acc_mag = float(np.linalg.norm(acc_eff))
        if acc_mag < TRANSLATION_TRIGGER_G:
            acc_eff[:] = 0.0
        acc_mps2 = acc_eff * 9.80665 * TRANSLATION_GAIN

        if stationary:
            self.vel *= 0.08
            self.vel[np.abs(self.vel) < 0.008] = 0.0
        else:
            if acc_mag >= TRANSLATION_TRIGGER_G:
                self.vel += acc_mps2 * dt
            self.vel *= TRANSLATION_VEL_DAMP
            speed = float(np.linalg.norm(self.vel))
            if speed > TRANSLATION_MAX_SPEED_MPS:
                self.vel *= TRANSLATION_MAX_SPEED_MPS / max(speed, 1e-9)

        self.pos += self.vel * dt
        self.pos *= TRANSLATION_POS_DAMP
        self.pos = np.clip(self.pos, -TRANSLATION_MAX_POS_M, TRANSLATION_MAX_POS_M)
        return self.pos


class VoxelViewer(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ToF Voxel Viewer")
        self.resize(1600, 950)

        self.ser = None
        self.serial_error = None
        self.rx_buffer = ""
        self.latest = {}
        self.frame = np.full((TOF_SIZE, TOF_SIZE), np.nan, dtype=float)
        self.pose = PoseTracker()
        self.translation = TranslationEstimator()
        self.voxel_map = SparseVoxelMap()
        self.max_display_distance_m = 4.0
        self.min_render_probability = 1.0
        self.view_mode = "world"
        self.connectivity = 26
        self.connectivity_range = 1
        self.render_merge_scale = 2.0
        self.show_clusters = False
        self.show_device = True
        self.recording_active = False
        self.playback_active = False
        self.recorded_payloads = []
        self.playback_frames = []
        self.playback_start_perf = None
        self.playback_index = 0
        self.playback_loop_duration_ms = 0
        self.last_recording_path = None
        self.last_recording_error = None
        self.last_scene_render_s = 0.0
        self.last_fpv_render_s = 0.0
        self.last_hist_render_s = 0.0
        self.last_status_s = 0.0
        self.last_status = ""
        self.voxel_actor = None
        self.voxel_mesh = None
        self.device_actor = None
        self.grid_actor = None
        self.grid_signature = None
        self.last_snapshot = None
        self.last_scene_render_count = 0
        self.last_fpv_render_count = 0
        self.last_render_tick_ms = None
        self._cube_offsets = None
        self._cube_faces = None
        self._scene_cube = None
        self._cluster_cache_signature = None
        self._cluster_cache_ids = None
        self._last_camera_mode = None
        self.scene_dirty = True
        self.fpv_dirty = True
        self.hist_dirty = True
        self.current_local_points = np.empty((0, 3), dtype=float)
        self.current_local_distances = np.empty((0,), dtype=float)
        self.last_hist_distances_m = np.full(8, 4.0, dtype=float)
        self.last_hist_coverage = np.zeros(8, dtype=float)
        self.frame_seq = 0
        self.calibration_active = False
        self.calibration_start_ms = None
        self.calibration_deadline_ms = None
        self.calibration_status = "idle"
        self.calibration_stats = {}
        self.calibration_samples = []
        self.connect_serial(PORT, BAUD)
        self.load_latest_recording()

        pv.set_plot_theme("document")

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QHBoxLayout(central)

        controls = QtWidgets.QVBoxLayout()
        layout.addLayout(controls, 0)

        btn_row = QtWidgets.QHBoxLayout()
        self.clear_btn = QtWidgets.QPushButton("Clear Map")
        self.clear_btn.clicked.connect(self.clear_map)
        self.calibrate_btn = QtWidgets.QPushButton("Calibrate 5s")
        self.calibrate_btn.clicked.connect(self.start_calibration)
        self.record_btn = QtWidgets.QPushButton("Record")
        self.record_btn.clicked.connect(self.toggle_record)
        self.play_btn = QtWidgets.QPushButton("Play Loop")
        self.play_btn.clicked.connect(self.toggle_play)
        btn_row.addWidget(self.clear_btn)
        btn_row.addWidget(self.calibrate_btn)
        btn_row.addWidget(self.record_btn)
        btn_row.addWidget(self.play_btn)
        controls.addLayout(btn_row)

        self.max_range_spin = self._add_double_spin(controls, "Max Range (m)", self.max_display_distance_m, 0.05, 8.0, 0.05, self.on_max_range)
        self.voxel_size_spin = self._add_double_spin(controls, "Voxel Size (m)", self.voxel_map.voxel_size, 0.01, 0.50, 0.01, self.on_voxel_size)
        self.render_merge_spin = self._add_double_spin(controls, "Render Merge X", self.render_merge_scale, 1.0, 6.0, 0.25, self.on_render_merge)
        self.merge_range_spin = self._add_double_spin(controls, "Voxel Merge X", self.voxel_map.merge_range_scale, 0.10, 4.00, 0.05, self.on_merge_range)
        self.decay_spin = self._add_double_spin(controls, "Point Life (s)", self.voxel_map.decay_tau, 0.5, 30.0, 0.5, self.on_decay_tau)
        self.min_prob_spin = self._add_double_spin(controls, "Min Prob", self.min_render_probability, 0.0, 1.0, 0.01, self.on_min_prob)

        connectivity_row = QtWidgets.QHBoxLayout()
        connectivity_row.addWidget(QtWidgets.QLabel("Connectivity"))
        self.connectivity_combo = QtWidgets.QComboBox()
        self.connectivity_combo.addItems(["6", "18", "26"])
        self.connectivity_combo.setCurrentText("26")
        self.connectivity_combo.currentTextChanged.connect(self.on_connectivity)
        connectivity_row.addWidget(self.connectivity_combo)
        connectivity_row.addWidget(QtWidgets.QLabel("Conn Gap"))
        self.connectivity_range_spin = QtWidgets.QSpinBox()
        self.connectivity_range_spin.setRange(1, 3)
        self.connectivity_range_spin.setValue(self.connectivity_range)
        self.connectivity_range_spin.valueChanged.connect(self.on_connectivity_range)
        connectivity_row.addWidget(self.connectivity_range_spin)
        controls.addLayout(connectivity_row)

        self.cluster_check = QtWidgets.QCheckBox("Color Clusters")
        self.cluster_check.toggled.connect(self.on_show_clusters)
        controls.addWidget(self.cluster_check)

        self.device_check = QtWidgets.QCheckBox("Show Device")
        self.device_check.setChecked(True)
        self.device_check.toggled.connect(self.on_show_device)
        controls.addWidget(self.device_check)

        right_col = QtWidgets.QVBoxLayout()
        layout.addLayout(right_col, 1)

        self.plotter = QtInteractor(central)
        right_col.addWidget(self.plotter.interactor, 4)
        self.plotter.set_background("white")
        self.plotter.add_axes()
        self.plotter.show_grid(color="lightgray")
        self._build_cube_cache()
        self._build_scene_cube()
        self._ensure_device_actor()
        self._apply_camera_preset(force=True)

        fp_group = QtWidgets.QGroupBox("First-Person View")
        fp_layout = QtWidgets.QVBoxLayout(fp_group)
        self.fpv_label = QtWidgets.QLabel()
        self.fpv_label.setMinimumSize(520, 260)
        self.fpv_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.fpv_label.setStyleSheet("background-color: #dcefdc; border: 1px solid #888;")
        fp_layout.addWidget(self.fpv_label)
        right_col.addWidget(fp_group, 2)

        hist_group = QtWidgets.QGroupBox("Nearest Object By Slice")
        hist_layout = QtWidgets.QVBoxLayout(hist_group)
        self.hist_label = QtWidgets.QLabel()
        self.hist_label.setMinimumSize(520, 120)
        self.hist_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.hist_label.setStyleSheet("background-color: white; border: 1px solid #888;")
        hist_layout.addWidget(self.hist_label)
        right_col.addWidget(hist_group, 1)

        status_group = QtWidgets.QGroupBox("Status")
        status_layout = QtWidgets.QVBoxLayout(status_group)
        self.status_box = QtWidgets.QPlainTextEdit()
        self.status_box.setReadOnly(True)
        self.status_box.setMinimumWidth(320)
        status_layout.addWidget(self.status_box)
        right_col.addWidget(status_group, 2)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.on_timer)
        self.timer.start(UI_TIMER_MS)

    def _build_cube_cache(self):
        self._cube_offsets = np.array(
            [
                [-0.5, -0.5, -0.5],
                [0.5, -0.5, -0.5],
                [0.5, 0.5, -0.5],
                [-0.5, 0.5, -0.5],
                [-0.5, -0.5, 0.5],
                [0.5, -0.5, 0.5],
                [0.5, 0.5, 0.5],
                [-0.5, 0.5, 0.5],
            ],
            dtype=float,
        )
        self._cube_faces = [
            (np.array([2, 3, 7, 6], dtype=int), np.array([0.0, 1.0, 0.0], dtype=float), 1.00),
            (np.array([0, 1, 5, 4], dtype=int), np.array([0.0, -1.0, 0.0], dtype=float), 0.55),
            (np.array([1, 2, 6, 5], dtype=int), np.array([1.0, 0.0, 0.0], dtype=float), 0.78),
            (np.array([0, 3, 7, 4], dtype=int), np.array([-1.0, 0.0, 0.0], dtype=float), 0.70),
            (np.array([4, 5, 6, 7], dtype=int), np.array([0.0, 0.0, 1.0], dtype=float), 0.90),
            (np.array([0, 1, 2, 3], dtype=int), np.array([0.0, 0.0, -1.0], dtype=float), 0.60),
        ]

    def _build_scene_cube(self):
        self._scene_cube = pv.Cube(
            center=(0.0, 0.0, 0.0),
            x_length=1.0,
            y_length=1.0,
            z_length=1.0,
        )

    def _add_double_spin(self, layout, label, value, vmin, vmax, step, slot):
        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel(label))
        spin = QtWidgets.QDoubleSpinBox()
        spin.setDecimals(3 if step < 0.1 else 2)
        spin.setRange(vmin, vmax)
        spin.setSingleStep(step)
        spin.setValue(value)
        spin.valueChanged.connect(slot)
        row.addWidget(spin)
        layout.addLayout(row)
        return spin

    def connect_serial(self, port, baud):
        try:
            self.ser = serial.Serial(port, baud, timeout=0, write_timeout=0)
            self.serial_error = None
        except serial.SerialException as exc:
            self.ser = None
            self.serial_error = f"Open failed: {exc}"

    def on_max_range(self, value):
        self.max_display_distance_m = float(value)
        self.mark_render_dirty()

    def on_voxel_size(self, value):
        self.voxel_map.set_voxel_size(float(value))
        self._build_cube_cache()
        self._build_scene_cube()
        self.grid_signature = None
        self._cluster_cache_signature = None
        self._cluster_cache_ids = None
        if self.voxel_actor is not None:
            try:
                self.plotter.remove_actor(self.voxel_actor, render=False)
            except Exception:
                pass
            self.voxel_actor = None
            self.voxel_mesh = None
        self.mark_render_dirty()

    def on_merge_range(self, value):
        self.voxel_map.merge_range_scale = float(value)
        self.mark_render_dirty()

    def on_render_merge(self, value):
        self.render_merge_scale = float(value)
        self._cluster_cache_signature = None
        self._cluster_cache_ids = None
        self.mark_render_dirty()

    def on_decay_tau(self, value):
        self.voxel_map.decay_tau = float(value)
        self.mark_render_dirty()

    def on_min_prob(self, value):
        self.min_render_probability = float(value)
        self.mark_render_dirty()

    def on_connectivity(self, text):
        self.connectivity = int(text)
        self._cluster_cache_signature = None
        self._cluster_cache_ids = None
        self.mark_render_dirty()

    def on_connectivity_range(self, value):
        self.connectivity_range = int(value)
        self._cluster_cache_signature = None
        self._cluster_cache_ids = None
        self.mark_render_dirty()

    def on_show_clusters(self, checked):
        self.show_clusters = bool(checked)
        self._cluster_cache_signature = None
        self._cluster_cache_ids = None
        self.mark_render_dirty()

    def on_show_device(self, checked):
        self.show_device = bool(checked)
        self._ensure_device_actor()
        self.mark_render_dirty()

    def _apply_camera_preset(self, force=False):
        if (not force) and self._last_camera_mode == self.view_mode:
            return
        if self.view_mode == "device":
            p = self.translation.pos
            self.plotter.camera_position = [
                (float(p[0]), float(p[1] - 3.0), float(p[2] + 1.5)),
                (float(p[0]), float(p[1] + 0.8), float(p[2])),
                (0.0, 0.0, 1.0),
            ]
        else:
            self.plotter.camera_position = [(2.8, -2.8, 2.2), (0.0, 0.6, 0.0), (0.0, 0.0, 1.0)]
        self._last_camera_mode = self.view_mode

    def mark_render_dirty(self, scene=True, fpv=True):
        if scene:
            self.scene_dirty = True
        if fpv:
            self.fpv_dirty = True
            self.hist_dirty = True

    def clear_map(self):
        self.voxel_map.clear()
        self.translation.reset(self.latest.get("tick_ms") if self.latest else None)
        self.last_snapshot = None
        self.grid_signature = None
        self._cluster_cache_signature = None
        self._cluster_cache_ids = None
        self.mark_render_dirty()

    def start_calibration(self):
        tick_ms = self.latest.get("tick_ms") if self.latest else 0
        self.calibration_active = True
        self.calibration_start_ms = tick_ms
        self.calibration_deadline_ms = tick_ms + CALIBRATION_DURATION_MS
        self.calibration_status = "collecting"
        self.calibration_stats = {}
        self.calibration_samples = []

    def toggle_record(self):
        if self.recording_active:
            self.recording_active = False
            self.auto_save_recording()
        else:
            self.recorded_payloads = []
            self.playback_frames = []
            self.last_recording_path = None
            self.last_recording_error = None
            self.recording_active = True
            self.playback_active = False
        self.update_button_labels()

    def toggle_play(self):
        if self.playback_active:
            self.playback_active = False
            self.playback_start_perf = None
            self.playback_index = 0
            self.update_button_labels()
            return
        if not self.recorded_payloads:
            self.load_latest_recording()
        if not self.recorded_payloads:
            self.last_recording_error = "no saved clip to play"
            self.update_button_labels()
            return
        if not self.playback_frames:
            self.rebuild_playback_frames()
        if not self.playback_frames:
            self.last_recording_error = "recording has no playable frames"
            self.update_button_labels()
            return
        self.playback_active = True
        self.recording_active = False
        self.playback_start_perf = time.perf_counter()
        self.playback_index = 0
        first_tick = int(self.playback_frames[0].get("tick_ms", 0))
        last_tick = int(self.playback_frames[-1].get("tick_ms", first_tick))
        self.playback_loop_duration_ms = max(1, last_tick - first_tick)
        self.voxel_map.clear()
        self.last_snapshot = None
        self._cluster_cache_signature = None
        self._cluster_cache_ids = None
        self.mark_render_dirty()
        self.update_button_labels()

    def update_button_labels(self):
        self.record_btn.setText("Stop Rec" if self.recording_active else "Record")
        self.play_btn.setText("Stop Play" if self.playback_active else "Play Loop")

    def auto_save_recording(self):
        if not self.recorded_payloads:
            return
        try:
            RECORDINGS_DIR.mkdir(parents=True, exist_ok=True)
            ts = time.strftime("%Y%m%d_%H%M%S")
            path = RECORDINGS_DIR / f"tof_clip_{ts}.jsonl"
            with path.open("w", encoding="utf-8") as f:
                for payload in self.recorded_payloads:
                    f.write(json.dumps(payload, separators=(",", ":")))
                    f.write("\n")
            self.last_recording_path = str(path)
            self.last_recording_error = None
        except OSError as exc:
            self.last_recording_error = f"save failed: {exc}"

    def load_recording_file(self, path):
        payloads = []
        with Path(path).open("r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line or not line.startswith("{"):
                    continue
                try:
                    payload = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if isinstance(payload, dict):
                    payloads.append(payload)
        if not payloads:
            raise ValueError("recording file contains no valid frames")
        self.recorded_payloads = payloads
        self.rebuild_playback_frames()
        self.last_recording_path = str(path)
        self.last_recording_error = None

    def load_latest_recording(self):
        try:
            if not RECORDINGS_DIR.exists():
                return
            files = sorted(RECORDINGS_DIR.glob("tof_clip_*.jsonl"), key=lambda p: p.stat().st_mtime, reverse=True)
            if not files:
                return
            self.load_recording_file(files[0])
        except (OSError, ValueError) as exc:
            self.last_recording_error = f"load failed: {exc}"

    def expand_payload(self, payload):
        if not isinstance(payload, dict):
            return []
        if "frames" not in payload:
            return [payload]
        present = payload.get("present")
        env = payload.get("env")
        expanded = []
        for sub in payload.get("frames", []):
            if not isinstance(sub, dict):
                continue
            merged = copy.deepcopy(sub)
            if present is not None and "present" not in merged:
                merged["present"] = copy.deepcopy(present)
            if env is not None and "env" not in merged:
                merged["env"] = copy.deepcopy(env)
            expanded.append(merged)
        return expanded

    def rebuild_playback_frames(self):
        frames = []
        for payload in self.recorded_payloads:
            frames.extend(self.expand_payload(payload))
        frames.sort(key=lambda item: int(item.get("tick_ms", 0)))
        self.playback_frames = frames

    def read_frames(self):
        if self.playback_active or self.ser is None:
            return
        try:
            pending = self.ser.in_waiting
            if pending > SERIAL_BACKLOG_DROP_BYTES:
                self.ser.reset_input_buffer()
                self.rx_buffer = ""
                return
            if pending <= 0:
                return
            raw = self.ser.read(min(pending, SERIAL_MAX_BYTES_PER_TICK))
        except serial.SerialException as exc:
            self.serial_error = f"Read failed: {exc}"
            return
        if not raw:
            return
        self.rx_buffer += raw.decode("utf-8", errors="ignore")
        lines = self.rx_buffer.splitlines()
        if self.rx_buffer and not self.rx_buffer.endswith(("\n", "\r")):
            self.rx_buffer = lines.pop() if lines else self.rx_buffer
        else:
            self.rx_buffer = ""
        for line in lines[-MAX_READS_PER_TICK:]:
            line = line.strip()
            if not line.startswith("{"):
                continue
            try:
                payload = json.loads(line)
            except json.JSONDecodeError:
                continue
            if ("frame" not in payload) and ("frames" not in payload):
                continue
            if self.recording_active:
                self.recorded_payloads.append(copy.deepcopy(payload))
            self.process_payload(payload)

    def playback_step(self):
        if not self.playback_active or not self.playback_frames:
            return
        if len(self.playback_frames) == 1:
            self.process_payload(copy.deepcopy(self.playback_frames[0]))
            return
        if self.playback_start_perf is None:
            self.playback_start_perf = time.perf_counter()
        first_tick = int(self.playback_frames[0].get("tick_ms", 0))
        elapsed_ms = int((time.perf_counter() - self.playback_start_perf) * 1000.0)
        loop_elapsed_ms = elapsed_ms % max(1, self.playback_loop_duration_ms)
        target_tick = first_tick + loop_elapsed_ms
        while (
            self.playback_index + 1 < len(self.playback_frames)
            and int(self.playback_frames[self.playback_index + 1].get("tick_ms", first_tick)) <= target_tick
        ):
            self.playback_index += 1
        if target_tick < int(self.playback_frames[self.playback_index].get("tick_ms", first_tick)):
            self.playback_index = 0
        self.process_payload(copy.deepcopy(self.playback_frames[self.playback_index]))

    def process_payload(self, payload):
        for subpayload in self.expand_payload(payload):
            self.latest = subpayload
            self.update_state_from_payload(subpayload)

    def update_state_from_payload(self, payload):
        self.frame_seq += 1
        tof = payload.get("tof", {})
        dist = tof.get("dist_mm", [])
        if isinstance(dist, list) and len(dist) == TOF_SIZE * TOF_SIZE:
            arr = np.array(
                [np.nan if (v is None or float(v) < 0) else float(v) for v in dist],
                dtype=float,
            )
            self.frame = arr.reshape((TOF_SIZE, TOF_SIZE))

        motion = payload.get("motion", {})
        acc = transform_axes(motion.get("lsm6dsv16x_acc_mg"))
        gyro = transform_axes(motion.get("lsm6dsv16x_gyro_mdps"))
        mag = transform_axes(motion.get("lis2mdl_mag_mgauss"))
        tick_ms = payload.get("tick_ms")
        self.update_calibration_samples(tick_ms, acc, gyro, mag)
        self.pose.update_orientation(acc, gyro, mag, tick_ms)
        self.translation.update(
            self.pose.linear_acc_world,
            self.pose.stationary,
            tick_ms,
            accel_deadband_g=self.pose.accel_deadband_g,
        )
        self.update_voxel_map(tick_ms)

    def update_calibration_samples(self, tick_ms, acc_xyz, gyro_xyz, mag_xyz):
        if not self.calibration_active or tick_ms is None:
            return

        still = False
        if acc_xyz is not None and gyro_xyz is not None:
            acc_g = np.array(acc_xyz, dtype=float) / 1000.0
            gyro_dps = np.array(gyro_xyz, dtype=float) / 1000.0
            acc_norm = float(np.linalg.norm(acc_g))
            pose_match = False
            if acc_norm > 1e-6:
                acc_unit = acc_g / acc_norm
                pose_match = float(np.dot(acc_unit, CALIBRATION_EXPECTED_ACC_G)) >= CALIBRATION_FRONT_UP_DOT_MIN
            still = (
                pose_match
                and abs(acc_norm - 1.0) < CALIBRATION_ACC_STILL_G
                and float(np.linalg.norm(gyro_dps)) < CALIBRATION_GYRO_STILL_DPS
            )

        self.calibration_samples.append(
            {
                "tick_ms": tick_ms,
                "acc": None if acc_xyz is None else np.array(acc_xyz, dtype=float) / 1000.0,
                "gyro": None if gyro_xyz is None else np.array(gyro_xyz, dtype=float) / 1000.0,
                "mag": None if mag_xyz is None else np.array(mag_xyz, dtype=float),
                "still": still,
            }
        )

        elapsed = tick_ms - self.calibration_start_ms
        still_count = sum(1 for sample in self.calibration_samples if sample["still"])
        total_count = len(self.calibration_samples)
        self.calibration_stats = {
            "elapsed_ms": elapsed,
            "progress": float(np.clip(elapsed / CALIBRATION_DURATION_MS, 0.0, 1.0)),
            "still_count": still_count,
            "total_count": total_count,
        }

        if tick_ms >= self.calibration_deadline_ms:
            self.finish_calibration(tick_ms)

    def finish_calibration(self, tick_ms):
        self.calibration_active = False
        total_count = len(self.calibration_samples)
        still_samples = [sample for sample in self.calibration_samples if sample["still"]]
        still_ratio = (len(still_samples) / total_count) if total_count else 0.0
        if total_count < 10 or still_ratio < CALIBRATION_MIN_STILL_RATIO:
            self.calibration_status = "failed: keep device still for 5s"
            self.calibration_stats.update({"still_ratio": still_ratio})
            return

        acc_stack = [sample["acc"] for sample in still_samples if sample["acc"] is not None]
        gyro_stack = [sample["gyro"] for sample in still_samples if sample["gyro"] is not None]
        mag_stack = [sample["mag"] for sample in still_samples if sample["mag"] is not None]
        acc_mean, acc_std, acc_used = robust_mean_and_std(acc_stack)
        gyro_mean, gyro_std, gyro_used = robust_mean_and_std(gyro_stack)
        mag_mean, mag_std, mag_used = robust_mean_and_std(mag_stack)

        if acc_mean is None or gyro_mean is None:
            self.calibration_status = "failed: insufficient IMU samples"
            return

        acc_mean_norm = float(np.linalg.norm(acc_mean))
        pose_match = False
        if acc_mean_norm > 1e-6:
            pose_match = float(np.dot(acc_mean / acc_mean_norm, CALIBRATION_EXPECTED_ACC_G)) >= CALIBRATION_FRONT_UP_DOT_MIN
        if not pose_match:
            self.calibration_status = "failed: place device flat with front face up"
            self.calibration_stats.update({"still_ratio": still_ratio})
            return

        accel_sensor_bias = acc_mean - CALIBRATION_EXPECTED_ACC_G
        self.pose.apply_noise_calibration(
            gyro_xyz=(gyro_mean * 1000.0).tolist(),
            tick_ms=tick_ms,
            accel_sensor_bias=accel_sensor_bias,
            accel_noise_std=acc_std,
            gyro_noise_std=gyro_std,
            mag_noise_std=mag_std,
        )
        self.voxel_map.clear()
        self.translation.reset(tick_ms)
        self.last_snapshot = None
        self.grid_signature = None
        self._cluster_cache_signature = None
        self._cluster_cache_ids = None
        self.mark_render_dirty()
        self.calibration_status = "complete: noise only"
        self.calibration_stats = {
            "elapsed_ms": CALIBRATION_DURATION_MS,
            "progress": 1.0,
            "still_count": len(still_samples),
            "total_count": total_count,
            "still_ratio": still_ratio,
            "acc_used": acc_used,
            "gyro_used": gyro_used,
            "mag_used": mag_used,
        }

    def update_voxel_map(self, tick_ms):
        local_points = project_tof_to_local_points(self.frame)
        self.current_local_points = local_points.copy() if local_points.size else np.empty((0, 3), dtype=float)
        self.current_local_distances = np.linalg.norm(local_points, axis=1) if local_points.size else np.empty((0,), dtype=float)
        if local_points.size == 0:
            self.mark_render_dirty(scene=False, fpv=True)
            return
        rot_current = rotation_matrix(self.pose.roll, self.pose.pitch, self.pose.yaw)
        points_from_sensor = local_points + DEVICE_SENSOR_FACE_LOCAL
        world_points = points_from_sensor @ rot_current.T + self.translation.pos
        origin_world = DEVICE_SENSOR_FACE_LOCAL @ rot_current.T + self.translation.pos
        self.voxel_map.add_hits(origin_world, world_points, tick_ms, self.frame_seq)
        self.mark_render_dirty()

    def _ensure_device_actor(self):
        if self.device_actor is not None:
            try:
                self.plotter.remove_actor(self.device_actor, render=False)
            except Exception:
                pass
            self.device_actor = None
        if not self.show_device:
            return
        box = pv.Box(
            bounds=(
                -DEVICE_HALF_WIDTH_M,
                DEVICE_HALF_WIDTH_M,
                -DEVICE_HALF_DEPTH_M,
                DEVICE_HALF_DEPTH_M,
                -DEVICE_HALF_HEIGHT_M,
                DEVICE_HALF_HEIGHT_M,
            )
        )
        box = box.translate(self.translation.pos, inplace=False)
        self.device_actor = self.plotter.add_mesh(box, color="#d8a23b", opacity=0.85, smooth_shading=False, render=False)

    def _update_voxel_grid_actor(self, centers):
        voxel = float(self.voxel_map.voxel_size)
        pad = max(voxel * 2.0, 0.08)
        if centers is not None and len(centers):
            mins = np.min(centers, axis=0) - pad
            maxs = np.max(centers, axis=0) + pad
        else:
            p = np.asarray(self.translation.pos, dtype=float)
            mins = p + np.array([-0.5, -0.5, -0.2], dtype=float)
            maxs = p + np.array([0.5, 1.5, 1.2], dtype=float)

        bound_quant = max(GRID_BOUND_QUANT_M, voxel * 2.0)
        mins = np.floor(mins / bound_quant) * bound_quant
        maxs = np.ceil(maxs / bound_quant) * bound_quant

        max_lines_per_axis = 48
        counts = np.maximum(2, np.ceil((maxs - mins) / voxel).astype(int) + 1)
        if np.any(counts > max_lines_per_axis):
            step_mul = int(np.ceil(np.max(counts / max_lines_per_axis)))
            voxel *= step_mul
            mins = np.floor(mins / voxel) * voxel
            maxs = np.ceil(maxs / voxel) * voxel

        xs = np.arange(mins[0], maxs[0] + voxel * 0.5, voxel, dtype=float)
        ys = np.arange(mins[1], maxs[1] + voxel * 0.5, voxel, dtype=float)
        zs = np.arange(mins[2], maxs[2] + voxel * 0.5, voxel, dtype=float)
        if len(xs) < 2 or len(ys) < 2 or len(zs) < 2:
            return
        sig = (
            round(float(voxel), 6),
            round(float(xs[0]), 6), round(float(xs[-1]), 6), len(xs),
            round(float(ys[0]), 6), round(float(ys[-1]), 6), len(ys),
            round(float(zs[0]), 6), round(float(zs[-1]), 6), len(zs),
        )
        if sig == self.grid_signature:
            return
        self.grid_signature = sig
        if self.grid_actor is not None:
            try:
                self.plotter.remove_actor(self.grid_actor, render=False)
            except Exception:
                pass
            self.grid_actor = None

        grid = pv.RectilinearGrid(xs, ys, zs)
        edges = grid.extract_all_edges()
        self.grid_actor = self.plotter.add_mesh(edges, color="#b8b8b8", line_width=1.0, opacity=0.35, render=False)

    def _limit_visible_voxels(self, centers, weights, distances, keys, sizes, max_count):
        if centers.size == 0 or len(centers) <= max_count:
            return centers, weights, distances, keys, sizes
        score = weights / np.maximum(0.15, distances)
        keep_idx = np.argpartition(score, -max_count)[-max_count:]
        keep_idx = keep_idx[np.argsort(distances[keep_idx])[::-1]]
        limited_keys = [keys[int(i)] for i in keep_idx]
        return centers[keep_idx], weights[keep_idx], distances[keep_idx], limited_keys, sizes[keep_idx]

    def render_scene(self):
        tick_ms = self.latest.get("tick_ms") if self.latest else None
        centers, weights, distances, keys, sizes = self.voxel_map.merged_snapshot(
            tick_ms,
            self.max_display_distance_m,
            self.min_render_probability,
            merge_scale=self.render_merge_scale,
        )
        scene_centers, scene_weights, scene_distances, scene_keys, scene_sizes = self._limit_visible_voxels(
            centers, weights, distances, keys, sizes, SCENE_MAX_RENDER_VOXELS
        )
        self.last_snapshot = (centers, weights, distances, keys, sizes)
        self.last_scene_render_count = len(scene_centers)

        if scene_centers.size:
            pdata = pv.PolyData(scene_centers)
            pdata["size"] = scene_sizes
            if self.show_clusters:
                cluster_signature = (self.connectivity, self.connectivity_range, tuple(scene_keys))
                if cluster_signature != self._cluster_cache_signature:
                    comps = self.voxel_map.connected_components(
                        scene_keys,
                        connectivity=self.connectivity,
                        neighbor_range=self.connectivity_range,
                    )
                    cluster_id = np.zeros(len(scene_keys), dtype=np.int32)
                    key_to_index = {key: i for i, key in enumerate(scene_keys)}
                    for idx, comp in enumerate(comps):
                        for key in comp:
                            i = key_to_index.get(key)
                            if i is not None:
                                cluster_id[i] = idx
                    self._cluster_cache_signature = cluster_signature
                    self._cluster_cache_ids = cluster_id
                else:
                    cluster_id = self._cluster_cache_ids
                pdata["cluster_id"] = cluster_id
                scalars_name = "cluster_id"
                cmap = "tab20"
            else:
                self._cluster_cache_signature = None
                self._cluster_cache_ids = None
                pdata["distance"] = scene_distances
                scalars_name = "distance"
                cmap = "gray_r"
            glyphs = pdata.glyph(scale="size", orient=False, geom=self._scene_cube)
            kwargs = {"show_scalar_bar": False, "render": False}
            if scalars_name == "distance":
                kwargs.update({"scalars": scalars_name, "cmap": cmap, "clim": [TOF_MIN_RANGE_M, self.max_display_distance_m]})
            else:
                kwargs.update({"scalars": scalars_name, "cmap": cmap})
            if self.voxel_actor is None or self.voxel_mesh is None:
                self.voxel_mesh = glyphs.copy(deep=True)
                self.voxel_actor = self.plotter.add_mesh(self.voxel_mesh, **kwargs)
            else:
                self.voxel_mesh.shallow_copy(glyphs)
                self.voxel_actor.mapper.SetInputData(self.voxel_mesh)
                self.voxel_actor.SetVisibility(True)
        elif self.voxel_actor is not None:
            self.voxel_actor.SetVisibility(False)

        self._update_voxel_grid_actor(scene_centers if scene_centers.size else None)
        self._ensure_device_actor()
        self._apply_camera_preset(force=False)
        self.plotter.render()
        self.scene_dirty = False

    def render_fpv(self):
        width = max(240, self.fpv_label.width())
        height = max(160, self.fpv_label.height())
        qimg = QtGui.QImage(width, height, QtGui.QImage.Format.Format_RGB32)
        qimg.fill(QtGui.QColor(220, 239, 220))
        painter = QtGui.QPainter(qimg)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing, True)
        mask_img = QtGui.QImage(width, height, QtGui.QImage.Format.Format_Grayscale8)
        mask_img.fill(0)
        mask_painter = QtGui.QPainter(mask_img)
        mask_painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing, False)
        mask_painter.setPen(QtCore.Qt.PenStyle.NoPen)
        mask_painter.setBrush(QtGui.QBrush(QtGui.QColor(255, 255, 255)))
        x_extent = 1.1
        y_extent = 0.9
        fpv_scale = min(width / (2.0 * x_extent), height / (2.0 * y_extent))
        fpv_cx = width * 0.5
        fpv_cy = height * 0.5

        def project_xy(proj_x, proj_y):
            px = fpv_cx + np.asarray(proj_x, dtype=float) * fpv_scale
            py = fpv_cy - np.asarray(proj_y, dtype=float) * fpv_scale
            return px, py

        tick_ms = self.latest.get("tick_ms") if self.latest else None
        if self.last_snapshot is not None and tick_ms == self.last_render_tick_ms:
            centers, weights, distances, keys, sizes = self.last_snapshot
        else:
            centers, weights, distances, keys, sizes = self.voxel_map.merged_snapshot(
                tick_ms,
                self.max_display_distance_m,
                self.min_render_probability,
                merge_scale=self.render_merge_scale,
            )

        rot_current = rotation_matrix(self.pose.roll, self.pose.pitch, self.pose.yaw)
        if centers.size:
            if self.view_mode == "world":
                points_local = centers @ rot_current
            else:
                points_local = centers - self.translation.pos
            forward = points_local[:, 1]
            valid = forward > 0.05
            points_local = points_local[valid]
            distances = distances[valid]
            weights = weights[valid]
            sizes = sizes[valid]
            keys = [key for key, keep in zip(keys, valid) if keep]
        else:
            points_local = np.empty((0, 3), dtype=float)
            distances = np.empty((0,), dtype=float)
            weights = np.empty((0,), dtype=float)
            sizes = np.empty((0,), dtype=float)
            keys = []

        if points_local.size:
            points_local, weights, distances, keys, sizes = self._limit_visible_voxels(
                points_local, weights, distances, keys, sizes, FPV_MAX_RENDER_VOXELS
            )

        if points_local.size:
            proj_x_center = points_local[:, 0] / np.maximum(points_local[:, 1], 1e-6)
            proj_y_center = points_local[:, 2] / np.maximum(points_local[:, 1], 1e-6)
            center_valid = (
                (proj_x_center >= -x_extent)
                & (proj_x_center <= x_extent)
                & (proj_y_center >= -y_extent)
                & (proj_y_center <= y_extent)
            )
            points_local = points_local[center_valid]
            distances = distances[center_valid]
            weights = weights[center_valid]
            sizes = sizes[center_valid]
            keys = [key for key, keep in zip(keys, center_valid) if keep]

        hist_strip_distances = np.full(8, 4.0, dtype=float)
        if points_local.size and keys:
            hist_edges = np.linspace(-1.0, 1.0, 9)
            components = self.voxel_map.connected_components(
                keys,
                connectivity=self.connectivity,
                neighbor_range=self.connectivity_range,
            )
            key_to_idx = {key: i for i, key in enumerate(keys)}
            for comp in components:
                comp_indices = [key_to_idx[key] for key in comp if key in key_to_idx]
                if len(comp_indices) < 8:
                    continue
                comp_indices = np.asarray(comp_indices, dtype=int)
                comp_forward = np.clip(points_local[comp_indices, 1], 0.15, 4.0)
                comp_proj_x = np.clip(points_local[comp_indices, 0] / comp_forward, -1.2, 1.2)
                comp_bins = np.clip(np.digitize(comp_proj_x, hist_edges) - 1, 0, 7)
                comp_min_dist = float(np.min(distances[comp_indices]))
                touched_bins = np.unique(comp_bins)
                for bin_idx in touched_bins:
                    hist_strip_distances[int(bin_idx)] = min(hist_strip_distances[int(bin_idx)], comp_min_dist)
            hist_strip_distances = np.where(np.isfinite(hist_strip_distances), hist_strip_distances, 4.0)
        self.last_hist_distances_m = hist_strip_distances
        self.last_hist_coverage = np.where(hist_strip_distances < 4.0, 1.0, 0.0)

        if points_local.size:
            px_center, py_center = project_xy(proj_x_center[center_valid], proj_y_center[center_valid])
            tile_x = np.clip((px_center / FPV_TILE_PX).astype(int), 0, max(0, (width - 1) // FPV_TILE_PX))
            tile_y = np.clip((py_center / FPV_TILE_PX).astype(int), 0, max(0, (height - 1) // FPV_TILE_PX))
            tile_best = {}
            for idx, (tx, ty, depth, dist, wt) in enumerate(zip(tile_x, tile_y, points_local[:, 1], distances, weights)):
                key = (int(tx), int(ty))
                score = (float(depth), float(wt) / max(float(dist), 1e-6))
                prev = tile_best.get(key)
                if prev is None or score[0] < prev[0] or (abs(score[0] - prev[0]) < 1e-6 and score[1] > prev[1]):
                    tile_best[key] = (score[0], score[1], idx)
            keep_idx = np.array(sorted({item[2] for item in tile_best.values()}), dtype=int)
            points_local = points_local[keep_idx]
            distances = distances[keep_idx]
            weights = weights[keep_idx]
            sizes = sizes[keep_idx]
        self.last_fpv_render_count = len(points_local)

        if points_local.size:
            draw_faces = []
            strip_nearest = np.full(8, np.inf, dtype=float)
            order = np.argsort(points_local[:, 1])[::-1]
            for idx in order:
                center = points_local[idx]
                if abs(center[0]) > center[1] * 1.25 or abs(center[2]) > center[1]:
                    continue
                corners = self._cube_offsets * float(sizes[idx]) + center[None, :]
                norm = np.clip((distances[idx] - TOF_MIN_RANGE_M) / max(self.max_display_distance_m - TOF_MIN_RANGE_M, 1e-6), 0.0, 1.0)
                base_level = float(np.clip(255.0 * (1.0 - math.sqrt(norm)), 0.0, 255.0))
                best_face = None
                best_score = -1e9
                for face_idx, normal, shade in self._cube_faces:
                    face_pts = corners[face_idx]
                    face_center = np.mean(face_pts, axis=0)
                    facing = float(np.dot(normal, -face_center))
                    if facing <= 0.0 or np.any(face_pts[:, 1] <= 0.03):
                        continue
                    if facing > best_score:
                        best_score = facing
                        best_face = (face_idx, shade, face_pts)
                if best_face is None:
                    continue
                face_idx, shade, face_pts = best_face
                proj_x = np.clip(face_pts[:, 0] / face_pts[:, 1], -x_extent, x_extent)
                proj_y = np.clip(face_pts[:, 2] / face_pts[:, 1], -y_extent, y_extent)
                px, py = project_xy(proj_x, proj_y)
                polygon = [QtCore.QPointF(float(x), float(y)) for x, y in zip(px, py)]
                draw_faces.append(
                    (
                        float(np.mean(face_pts[:, 1])),
                        polygon,
                        int(np.clip(base_level * shade, 0.0, 255.0)),
                    )
                )

            draw_faces.sort(key=lambda item: item[0], reverse=True)
            edge_color = QtGui.QColor(255, 255, 255) if self.show_clusters else QtGui.QColor(40, 40, 40)
            for _depth, polygon, level in draw_faces:
                qpoly = QtGui.QPolygonF(polygon)
                painter.setPen(QtGui.QPen(edge_color, 1.0))
                painter.setBrush(QtGui.QBrush(QtGui.QColor(level, level, level)))
                painter.drawPolygon(qpoly)
                mask_painter.drawPolygon(qpoly)
                bounds = qpoly.boundingRect()
                x0 = max(0.0, bounds.left())
                x1 = min(float(width - 1), bounds.right())
                if x1 >= x0:
                    s0 = max(0, min(7, int((x0 / max(1.0, float(width))) * 8.0)))
                    s1 = max(0, min(7, int((x1 / max(1.0, float(width))) * 8.0)))
                    for s in range(s0, s1 + 1):
                        strip_nearest[s] = min(strip_nearest[s], float(_depth))
        else:
            strip_nearest = np.full(8, np.inf, dtype=float)

        cx = width // 2
        cy = height // 2
        painter.setPen(QtGui.QPen(QtGui.QColor(90, 90, 90), 1.0))
        painter.drawLine(cx, 0, cx, height - 1)
        painter.drawLine(0, cy, width - 1, cy)
        painter.end()
        mask_painter.end()

        pixmap = QtGui.QPixmap.fromImage(qimg)
        self.fpv_label.setPixmap(pixmap)
        self.fpv_dirty = False
        self.hist_dirty = True

    def render_histogram(self):
        width = max(240, self.hist_label.width())
        height = max(100, self.hist_label.height())
        qimg = QtGui.QImage(width, height, QtGui.QImage.Format.Format_RGB32)
        qimg.fill(QtGui.QColor(255, 255, 255))
        painter = QtGui.QPainter(qimg)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing, True)

        top_pad = 10
        bottom_pad = 22
        left_pad = 18
        right_pad = 8
        plot_w = max(1, width - left_pad - right_pad)
        plot_h = max(1, height - top_pad - bottom_pad)
        max_dist = 4.0

        painter.setPen(QtGui.QPen(QtGui.QColor(210, 210, 210), 1))
        for frac in (0.25, 0.5, 0.75, 1.0):
            y = top_pad + int(round(plot_h * frac))
            painter.drawLine(left_pad, y, left_pad + plot_w, y)

        vals = self.last_hist_distances_m
        xs = np.linspace(left_pad, left_pad + plot_w, 9)
        bar_w = max(4, int(plot_w / 8) - 6)
        for idx, val in enumerate(vals):
            x = int(round(0.5 * (xs[idx] + xs[idx + 1]) - bar_w * 0.5))
            v = float(np.clip(val, TOF_MIN_RANGE_M, max_dist))
            frac = v / max_dist
            bar_h = int(round(plot_h * frac))
            y = top_pad
            level = int(np.clip(255.0 * (1.0 - math.sqrt(np.clip((v - TOF_MIN_RANGE_M) / max(max_dist - TOF_MIN_RANGE_M, 1e-6), 0.0, 1.0))), 0.0, 255.0))
            painter.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, 180), 1))
            painter.setBrush(QtGui.QBrush(QtGui.QColor(level, level, level, 220)))
            painter.drawRect(x, y, bar_w, bar_h)
            painter.setPen(QtGui.QPen(QtGui.QColor(30, 30, 30), 1))
            painter.drawText(x, height - 6, str(idx + 1))
            painter.drawText(x + 2, min(height - 24, top_pad + bar_h + 12), f"{int(round(v * 100.0))}")

        painter.end()
        self.hist_label.setPixmap(QtGui.QPixmap.fromImage(qimg))
        self.hist_dirty = False

    def update_status(self):
        latest = self.latest or {}
        tof = latest.get("tof", {})
        lines = [
            "voxel viewer",
            f"frame        : {latest.get('frame')}",
            f"tick_ms      : {latest.get('tick_ms')}",
            f"source       : {'playback' if self.playback_active else 'live'}",
            f"recording    : {'on' if self.recording_active else 'off'}",
            f"serial       : {self.serial_error or 'ok'}",
            "",
            f"voxels       : {len(self.voxel_map.cells)}",
            f"scene/fpv    : {self.last_scene_render_count} / {self.last_fpv_render_count}",
            f"voxel size   : {self.voxel_map.voxel_size:.3f} m",
            f"render x     : {self.render_merge_scale:.2f}",
            f"merge x      : {self.voxel_map.merge_range_scale:.2f}",
            f"point life   : {self.voxel_map.decay_tau:.1f} s",
            f"min prob     : {self.min_render_probability:.2f}",
            f"max range    : {self.max_display_distance_m:.2f} m",
            f"connectivity : {self.connectivity}",
            f"conn gap     : {self.connectivity_range}",
            f"clusters     : {'on' if self.show_clusters else 'off'}",
            f"device       : {'on' if self.show_device else 'off'}",
            f"stationary   : {'yes' if self.pose.stationary else 'no'}",
            f"pos xyz      : {self.translation.pos[0]: .2f} {self.translation.pos[1]: .2f} {self.translation.pos[2]: .2f}",
            f"vel xyz      : {self.translation.vel[0]: .2f} {self.translation.vel[1]: .2f} {self.translation.vel[2]: .2f}",
            f"lin acc xyz  : {self.pose.linear_acc_world[0]: .3f} {self.pose.linear_acc_world[1]: .3f} {self.pose.linear_acc_world[2]: .3f}",
            "",
            f"last file    : {Path(self.last_recording_path).name if self.last_recording_path else '-'}",
            f"clip error   : {self.last_recording_error or '-'}",
            "",
            f"tof ok       : {tof.get('ok')}",
            "",
            "calibration",
            f"status       : {self.calibration_status}",
        ]
        if self.calibration_active:
            lines.extend(
                [
                    f"progress     : {100.0 * self.calibration_stats.get('progress', 0.0):.0f}%",
                    f"still        : {self.calibration_stats.get('still_count', 0)} / {self.calibration_stats.get('total_count', 0)}",
                ]
            )
        elif self.calibration_stats:
            lines.extend(
                [
                    f"still ratio  : {100.0 * self.calibration_stats.get('still_ratio', 0.0):.0f}%",
                    f"used samples : a={self.calibration_stats.get('acc_used', 0)} g={self.calibration_stats.get('gyro_used', 0)} m={self.calibration_stats.get('mag_used', 0)}",
                ]
            )
        text = "\n".join(lines)
        if text != self.last_status:
            self.status_box.setPlainText(text)
            self.last_status = text

    def on_timer(self):
        self.read_frames()
        self.playback_step()
        now = time.perf_counter()
        if now - self.last_status_s >= (STATUS_INTERVAL_MS / 1000.0):
            self.update_status()
            self.last_status_s = now
        if self.scene_dirty and (now - self.last_scene_render_s >= (RENDER_INTERVAL_MS / 1000.0)):
            self.render_scene()
            self.last_render_tick_ms = self.latest.get("tick_ms") if self.latest else None
            self.last_scene_render_s = now
        if self.fpv_dirty and (now - self.last_fpv_render_s >= (FPV_RENDER_INTERVAL_MS / 1000.0)):
            self.render_fpv()
            self.last_fpv_render_s = now
        if self.hist_dirty and (now - self.last_hist_render_s >= (FPV_RENDER_INTERVAL_MS / 1000.0)):
            self.render_histogram()
            self.last_hist_render_s = now

    def closeEvent(self, event):
        try:
            if self.ser is not None:
                self.ser.close()
        finally:
            super().closeEvent(event)


def main():
    app = QtWidgets.QApplication(sys.argv)
    win = VoxelViewer()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
