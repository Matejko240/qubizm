import json
import math
import time
import copy
from collections import deque
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.tri as mtri
import numpy as np
import serial
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection, PolyCollection
from matplotlib.widgets import Button, CheckButtons, Slider
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

PORT = "COM10"
BAUD = 115200
TOF_SIZE = 8
MAX_READS_PER_TICK = 100
SERIAL_MAX_BYTES_PER_TICK = 32768
SERIAL_BACKLOG_DROP_BYTES = 131072
TOF_DIAGONAL_FOV_DEG = 65.0
SPHERE_AZIMUTH_RES_DEG = 0.50
SPHERE_ELEVATION_RES_DEG = 0.50
MAP_AZIMUTH_EXTENT_DEG = 360.0
MAP_ELEVATION_EXTENT_DEG = 180.0
MAP_DECAY_TAU_S = 4.0
MAP_MIN_WEIGHT = 0.08
TOF_MIN_RANGE_M = 0.10
TOF_MAX_RANGE_M = 4.00
ACCEL_DEADBAND_G = 0.025
CALIBRATION_DURATION_MS = 5000
CALIBRATION_MIN_STILL_RATIO = 0.70
CALIBRATION_GYRO_STILL_DPS = 4.0
CALIBRATION_ACC_STILL_G = 0.06
CALIBRATION_FRONT_UP_DOT_MIN = 0.90
AXIS_CAPTURE_DURATION_MS = 1800
HEAVY_RENDER_INTERVAL_S = 0.20
FP_MAX_RENDER_POINTS = 220
FP_NEIGHBOR_WINDOW = 18
FP_MAX_LINKS_PER_POINT = 3
FP_MAX_SURFACES = 220
MAP_MAX_RENDER_POINTS = 700
FP_BG_COLOR = "#dcefdc"
RECORDINGS_DIR = Path("recordings")
FP_STITCH_NEAR_SPAN_FRAC_DEFAULT = 0.53
FP_STITCH_MID_SPAN_FRAC_DEFAULT = 0.53
FP_STITCH_FAR_SPAN_FRAC_DEFAULT = 0.53
FP_VIEW_GAP_FRAC_DEFAULT = 0.59
FP_DEPTH_THRESHOLD_CM_DEFAULT = 10.0
FP_COMMON_CONNECT_MULT_DEFAULT = 1.50
FP_DISTANCE_BIAS_DEFAULT = 1.45
DOT_SIZE_SCALE_DEFAULT = 0.15
MIN_RENDER_PROBABILITY_DEFAULT = 0.18
NEAR_SPREAD_SCALE_DEFAULT = 0.50
MID_SPREAD_SCALE_DEFAULT = 1.10
FAR_SPREAD_SCALE_DEFAULT = 1.25
FP_STITCH_3D_SCALE_DEFAULT = 1.0
FP_STITCH_PROJ_MAX_DEFAULT = 0.22
FP_STITCH_DEPTH_SCALE_DEFAULT = 1.0
FP_STITCH_MIN_AREA_DEFAULT = 0.0006
FP_KNN_NEIGHBORS_DEFAULT = 6
FP_KNN_RADIUS_DEFAULT = 0.35
DEVICE_HALF_WIDTH_M = 0.28
DEVICE_HALF_DEPTH_M = 0.03
DEVICE_HALF_HEIGHT_M = 0.55
DEVICE_CENTER_UP_M = 0.70
DEVICE_SENSOR_FACE_LOCAL = np.array([0.0, DEVICE_HALF_DEPTH_M, 0.0], dtype=float)

TOF_HALF_DIAGONAL_RAD = math.radians(TOF_DIAGONAL_FOV_DEG) / 2.0
TOF_AXIS_HALF_RAD = math.atan(math.tan(TOF_HALF_DIAGONAL_RAD) / math.sqrt(2.0))

# Remap sensor axes into the app frame: right, forward, up.
# Learned mapping for the current vertical mounting:
# app right <- sensor Y, app forward <- sensor Z, app up <- sensor X.
DEFAULT_IMU_AXIS_ORDER = (1, 2, 0)
DEFAULT_IMU_AXIS_SIGN = np.array([1.0, 1.0, 1.0], dtype=float)
CALIBRATION_EXPECTED_ACC_G = np.array([0.0, 1.0, 0.0], dtype=float)

NEAR_SPLAT_OFFSETS = np.array(
    [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        [0.0, 0.0, -1.0],
        [0.5, 0.0, 0.0],
        [-0.5, 0.0, 0.0],
        [0.0, 0.0, 0.5],
        [0.0, 0.0, -0.5],
        [0.75, 0.0, 0.75],
        [0.75, 0.0, -0.75],
        [-0.75, 0.0, 0.75],
        [-0.75, 0.0, -0.75],
    ],
    dtype=float,
)
NEAR_SPLAT_WEIGHTS = np.array([0.20, 0.09, 0.09, 0.09, 0.09, 0.07, 0.07, 0.07, 0.07, 0.04, 0.04, 0.04, 0.04], dtype=float)

MID_SPLAT_OFFSETS = np.array(
    [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        [0.0, 0.0, -1.0],
        [0.0, 0.35, 0.0],
        [0.0, -0.35, 0.0],
    ],
    dtype=float,
)
MID_SPLAT_WEIGHTS = np.array([0.28, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12], dtype=float)

FAR_SPLAT_OFFSETS = np.array(
    [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        [0.0, 0.0, -1.0],
    ],
    dtype=float,
)
FAR_SPLAT_WEIGHTS = np.array([0.40, 0.15, 0.15, 0.15, 0.15], dtype=float)


def fmt_xyz(values):
    if values is None or len(values) != 3:
        return "null"
    return f"x={values[0]:>6} y={values[1]:>6} z={values[2]:>6}"


def distance_gray_rgba(distances, max_distance, min_distance=TOF_MIN_RANGE_M, alpha=1.0, near_level=1.0, far_level=0.0):
    dist = np.asarray(distances, dtype=float)
    span = max(1e-6, float(max_distance) - float(min_distance))
    norm = np.clip((dist - float(min_distance)) / span, 0.0, 1.0)
    # Emphasize near-range contrast while compressing far-range differences.
    near_level = float(np.clip(near_level, 0.0, 1.0))
    far_level = float(np.clip(far_level, 0.0, 1.0))
    intensity = far_level + (near_level - far_level) * (1.0 - np.sqrt(norm))
    alpha_arr = np.asarray(alpha, dtype=float)
    if alpha_arr.ndim == 0:
        alpha_arr = np.full(dist.shape, float(alpha_arr), dtype=float)
    return np.column_stack([intensity, intensity, intensity, np.clip(alpha_arr, 0.0, 1.0)])


def ttl_color_rgba(distances, max_distance, weights, decay_tau, min_weight, alpha=1.0, start_white=False, near_level=1.0, far_level=0.0):
    if start_white:
        dist = np.asarray(distances, dtype=float)
        base = np.ones((dist.shape[0], 4), dtype=float)
        base[:, 3] = 1.0
    else:
        base = distance_gray_rgba(
            distances,
            max_distance,
            min_distance=TOF_MIN_RANGE_M,
            alpha=1.0,
            near_level=near_level,
            far_level=far_level,
        )
    w = np.clip(np.asarray(weights, dtype=float), max(min_weight, 1e-6), 1.0)
    min_w = max(float(min_weight), 1e-6)
    max_ttl = max(1e-6, float(decay_tau) * math.log(1.0 / min_w))
    ttl = float(decay_tau) * np.log(w / min_w)
    ttl_norm = np.clip(ttl / max_ttl, 0.0, 1.0)
    colors = base.copy()
    colors[:, :3] *= ttl_norm[:, None]
    alpha_arr = np.asarray(alpha, dtype=float)
    if alpha_arr.ndim == 0:
        alpha_arr = np.full(w.shape, float(alpha_arr), dtype=float)
    colors[:, 3] = np.clip(alpha_arr, 0.0, 1.0)
    return colors


def fp_arc_spread_gain(tri_proj, tri_mean_forward):
    proj_radius = np.mean(np.linalg.norm(tri_proj, axis=2), axis=1)
    edge_ref = max(1e-6, math.tan(TOF_AXIS_HALF_RAD))
    off_axis_gain = 1.0 + 0.45 * np.clip(proj_radius / edge_ref, 0.0, 1.2)

    # Close surfaces occupy a wider angular span in the view, so projected gaps
    # between neighboring hits can be larger even when they belong together.
    near_gain = 1.0 + 0.60 * np.clip((1.10 - tri_mean_forward) / 0.95, 0.0, 1.0)
    return np.clip(off_axis_gain * near_gain, 1.0, 2.4)


def tof_matrix_width_m(distance_m):
    dist = np.asarray(distance_m, dtype=float)
    return 2.0 * dist * math.tan(TOF_AXIS_HALF_RAD)


def safe_fp_triangulation(proj_x, proj_y):
    points_2d = np.column_stack([proj_x, proj_y])
    if len(points_2d) < 3:
        return None, None

    rounded = np.round(points_2d, decimals=4)
    _, unique_idx = np.unique(rounded, axis=0, return_index=True)
    unique_idx = np.sort(unique_idx)
    if len(unique_idx) < 3:
        return None, None

    unique_points = points_2d[unique_idx]
    centered = unique_points - np.mean(unique_points, axis=0, keepdims=True)
    if np.linalg.matrix_rank(centered) < 2:
        return None, None

    try:
        triangulation = mtri.Triangulation(unique_points[:, 0], unique_points[:, 1])
    except RuntimeError:
        return None, None

    return triangulation, unique_idx


def knn_fp_triangles(proj_x, proj_y, k=6, max_radius=None):
    points_2d = np.column_stack([proj_x, proj_y])
    num_points = len(points_2d)
    if num_points < 3:
        return np.empty((0, 3), dtype=int)

    k = max(2, min(int(k), num_points - 1))
    deltas = points_2d[:, None, :] - points_2d[None, :, :]
    dist2 = np.sum(deltas * deltas, axis=2)
    np.fill_diagonal(dist2, np.inf)
    neighbor_idx = np.argpartition(dist2, kth=k - 1, axis=1)[:, :k]
    radius2 = None if max_radius is None else float(max_radius) * float(max_radius)

    tri_set = set()
    for center in range(num_points):
        neighbors = neighbor_idx[center]
        for ia in range(len(neighbors)):
            a = int(neighbors[ia])
            if a == center:
                continue
            if radius2 is not None and dist2[center, a] > radius2:
                continue
            for ib in range(ia + 1, len(neighbors)):
                b = int(neighbors[ib])
                if b == center or a == b:
                    continue
                if radius2 is not None and dist2[center, b] > radius2:
                    continue
                tri = tuple(sorted((center, a, b)))
                tri_set.add(tri)

    if not tri_set:
        return np.empty((0, 3), dtype=int)
    return np.asarray(sorted(tri_set), dtype=int)


def rotation_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


class AxisMapper:
    APP_AXIS_LABELS = ("right", "forward", "up")
    SHORT_LABELS = ("R", "F", "U")
    SENSOR_AXIS_LABELS = ("X", "Y", "Z")

    def __init__(self, default_order=DEFAULT_IMU_AXIS_ORDER, default_sign=DEFAULT_IMU_AXIS_SIGN):
        self.order = list(default_order)
        self.sign = np.array(default_sign, dtype=float)
        self.capture_axis = None
        self.capture_start_ms = None
        self.capture_deadline_ms = None
        self.capture_samples = []
        self.status = f"default {self.mapping_text()}"

    def transform(self, values):
        if values is None or len(values) != 3:
            return None
        arr = np.array(values, dtype=float)
        out = np.zeros(3, dtype=float)
        for app_idx, sensor_idx in enumerate(self.order):
            out[app_idx] = self.sign[app_idx] * arr[sensor_idx]
        return out

    def mapping_text(self):
        parts = []
        for app_idx, sensor_idx in enumerate(self.order):
            sign = "+" if self.sign[app_idx] >= 0 else "-"
            parts.append(f"{self.SHORT_LABELS[app_idx]}<-{sign}{self.SENSOR_AXIS_LABELS[sensor_idx]}")
        return " ".join(parts)

    def start_capture(self, app_axis_idx, tick_ms):
        hints = {
            0: "rotate mainly about the device right axis",
            1: "rotate mainly about the device forward axis",
            2: "rotate mainly about the device up axis",
        }
        self.capture_axis = int(app_axis_idx)
        self.capture_start_ms = tick_ms if tick_ms is not None else 0
        self.capture_deadline_ms = self.capture_start_ms + AXIS_CAPTURE_DURATION_MS
        self.capture_samples = []
        self.status = f"learning {self.APP_AXIS_LABELS[self.capture_axis]}: {hints[self.capture_axis]}"

    def update_capture(self, tick_ms, gyro_raw):
        if self.capture_axis is None or tick_ms is None:
            return False
        if gyro_raw is not None:
            self.capture_samples.append(np.array(gyro_raw, dtype=float) / 1000.0)
        if tick_ms >= self.capture_deadline_ms:
            return self.finish_capture()
        return False

    def finish_capture(self):
        changed = False
        label = self.APP_AXIS_LABELS[self.capture_axis] if self.capture_axis is not None else "axis"
        if len(self.capture_samples) < 5:
            self.status = f"{label}: too few samples"
        else:
            samples = np.array(self.capture_samples, dtype=float)
            energy = np.sum(np.abs(samples), axis=0)
            sensor_idx = int(np.argmax(energy))
            signed_sum = float(np.sum(samples[:, sensor_idx]))
            self.order[self.capture_axis] = sensor_idx
            self.sign[self.capture_axis] = 1.0 if signed_sum >= 0.0 else -1.0
            self.status = f"updated {self.mapping_text()}"
            changed = True

        self.capture_axis = None
        self.capture_start_ms = None
        self.capture_deadline_ms = None
        self.capture_samples = []
        return changed

    def flip(self, app_axis_idx):
        app_axis_idx = int(app_axis_idx)
        self.sign[app_axis_idx] *= -1.0
        self.status = f"flipped {self.APP_AXIS_LABELS[app_axis_idx]}: {self.mapping_text()}"

    def capture_progress(self, tick_ms):
        if self.capture_axis is None or self.capture_start_ms is None or tick_ms is None:
            return None
        return float(np.clip((tick_ms - self.capture_start_ms) / AXIS_CAPTURE_DURATION_MS, 0.0, 1.0))


def log_axis_mapping(prefix, mapper):
    print(f"[axes] {prefix} | {mapper.mapping_text()} | {mapper.status}", flush=True)


def project_tof_to_local_points(tof_frame_mm):
    points = []
    if tof_frame_mm is None:
        return np.empty((0, 3), dtype=float)

    for row in range(TOF_SIZE):
        for col in range(TOF_SIZE):
            dist_mm = tof_frame_mm[row, col]
            if not np.isfinite(dist_mm):
                continue
            dist_m = float(dist_mm) / 1000.0
            if not (TOF_MIN_RANGE_M <= dist_m <= TOF_MAX_RANGE_M):
                continue

            yaw = ((col - (TOF_SIZE - 1) / 2.0) / ((TOF_SIZE - 1) / 2.0)) * TOF_AXIS_HALF_RAD
            pitch = (((TOF_SIZE - 1) / 2.0 - row) / ((TOF_SIZE - 1) / 2.0)) * TOF_AXIS_HALF_RAD

            direction = np.array(
                [math.tan(yaw), 1.0, math.tan(pitch)],
                dtype=float,
            )
            direction /= np.linalg.norm(direction)
            points.append(direction * dist_m)

    if not points:
        return np.empty((0, 3), dtype=float)
    return np.array(points, dtype=float)


def robust_mean_and_std(samples):
    if samples is None or len(samples) == 0:
        return None, None, 0
    arr = np.array(samples, dtype=float)
    if arr.ndim == 1:
        arr = arr[:, None]

    median = np.median(arr, axis=0)
    mad = np.median(np.abs(arr - median), axis=0)
    mad = np.where(mad < 1e-9, 1e-9, mad)
    score = np.abs(arr - median) / mad
    mask = np.all(score < 3.5, axis=1)
    filtered = arr[mask]
    if filtered.shape[0] == 0:
        filtered = arr
        mask = np.ones(arr.shape[0], dtype=bool)
    mean = np.mean(filtered, axis=0)
    std = np.std(filtered, axis=0)
    return mean, std, int(mask.sum())


class WorldMap:
    def __init__(
        self,
        azimuth_resolution_deg=SPHERE_AZIMUTH_RES_DEG,
        elevation_resolution_deg=SPHERE_ELEVATION_RES_DEG,
        decay_tau=MAP_DECAY_TAU_S,
        near_spread_scale=NEAR_SPREAD_SCALE_DEFAULT,
        mid_spread_scale=MID_SPREAD_SCALE_DEFAULT,
        far_spread_scale=FAR_SPREAD_SCALE_DEFAULT,
        near_band_max=0.85,
        mid_band_max=1.80,
    ):
        self.azimuth_resolution_deg = float(azimuth_resolution_deg)
        self.elevation_resolution_deg = float(elevation_resolution_deg)
        self.decay_tau = float(decay_tau)
        self.near_spread_scale = float(near_spread_scale)
        self.mid_spread_scale = float(mid_spread_scale)
        self.far_spread_scale = float(far_spread_scale)
        self.near_band_max = float(near_band_max)
        self.mid_band_max = float(mid_band_max)
        self.current_tick_ms = None
        self.last_prune_ms = None
        self.last_compact_ms = None
        self._configure_storage()

    def _configure_storage(self):
        self.n_az = max(8, int(round(MAP_AZIMUTH_EXTENT_DEG / self.azimuth_resolution_deg)))
        self.n_el = max(4, int(round(MAP_ELEVATION_EXTENT_DEG / self.elevation_resolution_deg)))
        self.azimuth_resolution_deg = MAP_AZIMUTH_EXTENT_DEG / self.n_az
        self.elevation_resolution_deg = MAP_ELEVATION_EXTENT_DEG / self.n_el

        az_centers_deg = -MAP_AZIMUTH_EXTENT_DEG / 2.0 + (np.arange(self.n_az, dtype=float) + 0.5) * self.azimuth_resolution_deg
        el_centers_deg = -MAP_ELEVATION_EXTENT_DEG / 2.0 + (np.arange(self.n_el, dtype=float) + 0.5) * self.elevation_resolution_deg
        az_grid_deg, el_grid_deg = np.meshgrid(az_centers_deg, el_centers_deg)
        az_grid_rad = np.radians(az_grid_deg)
        el_grid_rad = np.radians(el_grid_deg)
        cp = np.cos(el_grid_rad)
        dir_grid = np.stack(
            [
                np.sin(az_grid_rad) * cp,
                np.cos(az_grid_rad) * cp,
                np.sin(el_grid_rad),
            ],
            axis=-1,
        )
        self.flat_dirs = dir_grid.reshape(-1, 3).astype(float)
        self.flat_iy = np.repeat(np.arange(self.n_el, dtype=np.int32), self.n_az)
        self.flat_ix = np.tile(np.arange(self.n_az, dtype=np.int32), self.n_el)

        size = self.n_el * self.n_az
        self.weight_flat = np.zeros(size, dtype=np.float32)
        self.dist_flat = np.zeros(size, dtype=np.float32)
        self.last_seen_flat = np.zeros(size, dtype=np.float64)
        self.active_flat = np.zeros(size, dtype=bool)
        self._rebuild_kernels()

    def clear(self):
        self.weight_flat.fill(0.0)
        self.dist_flat.fill(0.0)
        self.last_seen_flat.fill(0.0)
        self.active_flat.fill(False)
        self.current_tick_ms = None
        self.last_prune_ms = None
        self.last_compact_ms = None

    def active_count(self):
        return int(np.count_nonzero(self.active_flat))

    def set_azimuth_resolution(self, value):
        self.azimuth_resolution_deg = float(value)
        self._configure_storage()
        self.current_tick_ms = None
        self.last_prune_ms = None

    def set_elevation_resolution(self, value):
        self.elevation_resolution_deg = float(value)
        self._configure_storage()
        self.current_tick_ms = None
        self.last_prune_ms = None

    def _distance_band(self, dist_m):
        if dist_m < self.near_band_max:
            return "near"
        if dist_m < self.mid_band_max:
            return "mid"
        return "far"

    def angular_resolution_summary(self):
        return self.azimuth_resolution_deg, self.elevation_resolution_deg

    def _angles_from_point(self, point):
        x, y, z = np.asarray(point, dtype=float)
        horiz = math.hypot(x, y)
        yaw = math.atan2(x, y)
        pitch = math.atan2(z, max(horiz, 1e-9))
        return yaw, pitch

    def _point_from_angles(self, yaw, pitch, dist):
        cp = math.cos(pitch)
        return np.array(
            [
                dist * math.sin(yaw) * cp,
                dist * math.cos(yaw) * cp,
                dist * math.sin(pitch),
            ],
            dtype=float,
        )

    @staticmethod
    def _ray_spacing_m(dist_m):
        dist = np.asarray(dist_m, dtype=float)
        spacing = dist * (2.0 * math.tan(TOF_AXIS_HALF_RAD) / max(1, TOF_SIZE - 1))
        return np.maximum(0.018, spacing)

    def sample_spacing_for_distance(self, dist_m):
        dist = np.asarray(dist_m, dtype=float)
        az_rad = math.radians(self.azimuth_resolution_deg)
        el_rad = math.radians(self.elevation_resolution_deg)
        spacing = dist * max(az_rad, el_rad)
        spacing = np.maximum(spacing, self._ray_spacing_m(dist))
        if np.ndim(spacing) == 0:
            return float(spacing)
        return spacing

    def _rebuild_kernels(self):
        self.kernel_near = self._build_kernel(self._fill_radius_deg(max(0.01, min(self.near_band_max * 0.5, self.near_band_max - 1e-3))))
        self.kernel_mid = self._build_kernel(self._fill_radius_deg(0.5 * (self.near_band_max + self.mid_band_max)))
        self.kernel_far = self._build_kernel(self._fill_radius_deg(self.mid_band_max + 0.5))

    def _fill_radius_deg(self, dist_m):
        base_pixel_deg = math.degrees((2.0 * TOF_AXIS_HALF_RAD) / max(1, TOF_SIZE - 1))
        if dist_m < self.near_band_max:
            return np.clip(1.15 * self.near_spread_scale * base_pixel_deg, 0.4, 12.0)
        if dist_m < self.mid_band_max:
            return np.clip(0.95 * self.mid_spread_scale * base_pixel_deg, 0.4, 12.0)
        return np.clip(0.80 * self.far_spread_scale * base_pixel_deg, 0.4, 12.0)

    def _build_kernel(self, fill_radius_deg):
        fill_radius_deg = max(float(fill_radius_deg), 1e-6)
        rx = max(0, int(math.ceil(fill_radius_deg / self.azimuth_resolution_deg)))
        ry = max(0, int(math.ceil(fill_radius_deg / self.elevation_resolution_deg)))
        dx = np.arange(-rx, rx + 1, dtype=np.int32)
        dy = np.arange(-ry, ry + 1, dtype=np.int32)
        dx_grid, dy_grid = np.meshgrid(dx, dy)
        norm = np.sqrt(
            ((dx_grid.astype(float) * self.azimuth_resolution_deg) / fill_radius_deg) ** 2
            + ((dy_grid.astype(float) * self.elevation_resolution_deg) / fill_radius_deg) ** 2
        )
        mask = norm <= 1.0
        if not np.any(mask):
            return (
                np.zeros(1, dtype=np.int32),
                np.zeros(1, dtype=np.int32),
                np.ones(1, dtype=np.float32),
            )
        weights = (0.35 + 0.55 * (1.0 - norm[mask])).astype(np.float32)
        return dy_grid[mask].astype(np.int32), dx_grid[mask].astype(np.int32), weights

    def _center_indices_from_deg(self, yaw_deg, pitch_deg):
        yaw_deg = np.asarray(yaw_deg, dtype=float)
        pitch_deg = np.asarray(pitch_deg, dtype=float)
        ix = np.floor((yaw_deg + MAP_AZIMUTH_EXTENT_DEG / 2.0) / self.azimuth_resolution_deg).astype(np.int32) % self.n_az
        iy = np.clip(
            np.floor((pitch_deg + MAP_ELEVATION_EXTENT_DEG / 2.0) / self.elevation_resolution_deg).astype(np.int32),
            0,
            self.n_el - 1,
        )
        return ix, iy

    def _clear_flat_indices(self, flat_idx):
        if flat_idx.size == 0:
            return
        self.weight_flat[flat_idx] = 0.0
        self.dist_flat[flat_idx] = 0.0
        self.last_seen_flat[flat_idx] = 0.0
        self.active_flat[flat_idx] = False

    def _decay_indices_to_tick(self, flat_idx, tick_ms):
        flat_idx = np.asarray(flat_idx, dtype=np.int64)
        if flat_idx.size == 0 or tick_ms is None:
            return
        active_mask = self.active_flat[flat_idx]
        if not np.any(active_mask):
            return
        idx = flat_idx[active_mask]
        age_s = np.maximum(0.0, (tick_ms - self.last_seen_flat[idx]) / 1000.0)
        eff = self.weight_flat[idx] * np.exp(-age_s / self.decay_tau)
        stale = eff < MAP_MIN_WEIGHT
        if np.any(stale):
            self._clear_flat_indices(idx[stale])
        keep_idx = idx[~stale]
        if keep_idx.size:
            self.weight_flat[keep_idx] = eff[~stale].astype(np.float32)
            self.last_seen_flat[keep_idx] = float(tick_ms)

    def _prune_active(self, tick_ms):
        if tick_ms is None:
            return
        active_idx = np.flatnonzero(self.active_flat)
        if active_idx.size == 0:
            return
        self._decay_indices_to_tick(active_idx, tick_ms)

    def _compact_near_bins(self, tick_ms):
        if tick_ms is None:
            return
        active_idx = np.flatnonzero(self.active_flat)
        if active_idx.size == 0:
            return
        dists = self.dist_flat[active_idx].astype(float)
        near_mask = dists < self.near_band_max
        if not np.any(near_mask):
            return

        idx = active_idx[near_mask]
        dists = dists[near_mask]
        weights = self.weight_flat[idx].astype(float)
        iy = self.flat_iy[idx].astype(int)
        ix = self.flat_ix[idx].astype(int)

        # Very close points are merged more aggressively; modestly close points use 2x2 groups.
        stride = np.where(dists < max(0.20, 0.55 * self.near_band_max), 3, 2).astype(int)
        group_y = iy // stride
        group_x = ix // stride

        groups = {}
        for n, gkey in enumerate(zip(stride.tolist(), group_y.tolist(), group_x.tolist())):
            groups.setdefault(gkey, []).append(n)

        for (_stride, _gy, _gx), members in groups.items():
            if len(members) <= 1:
                continue
            member_idx = idx[members]
            member_d = dists[members]
            member_w = weights[members]
            # Don't collapse separate surfaces that only happen to be nearby angularly.
            if float(np.max(member_d) - np.min(member_d)) > 0.12:
                continue
            member_iy = iy[members].astype(float)
            member_ix = ix[members].astype(float)
            total_w = float(np.sum(member_w))
            if total_w <= 1e-9:
                continue
            rep_iy = int(np.clip(round(float(np.sum(member_iy * member_w) / total_w)), 0, self.n_el - 1))
            rep_ix = int(round(float(np.sum(member_ix * member_w) / total_w))) % self.n_az
            rep_flat = rep_iy * self.n_az + rep_ix
            rep_dist = float(np.sum(member_d * member_w) / total_w)
            # Preserve accumulated certainty when compacting multiple nearby bins into one
            # representative bin; otherwise compacted regions look like they were never saved.
            rep_weight = min(1.0, total_w)
            rep_seen = float(np.max(self.last_seen_flat[member_idx]))
            had_existing = bool(self.active_flat[rep_flat] and rep_flat not in set(member_idx.tolist()))
            old_w = float(self.weight_flat[rep_flat]) if had_existing else 0.0
            old_dist = float(self.dist_flat[rep_flat]) if had_existing else 0.0
            old_seen = float(self.last_seen_flat[rep_flat]) if had_existing else 0.0

            self._clear_flat_indices(member_idx)
            if had_existing and old_dist > 0.0:
                mix_w = max(old_w + rep_weight, 1e-9)
                self.dist_flat[rep_flat] = float((old_dist * old_w + rep_dist * rep_weight) / mix_w)
                self.weight_flat[rep_flat] = min(1.0, old_w + rep_weight)
            else:
                self.dist_flat[rep_flat] = rep_dist
                self.weight_flat[rep_flat] = rep_weight
            self.last_seen_flat[rep_flat] = max(old_seen, rep_seen)
            self.active_flat[rep_flat] = True

    def decay(self, tick_ms):
        if tick_ms is None:
            return
        self.current_tick_ms = float(tick_ms)
        if self.last_prune_ms is None or (tick_ms - self.last_prune_ms) >= 200:
            self._prune_active(tick_ms)
            self.last_prune_ms = float(tick_ms)
        if self.last_compact_ms is None or (tick_ms - self.last_compact_ms) >= 350:
            self._compact_near_bins(tick_ms)
            self.last_compact_ms = float(tick_ms)

    def add_points(self, points_local, tick_ms=None, origin=None):
        if points_local is None or len(points_local) == 0:
            return
        if tick_ms is not None:
            self.decay(tick_ms)
        points = np.asarray(points_local, dtype=float)
        dists = np.linalg.norm(points, axis=1)
        valid = (dists >= TOF_MIN_RANGE_M) & (dists <= TOF_MAX_RANGE_M)
        if not np.any(valid):
            return
        points = points[valid]
        dists = dists[valid]
        horiz = np.hypot(points[:, 0], points[:, 1])
        yaw_deg = np.degrees(np.arctan2(points[:, 0], points[:, 1]))
        pitch_deg = np.degrees(np.arctan2(points[:, 2], np.clip(horiz, 1e-9, None)))
        ix_center, iy_center = self._center_indices_from_deg(yaw_deg, pitch_deg)

        band_masks = (
            (dists < self.near_band_max, self.kernel_near),
            ((dists >= self.near_band_max) & (dists < self.mid_band_max), self.kernel_mid),
            (dists >= self.mid_band_max, self.kernel_far),
        )
        effective_tick = self.current_tick_ms if tick_ms is None else float(tick_ms)
        for mask, kernel in band_masks:
            if not np.any(mask):
                continue
            kernel_dy, kernel_dx, kernel_weight = kernel
            band_ix = ix_center[mask]
            band_iy = iy_center[mask]
            band_dist = dists[mask]
            for cx, cy, dist in zip(band_ix, band_iy, band_dist):
                iy = cy + kernel_dy
                valid_y = (iy >= 0) & (iy < self.n_el)
                if not np.any(valid_y):
                    continue
                iy_valid = iy[valid_y]
                ix_valid = (cx + kernel_dx[valid_y]) % self.n_az
                add_w = kernel_weight[valid_y]
                flat_idx = iy_valid.astype(np.int64) * self.n_az + ix_valid.astype(np.int64)
                self._decay_indices_to_tick(flat_idx, effective_tick)
                old_w = self.weight_flat[flat_idx].astype(float)
                total_w = old_w + add_w
                self.dist_flat[flat_idx] = np.where(
                    total_w > 1e-9,
                    ((self.dist_flat[flat_idx].astype(float) * old_w) + (dist * add_w)) / np.maximum(total_w, 1e-9),
                    dist,
                ).astype(np.float32)
                self.weight_flat[flat_idx] = np.minimum(1.0, total_w).astype(np.float32)
                self.last_seen_flat[flat_idx] = effective_tick
                self.active_flat[flat_idx] = True

    def _active_indices(self, tick_ms=None):
        effective_tick = self.current_tick_ms if tick_ms is None else float(tick_ms)
        if effective_tick is not None:
            self.decay(effective_tick)
        return np.flatnonzero(self.active_flat)

    def query_local_points(self, rot_current, map_frame_rot, max_distance, tick_ms=None, min_forward=None, max_points=None, score_bias=1.0, min_probability=0.0):
        active_idx = self._active_indices(tick_ms)
        if active_idx.size == 0:
            return (
                np.empty((0, 3), dtype=float),
                np.empty((0,), dtype=float),
                [],
                np.empty((0,), dtype=float),
            )
        weights = self.weight_flat[active_idx].astype(float)
        distances = self.dist_flat[active_idx].astype(float)
        points_anchor = self.flat_dirs[active_idx] * distances[:, None]
        if map_frame_rot is None:
            points_local = points_anchor
        else:
            points_local = points_anchor @ map_frame_rot.T @ rot_current

        valid = distances <= float(max_distance)
        valid &= weights >= float(min_probability)
        if min_forward is not None:
            valid &= points_local[:, 1] > float(min_forward)
        if not np.any(valid):
            return (
                np.empty((0, 3), dtype=float),
                np.empty((0,), dtype=float),
                [],
                np.empty((0,), dtype=float),
            )

        active_idx = active_idx[valid]
        points_local = points_local[valid]
        weights = weights[valid]
        distances = distances[valid]

        if max_points is not None and len(points_local) > int(max_points):
            score = weights / np.power(np.clip(distances, 0.25, max_distance), float(score_bias))
            keep = np.argpartition(score, -int(max_points))[-int(max_points):]
            keep = keep[np.argsort(score[keep])[::-1]]
            active_idx = active_idx[keep]
            points_local = points_local[keep]
            weights = weights[keep]
            distances = distances[keep]

        keys = list(zip(self.flat_iy[active_idx].tolist(), self.flat_ix[active_idx].tolist()))
        return points_local, weights, keys, distances

    def snapshot(self):
        active_idx = self._active_indices()
        if active_idx.size == 0:
            return np.empty((0, 3), dtype=float), np.empty((0,), dtype=float)
        points = self.flat_dirs[active_idx] * self.dist_flat[active_idx][:, None]
        weights = self.weight_flat[active_idx].astype(float)
        return points, weights

    def snapshot_with_keys(self):
        active_idx = self._active_indices()
        if active_idx.size == 0:
            return np.empty((0, 3), dtype=float), np.empty((0,), dtype=float), []
        points = self.flat_dirs[active_idx] * self.dist_flat[active_idx][:, None]
        weights = self.weight_flat[active_idx].astype(float)
        keys = list(zip(self.flat_iy[active_idx].tolist(), self.flat_ix[active_idx].tolist()))
        return points, weights, keys

    @staticmethod
    def connected_components_26(keys, connect_range=1):
        if not keys:
            return []
        connect_range = max(1, int(connect_range))
        key_set = set(keys)
        visited = set()
        components = []
        for key in keys:
            if key in visited:
                continue
            stack = [key]
            visited.add(key)
            component = []
            while stack:
                current = stack.pop()
                component.append(current)
                cy, cx = current
                for dy in range(-connect_range, connect_range + 1):
                    for dx in range(-connect_range, connect_range + 1):
                        if dx == 0 and dy == 0:
                            continue
                        neighbor = (cy + dy, cx + dx)
                        if neighbor in key_set and neighbor not in visited:
                            visited.add(neighbor)
                            stack.append(neighbor)
            components.append(component)
        return components


class PoseTracker:
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.pos = np.zeros(3, dtype=float)
        self.linear_acc_world = np.zeros(3, dtype=float)
        self.last_tick_ms = None
        self.cal_roll = 0.0
        self.cal_pitch = 0.0
        self.cal_yaw = 0.0
        self.gyro_bias = np.zeros(3, dtype=float)
        self.accel_sensor_bias = np.zeros(3, dtype=float)
        self.accel_world_bias = np.zeros(3, dtype=float)
        self.acc_lp = None
        self.mag_lp = None
        self.calibrated = False
        self.stationary = False
        self.accel_noise_std = np.zeros(3, dtype=float)
        self.gyro_noise_std = np.zeros(3, dtype=float)
        self.mag_noise_std = np.zeros(3, dtype=float)
        self.accel_deadband_g = ACCEL_DEADBAND_G

    @staticmethod
    def accel_to_tilt(acc_g):
        ax, ay, az = acc_g
        roll = math.atan2(ay, az if abs(az) > 1e-6 else 1e-6)
        pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az) + 1e-6)
        return roll, pitch

    @staticmethod
    def wrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def tilt_compensated_yaw(mag_xyz, roll, pitch):
        mx, my, mz = mag_xyz
        mx2 = mx * math.cos(pitch) + mz * math.sin(pitch)
        my2 = (
            mx * math.sin(roll) * math.sin(pitch)
            + my * math.cos(roll)
            - mz * math.sin(roll) * math.cos(pitch)
        )
        return math.atan2(-my2, mx2 if abs(mx2) > 1e-9 else 1e-9)

    def calibrate(self, acc_xyz=None, gyro_xyz=None, mag_xyz=None, tick_ms=None, accel_sensor_bias=None, accel_noise_std=None, gyro_noise_std=None, mag_noise_std=None):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.pos[:] = 0.0
        self.linear_acc_world[:] = 0.0
        self.last_tick_ms = tick_ms
        self.acc_lp = None
        self.mag_lp = None
        self.stationary = False
        self.accel_sensor_bias[:] = 0.0
        self.accel_world_bias[:] = 0.0
        self.accel_noise_std[:] = 0.0
        self.gyro_noise_std[:] = 0.0
        self.mag_noise_std[:] = 0.0
        self.accel_deadband_g = ACCEL_DEADBAND_G

        if gyro_xyz is not None:
            self.gyro_bias = np.array(gyro_xyz, dtype=float) / 1000.0
        else:
            self.gyro_bias[:] = 0.0

        if acc_xyz is not None:
            acc_g = np.array(acc_xyz, dtype=float) / 1000.0
            self.cal_roll, self.cal_pitch = self.accel_to_tilt(acc_g)
            self.calibrated = True
        else:
            self.cal_roll = 0.0
            self.cal_pitch = 0.0
            self.calibrated = False

        if mag_xyz is not None and acc_xyz is not None:
            mag = np.array(mag_xyz, dtype=float)
            self.cal_yaw = self.tilt_compensated_yaw(mag, self.cal_roll, self.cal_pitch)
        else:
            self.cal_yaw = 0.0

        if accel_sensor_bias is not None:
            self.accel_sensor_bias = np.array(accel_sensor_bias, dtype=float)
        if accel_noise_std is not None:
            self.accel_noise_std = np.array(accel_noise_std, dtype=float)
            self.accel_deadband_g = max(ACCEL_DEADBAND_G, 3.0 * float(np.max(self.accel_noise_std)))
        if gyro_noise_std is not None:
            self.gyro_noise_std = np.array(gyro_noise_std, dtype=float)
        if mag_noise_std is not None:
            self.mag_noise_std = np.array(mag_noise_std, dtype=float)

    def apply_noise_calibration(self, gyro_xyz=None, tick_ms=None, accel_sensor_bias=None, accel_noise_std=None, gyro_noise_std=None, mag_noise_std=None):
        self.last_tick_ms = tick_ms
        self.acc_lp = None
        self.mag_lp = None
        self.stationary = False
        self.linear_acc_world[:] = 0.0
        self.accel_world_bias[:] = 0.0
        self.calibrated = True

        if gyro_xyz is not None:
            self.gyro_bias = np.array(gyro_xyz, dtype=float) / 1000.0
        if accel_sensor_bias is not None:
            self.accel_sensor_bias = np.array(accel_sensor_bias, dtype=float)
        if accel_noise_std is not None:
            self.accel_noise_std = np.array(accel_noise_std, dtype=float)
            self.accel_deadband_g = max(ACCEL_DEADBAND_G, 3.0 * float(np.max(self.accel_noise_std)))
        else:
            self.accel_deadband_g = ACCEL_DEADBAND_G
        if gyro_noise_std is not None:
            self.gyro_noise_std = np.array(gyro_noise_std, dtype=float)
        if mag_noise_std is not None:
            self.mag_noise_std = np.array(mag_noise_std, dtype=float)

    def update_orientation(self, acc_xyz, gyro_xyz, mag_xyz, tick_ms):
        if acc_xyz is None or gyro_xyz is None or tick_ms is None:
            return

        if self.last_tick_ms is None:
            self.last_tick_ms = tick_ms
            return

        dt = max(0.001, min((tick_ms - self.last_tick_ms) / 1000.0, 0.2))
        self.last_tick_ms = tick_ms

        acc_g = np.array(acc_xyz, dtype=float) / 1000.0 - self.accel_sensor_bias
        if self.acc_lp is None:
            self.acc_lp = acc_g.copy()
        self.acc_lp = 0.85 * self.acc_lp + 0.15 * acc_g

        if mag_xyz is not None:
            mag = np.array(mag_xyz, dtype=float)
            if self.mag_lp is None:
                self.mag_lp = mag.copy()
            self.mag_lp = 0.85 * self.mag_lp + 0.15 * mag

        gyro_dps = np.array(gyro_xyz, dtype=float) / 1000.0 - self.gyro_bias
        gyro_rps = np.radians(gyro_dps)
        gyro_norm_dps = float(np.linalg.norm(gyro_dps))

        acc_mag = float(np.linalg.norm(self.acc_lp))
        stationary = abs(acc_mag - 1.0) < 0.04 and gyro_norm_dps < 3.0
        self.stationary = stationary

        roll_acc, pitch_acc = self.accel_to_tilt(self.acc_lp)
        roll_acc -= self.cal_roll
        pitch_acc -= self.cal_pitch

        alpha = 0.985 if not stationary else 0.92
        self.roll = alpha * (self.roll + gyro_rps[0] * dt) + (1.0 - alpha) * roll_acc
        self.pitch = alpha * (self.pitch + gyro_rps[1] * dt) + (1.0 - alpha) * pitch_acc
        self.yaw = self.wrap_angle(self.yaw + gyro_rps[2] * dt)

        if self.mag_lp is not None:
            yaw_mag = self.tilt_compensated_yaw(self.mag_lp, self.roll + self.cal_roll, self.pitch + self.cal_pitch) - self.cal_yaw
            yaw_mag = self.wrap_angle(yaw_mag)
            yaw_err = self.wrap_angle(yaw_mag - self.yaw)
            yaw_blend = 0.04 if stationary else 0.01
            self.yaw = self.wrap_angle(self.yaw + yaw_blend * yaw_err)

        if stationary:
            self.gyro_bias = 0.995 * self.gyro_bias + 0.005 * (np.array(gyro_xyz, dtype=float) / 1000.0)
        self.update_linear_acc_world(acc_g, stationary)

    def update_linear_acc_world(self, acc_g, stationary):
        rot = rotation_matrix(self.roll, self.pitch, self.yaw)
        acc_world = rot @ acc_g
        linear_acc = acc_world - np.array([0.0, 0.0, 1.0], dtype=float)

        if stationary:
            self.accel_world_bias = 0.995 * self.accel_world_bias + 0.005 * linear_acc

        linear_acc = linear_acc - self.accel_world_bias
        linear_acc[np.abs(linear_acc) < self.accel_deadband_g] = 0.0
        self.linear_acc_world = 0.8 * self.linear_acc_world + 0.2 * linear_acc
        self.pos[:] = 0.0


class SensorDashboard:
    def __init__(self, port: str, baud: int):
        self.ser = None
        self.serial_error = None
        self.rx_buffer = ""
        try:
            self.ser = serial.Serial(port, baud, timeout=0, write_timeout=0)
        except serial.SerialException as exc:
            self.serial_error = f"Open failed: {exc}"

        self.latest = {}
        self.frame = np.full((TOF_SIZE, TOF_SIZE), np.nan, dtype=float)
        self.cal_acc_samples = deque(maxlen=40)
        self.cal_gyro_samples = deque(maxlen=40)
        self.cal_mag_samples = deque(maxlen=40)
        self.calibration_active = False
        self.calibration_start_ms = None
        self.calibration_deadline_ms = None
        self.calibration_status = "idle"
        self.calibration_stats = {}
        self.calibration_samples = []
        self.map_frame_rot = None
        self.max_display_distance_m = 3.95
        self.fp_distance_bias = FP_DISTANCE_BIAS_DEFAULT
        self.dot_size_scale = DOT_SIZE_SCALE_DEFAULT
        self.near_color_level = 1.00
        self.far_color_level = 0.00
        self.points_start_white = False
        self.use_triangulation = True
        self.show_all_fp_points = False
        self.min_render_probability = MIN_RENDER_PROBABILITY_DEFAULT
        self.connection_range_bins = 1
        self.fp_max_render_points = FP_MAX_RENDER_POINTS
        self.stitch_span_near = FP_STITCH_NEAR_SPAN_FRAC_DEFAULT
        self.stitch_span_mid = FP_STITCH_MID_SPAN_FRAC_DEFAULT
        self.stitch_span_far = FP_STITCH_FAR_SPAN_FRAC_DEFAULT
        self.stitch_view_gap_frac = FP_VIEW_GAP_FRAC_DEFAULT
        self.depth_threshold_cm = FP_DEPTH_THRESHOLD_CM_DEFAULT
        self.stitch_min_area = FP_STITCH_MIN_AREA_DEFAULT
        self.common_connect_mult = FP_COMMON_CONNECT_MULT_DEFAULT
        self.knn_neighbors = FP_KNN_NEIGHBORS_DEFAULT
        self.knn_radius = FP_KNN_RADIUS_DEFAULT
        self.view_mode = "behind"
        self.last_render_mode = None
        self.last_heavy_render_s = 0.0
        self.serial_backlog_drops = 0
        self.recording_active = False
        self.playback_active = False
        self.recorded_payloads = []
        self.last_recording_path = None
        self.last_recording_error = None
        self.playback_start_perf = None
        self.playback_loop_duration_ms = 0
        self.playback_index = 0
        self.fp_component_cache_sig = None
        self.fp_component_cache = None
        self.fp_triangulation_cache = {}
        self.axis_mapper = AxisMapper()
        log_axis_mapping("startup", self.axis_mapper)
        self.pose = PoseTracker()
        self.world_map = WorldMap()
        self.world_map.near_band_max = 0.85
        self.world_map.mid_band_max = 1.80

        self.fig = plt.figure(figsize=(22, 12))
        gs = self.fig.add_gridspec(2, 4, width_ratios=[1.0, 1.35, 1.10, 0.95], height_ratios=[1.0, 1.0])
        right_gs = gs[:, 3].subgridspec(3, 1, height_ratios=[1.05, 0.42, 0.83], hspace=0.18)

        self.ax_tof = self.fig.add_subplot(gs[:, 0])
        self.ax_3d = self.fig.add_subplot(gs[:, 1:3], projection="3d")
        self.ax_fp = self.fig.add_subplot(right_gs[0, 0])
        self.ax_hist = self.fig.add_subplot(right_gs[1, 0])
        self.ax_text = self.fig.add_subplot(right_gs[2, 0])

        self.im = self.ax_tof.imshow(self.frame, cmap="gray_r", vmin=100, vmax=4000)
        self.cbar = self.fig.colorbar(self.im, ax=self.ax_tof, fraction=0.046, pad=0.04)
        self.cbar.set_label("Distance (mm)")
        self.ax_tof.set_title("VL53L8A1 Distance Matrix")

        self.ax_fp.set_title("First-Person View")
        self.ax_fp.set_xlim(-1.1, 1.1)
        self.ax_fp.set_ylim(-0.9, 0.9)
        self.ax_fp.set_facecolor(FP_BG_COLOR)
        self.ax_fp.grid(True, alpha=0.12, color="black")
        self.ax_fp.axhline(0.0, color="black", lw=0.8, alpha=0.35)
        self.ax_fp.axvline(0.0, color="black", lw=0.8, alpha=0.35)
        self.ax_fp.set_xticks([])
        self.ax_fp.set_yticks([])

        self.ax_hist.set_title("Nearest Object By Slice")
        self.ax_hist.set_facecolor("white")
        self.ax_hist.set_xlim(-0.5, 7.5)
        self.ax_hist.set_ylim(self.max_display_distance_m, 0.0)
        self.ax_hist.set_xticks(range(8))
        self.ax_hist.set_xticklabels([str(i + 1) for i in range(8)], fontsize=8)
        self.ax_hist.set_ylabel("m", fontsize=8)
        self.ax_hist.tick_params(axis="y", labelsize=8)
        self.ax_hist.grid(True, axis="y", alpha=0.20, color="black")

        self.ax_text.axis("off")
        self.text = self.ax_text.text(
            0.0,
            1.0,
            "Waiting for data...",
            va="top",
            ha="left",
            family="monospace",
            fontsize=8.5,
        )
        self.control_help = {}
        self.tooltip = self.fig.text(
            0.985,
            0.985,
            "",
            ha="right",
            va="top",
            fontsize=9,
            color="white",
            bbox=dict(boxstyle="round,pad=0.35", facecolor="black", edgecolor="#444444", alpha=0.88),
            visible=False,
        )

        self.button_ax = self.fig.add_axes([0.76, 0.915, 0.15, 0.048])
        self.cal_button = Button(self.button_ax, "Calibrate 5s")
        self.cal_button.on_clicked(self.on_calibrate)
        self.record_ax = self.fig.add_axes([0.76, 0.855, 0.07, 0.042])
        self.record_button = Button(self.record_ax, "Record")
        self.record_button.on_clicked(self.on_record_toggle)
        self.play_ax = self.fig.add_axes([0.84, 0.855, 0.07, 0.042])
        self.play_button = Button(self.play_ax, "Play Loop")
        self.play_button.on_clicked(self.on_play_toggle)
        self.view_behind_ax = self.fig.add_axes([0.12, 0.915, 0.12, 0.048])
        self.view_behind_button = Button(self.view_behind_ax, "World View")
        self.view_behind_button.on_clicked(self.on_view_behind)
        self.view_device_ax = self.fig.add_axes([0.26, 0.915, 0.12, 0.048])
        self.view_device_button = Button(self.view_device_ax, "Device View")
        self.view_device_button.on_clicked(self.on_view_device)
        self.learn_r_ax = self.fig.add_axes([0.12, 0.855, 0.09, 0.042])
        self.learn_r_button = Button(self.learn_r_ax, "Learn Right")
        self.learn_r_button.on_clicked(lambda _event: self.on_learn_axis(0))
        self.learn_f_ax = self.fig.add_axes([0.225, 0.855, 0.09, 0.042])
        self.learn_f_button = Button(self.learn_f_ax, "Learn Fwd")
        self.learn_f_button.on_clicked(lambda _event: self.on_learn_axis(1))
        self.learn_u_ax = self.fig.add_axes([0.33, 0.855, 0.09, 0.042])
        self.learn_u_button = Button(self.learn_u_ax, "Learn Up")
        self.learn_u_button.on_clicked(lambda _event: self.on_learn_axis(2))
        self.flip_r_ax = self.fig.add_axes([0.45, 0.855, 0.09, 0.042])
        self.flip_r_button = Button(self.flip_r_ax, "Flip Right")
        self.flip_r_button.on_clicked(lambda _event: self.on_flip_axis(0))
        self.flip_f_ax = self.fig.add_axes([0.555, 0.855, 0.09, 0.042])
        self.flip_f_button = Button(self.flip_f_ax, "Flip Fwd")
        self.flip_f_button.on_clicked(lambda _event: self.on_flip_axis(1))
        self.flip_u_ax = self.fig.add_axes([0.66, 0.855, 0.09, 0.042])
        self.flip_u_button = Button(self.flip_u_ax, "Flip Up")
        self.flip_u_button.on_clicked(lambda _event: self.on_flip_axis(2))
        self.dist_slider_ax = self.fig.add_axes([0.43, 0.922, 0.24, 0.028])
        self.dist_slider = Slider(self.dist_slider_ax, "Max Range", 0.05, 8.00, valinit=self.max_display_distance_m, valstep=0.05)
        self.dist_slider.on_changed(self.on_distance_slider)
        self.style_slider(self.dist_slider)
        self.stitch3d_slider_ax = self.fig.add_axes([0.12, 0.795, 0.20, 0.022])
        self.stitch3d_slider = Slider(self.stitch3d_slider_ax, "Near Span", 0.02, 4.00, valinit=self.stitch_span_near, valstep=0.01)
        self.stitch3d_slider.on_changed(self.on_stitch3d_slider)
        self.style_slider(self.stitch3d_slider)
        self.stitchmid_slider_ax = self.fig.add_axes([0.12, 0.525, 0.20, 0.022])
        self.stitchmid_slider = Slider(self.stitchmid_slider_ax, "Mid Span", 0.02, 4.00, valinit=self.stitch_span_mid, valstep=0.01)
        self.stitchmid_slider.on_changed(self.on_stitchmid_slider)
        self.style_slider(self.stitchmid_slider)
        self.stitchfar_slider_ax = self.fig.add_axes([0.40, 0.525, 0.20, 0.022])
        self.stitchfar_slider = Slider(self.stitchfar_slider_ax, "Far Span", 0.02, 4.00, valinit=self.stitch_span_far, valstep=0.01)
        self.stitchfar_slider.on_changed(self.on_stitchfar_slider)
        self.style_slider(self.stitchfar_slider)
        self.stitch2d_slider_ax = self.fig.add_axes([0.40, 0.795, 0.20, 0.022])
        self.stitch2d_slider = Slider(self.stitch2d_slider_ax, "Screen Gap", 0.05, 1.20, valinit=self.stitch_view_gap_frac, valstep=0.01)
        self.stitch2d_slider.on_changed(self.on_stitch2d_slider)
        self.style_slider(self.stitch2d_slider)
        self.stitchdepth_slider_ax = self.fig.add_axes([0.68, 0.795, 0.20, 0.022])
        self.stitchdepth_slider = Slider(self.stitchdepth_slider_ax, "Split Depth", 1.0, 100.0, valinit=self.depth_threshold_cm, valstep=1.0)
        self.stitchdepth_slider.on_changed(self.on_stitchdepth_slider)
        self.style_slider(self.stitchdepth_slider)
        self.stitcharea_slider_ax = self.fig.add_axes([0.68, 0.750, 0.20, 0.022])
        self.stitcharea_slider = Slider(self.stitcharea_slider_ax, "Min Patch", 0.0000, 0.0200, valinit=self.stitch_min_area, valstep=0.0001)
        self.stitcharea_slider.on_changed(self.on_stitcharea_slider)
        self.style_slider(self.stitcharea_slider)
        self.knnradius_slider_ax = self.fig.add_axes([0.40, 0.705, 0.20, 0.022])
        self.knnradius_slider = Slider(self.knnradius_slider_ax, "KNN Radius", 0.05, 1.20, valinit=self.knn_radius, valstep=0.01)
        self.knnradius_slider.on_changed(self.on_knnradius_slider)
        self.style_slider(self.knnradius_slider)
        self.knnk_slider_ax = self.fig.add_axes([0.68, 0.705, 0.20, 0.022])
        self.knnk_slider = Slider(self.knnk_slider_ax, "K Neighbors", 2, 16, valinit=self.knn_neighbors, valstep=1)
        self.knnk_slider.on_changed(self.on_knnk_slider)
        self.style_slider(self.knnk_slider)
        self.voxel_slider_ax = self.fig.add_axes([0.12, 0.750, 0.20, 0.022])
        self.voxel_slider = Slider(self.voxel_slider_ax, "Az Res", 0.01, 1.00, valinit=self.world_map.azimuth_resolution_deg, valstep=0.01)
        self.voxel_slider.on_changed(self.on_voxel_slider)
        self.style_slider(self.voxel_slider)
        self.maxvox_slider_ax = self.fig.add_axes([0.40, 0.750, 0.20, 0.022])
        self.maxvox_slider = Slider(self.maxvox_slider_ax, "El Res", 0.01, 1.00, valinit=self.world_map.elevation_resolution_deg, valstep=0.01)
        self.maxvox_slider.on_changed(self.on_maxvox_slider)
        self.style_slider(self.maxvox_slider)
        self.nearspread_slider_ax = self.fig.add_axes([0.12, 0.705, 0.20, 0.022])
        self.nearspread_slider = Slider(self.nearspread_slider_ax, "Near Fill", 0.10, 4.00, valinit=self.world_map.near_spread_scale, valstep=0.05)
        self.nearspread_slider.on_changed(self.on_nearspread_slider)
        self.style_slider(self.nearspread_slider)
        self.midspread_slider_ax = self.fig.add_axes([0.40, 0.705, 0.20, 0.022])
        self.midspread_slider = Slider(self.midspread_slider_ax, "Mid Fill", 0.10, 4.00, valinit=self.world_map.mid_spread_scale, valstep=0.05)
        self.midspread_slider.on_changed(self.on_midspread_slider)
        self.style_slider(self.midspread_slider)
        self.farspread_slider_ax = self.fig.add_axes([0.68, 0.705, 0.20, 0.022])
        self.farspread_slider = Slider(self.farspread_slider_ax, "Far Fill", 0.10, 4.00, valinit=self.world_map.far_spread_scale, valstep=0.05)
        self.farspread_slider.on_changed(self.on_farspread_slider)
        self.style_slider(self.farspread_slider)
        self.nearband_slider_ax = self.fig.add_axes([0.12, 0.615, 0.20, 0.022])
        self.nearband_slider = Slider(self.nearband_slider_ax, "Near End", 0.20, 2.00, valinit=self.world_map.near_band_max, valstep=0.05)
        self.nearband_slider.on_changed(self.on_nearband_slider)
        self.style_slider(self.nearband_slider)
        self.midband_slider_ax = self.fig.add_axes([0.40, 0.615, 0.20, 0.022])
        self.midband_slider = Slider(self.midband_slider_ax, "Mid End", 0.50, 4.00, valinit=self.world_map.mid_band_max, valstep=0.05)
        self.midband_slider.on_changed(self.on_midband_slider)
        self.style_slider(self.midband_slider)
        self.midvoxmul_slider_ax = self.fig.add_axes([0.68, 0.615, 0.20, 0.022])
        self.midvoxmul_slider = Slider(self.midvoxmul_slider_ax, "Conn Range", 1, 8, valinit=self.connection_range_bins, valstep=1)
        self.midvoxmul_slider.on_changed(self.on_midvoxmul_slider)
        self.style_slider(self.midvoxmul_slider)
        self.farvoxmul_slider_ax = self.fig.add_axes([0.68, 0.570, 0.20, 0.022])
        self.farvoxmul_slider = Slider(self.farvoxmul_slider_ax, "Min Prob", 0.00, 1.00, valinit=self.min_render_probability, valstep=0.01)
        self.farvoxmul_slider.on_changed(self.on_farvoxmul_slider)
        self.style_slider(self.farvoxmul_slider)
        self.pointlife_slider_ax = self.fig.add_axes([0.40, 0.570, 0.20, 0.022])
        self.pointlife_slider = Slider(self.pointlife_slider_ax, "Point Life", 0.5, 30.0, valinit=self.world_map.decay_tau, valstep=0.5)
        self.pointlife_slider.on_changed(self.on_pointlife_slider)
        self.style_slider(self.pointlife_slider)
        self.nearcolor_slider_ax = self.fig.add_axes([0.12, 0.485, 0.20, 0.022])
        self.nearcolor_slider = Slider(self.nearcolor_slider_ax, "Near Tone", 0.00, 1.00, valinit=self.near_color_level, valstep=0.01)
        self.nearcolor_slider.on_changed(self.on_nearcolor_slider)
        self.style_slider(self.nearcolor_slider)
        self.farcolor_slider_ax = self.fig.add_axes([0.40, 0.485, 0.20, 0.022])
        self.farcolor_slider = Slider(self.farcolor_slider_ax, "Far Tone", 0.00, 1.00, valinit=self.far_color_level, valstep=0.01)
        self.farcolor_slider.on_changed(self.on_farcolor_slider)
        self.style_slider(self.farcolor_slider)
        self.whitepoints_check_ax = self.fig.add_axes([0.68, 0.438, 0.16, 0.085])
        self.whitepoints_check_ax.set_facecolor("white")
        self.whitepoints_check = CheckButtons(
            self.whitepoints_check_ax,
            ["White Start", "Triangulate", "Show All Pts"],
            [self.points_start_white, self.use_triangulation, self.show_all_fp_points],
        )
        if hasattr(self.whitepoints_check, "set_frame_props"):
            self.whitepoints_check.set_frame_props({
                "facecolor": (1.0, 1.0, 1.0, 1.0),
                "edgecolor": (0.20, 0.20, 0.20, 1.0),
            })
        elif hasattr(self.whitepoints_check, "rectangles"):
            for rect in self.whitepoints_check.rectangles:
                rect.set_facecolor((1.0, 1.0, 1.0, 1.0))
                rect.set_edgecolor((0.20, 0.20, 0.20, 1.0))
        if hasattr(self.whitepoints_check, "set_check_props"):
            self.whitepoints_check.set_check_props({
                "color": "black",
                "linewidth": 1.5,
            })
        elif hasattr(self.whitepoints_check, "lines"):
            for lines in self.whitepoints_check.lines:
                for line in lines:
                    line.set_color("black")
                    line.set_linewidth(1.5)
        for label in self.whitepoints_check.labels:
            label.set_color("black")
            label.set_fontsize(9)
        self.commonmult_slider_ax = self.fig.add_axes([0.12, 0.570, 0.20, 0.022])
        self.commonmult_slider = Slider(self.commonmult_slider_ax, "Bridge Limit", 1.00, 4.00, valinit=self.common_connect_mult, valstep=0.05)
        self.commonmult_slider.on_changed(self.on_commonmult_slider)
        self.style_slider(self.commonmult_slider)
        self.nearbias_slider_ax = self.fig.add_axes([0.12, 0.660, 0.20, 0.022])
        self.nearbias_slider = Slider(self.nearbias_slider_ax, "Near Priority", 0.10, 4.00, valinit=self.fp_distance_bias, valstep=0.05)
        self.nearbias_slider.on_changed(self.on_nearbias_slider)
        self.style_slider(self.nearbias_slider)
        self.fppoints_slider_ax = self.fig.add_axes([0.40, 0.660, 0.20, 0.022])
        self.fppoints_slider = Slider(self.fppoints_slider_ax, "View Points", 20, 1200, valinit=self.fp_max_render_points, valstep=10)
        self.fppoints_slider.on_changed(self.on_fppoints_slider)
        self.style_slider(self.fppoints_slider)
        self.dotsize_slider_ax = self.fig.add_axes([0.68, 0.660, 0.20, 0.022])
        self.dotsize_slider = Slider(self.dotsize_slider_ax, "Point Size", 0.00, 6.00, valinit=self.dot_size_scale, valstep=0.05)
        self.dotsize_slider.on_changed(self.on_dotsize_slider)
        self.style_slider(self.dotsize_slider)
        self.register_control_help(self.view_behind_ax, "World View: keep the map fixed in the world while the device moves inside it.")
        self.register_control_help(self.view_device_ax, "Device View: keep the device centered and show world points around it.")
        self.register_control_help(self.button_ax, "Calibrate for 5 seconds while the unit is still, flat, and front-face up. This removes bias/noise only.")
        self.register_control_help(self.record_ax, "Start or stop recording incoming sensor frames from the live serial stream.")
        self.register_control_help(self.play_ax, "Play the recorded frames in a loop. Live serial is ignored while playback is active.")
        self.register_control_help(self.learn_r_ax, "Learn which raw gyro axis is the device Right axis.")
        self.register_control_help(self.learn_f_ax, "Learn which raw gyro axis is the device Forward axis.")
        self.register_control_help(self.learn_u_ax, "Learn which raw gyro axis is the device Up axis.")
        self.register_control_help(self.flip_r_ax, "Flip the sign of the learned Right axis.")
        self.register_control_help(self.flip_f_ax, "Flip the sign of the learned Forward axis.")
        self.register_control_help(self.flip_u_ax, "Flip the sign of the learned Up axis.")
        self.register_control_help(self.dist_slider_ax, "Maximum distance shown in the first-person and 3D views.")
        self.register_control_help(self.stitch3d_slider_ax, "Maximum allowed point spacing for a stitched patch in the near range, as a fraction of the effective local sample spacing from ray spacing and sphere resolution.")
        self.register_control_help(self.stitchmid_slider_ax, "Maximum allowed point spacing for a stitched patch in the mid range, as a fraction of the effective local sample spacing from ray spacing and sphere resolution.")
        self.register_control_help(self.stitchfar_slider_ax, "Maximum allowed point spacing for a stitched patch in the far range, as a fraction of the effective local sample spacing from ray spacing and sphere resolution.")
        self.register_control_help(self.stitch2d_slider_ax, "Maximum allowed screen-space gap between stitched points after FOV compensation.")
        self.register_control_help(self.stitchdepth_slider_ax, "Depth difference in centimeters at which samples are treated as separate objects.")
        self.register_control_help(self.stitcharea_slider_ax, "Minimum visible patch area. Raise this to suppress tiny noisy surfaces.")
        self.register_control_help(self.knnradius_slider_ax, "Maximum projected distance from a point to its candidate KNN neighbors before a triangle can be proposed.")
        self.register_control_help(self.knnk_slider_ax, "How many nearest neighbors each visible point considers when building local KNN surface triangles.")
        self.register_control_help(self.voxel_slider_ax, "Azimuth resolution of the persistent sphere map in degrees. Smaller values give finer horizontal detail.")
        self.register_control_help(self.maxvox_slider_ax, "Elevation resolution of the persistent sphere map in degrees. Smaller values give finer vertical detail.")
        self.register_control_help(self.nearspread_slider_ax, "How much a near hit fills nearby angular bins on the sphere map.")
        self.register_control_help(self.midspread_slider_ax, "How much a mid-range hit fills nearby angular bins on the sphere map.")
        self.register_control_help(self.farspread_slider_ax, "How much a far hit fills nearby angular bins on the sphere map.")
        self.register_control_help(self.nearband_slider_ax, "Distance where the near sphere-fill band ends and the mid band begins.")
        self.register_control_help(self.midband_slider_ax, "Distance where the mid sphere-fill band ends and the far band begins.")
        self.register_control_help(self.midvoxmul_slider_ax, "How many neighboring sphere bins can belong to the same connected object in non-triangulated mode. Higher values connect across larger gaps.")
        self.register_control_help(self.farvoxmul_slider_ax, "Minimum stored point certainty required before a point or surface is rendered.")
        self.register_control_help(self.pointlife_slider_ax, "How many seconds points persist before fading out and disappearing.")
        self.register_control_help(self.nearcolor_slider_ax, "Brightness used for nearby objects in the distance colormap. 1.0 is white, 0.0 is black.")
        self.register_control_help(self.farcolor_slider_ax, "Brightness used for distant objects in the distance colormap. 1.0 is white, 0.0 is black.")
        self.register_control_help(self.whitepoints_check_ax, "White Start: new dots start white before fading. Triangulate: render first-person objects as local KNN-stitched surfaces instead of connected bin patches. Show All Pts: overlay every visible point in the first-person view instead of only connected objects.")
        self.register_control_help(self.commonmult_slider_ax, "Relaxed bridge limit for triangles that share a common point. 1.0 means no extra allowance.")
        self.register_control_help(self.nearbias_slider_ax, "How strongly the first-person renderer favors nearby points over distant ones.")
        self.register_control_help(self.fppoints_slider_ax, "Maximum number of map points kept for the first-person view.")
        self.register_control_help(self.dotsize_slider_ax, "Scales point size in both the 3D and first-person views.")
        self.fig.canvas.mpl_connect("motion_notify_event", self.on_hover_tooltip)
        self.fig.canvas.mpl_connect("close_event", self.on_close)
        self.whitepoints_check.on_clicked(self.on_whitepoints_toggle)
        self.triangulation_control_axes = [
            self.stitch3d_slider_ax,
            self.stitchmid_slider_ax,
            self.stitchfar_slider_ax,
            self.stitch2d_slider_ax,
            self.stitchdepth_slider_ax,
            self.stitcharea_slider_ax,
            self.commonmult_slider_ax,
            self.knnradius_slider_ax,
            self.knnk_slider_ax,
        ]
        self.update_control_visibility(redraw=False)

    def on_calibrate(self, _event):
        tick_ms = self.latest.get("tick_ms") if self.latest else 0
        self.calibration_active = True
        self.calibration_start_ms = tick_ms
        self.calibration_deadline_ms = tick_ms + CALIBRATION_DURATION_MS
        self.calibration_status = "collecting"
        self.calibration_samples = []
        self.calibration_stats = {}
        self.world_map.clear()
        self.clear_map_visual_cache()
        self.map_frame_rot = None

    def on_close(self, _event):
        if self.ser is not None:
            self.ser.close()
            self.ser = None

    def clear_map_visual_cache(self):
        self.fp_component_cache_sig = None
        self.fp_component_cache = None
        self.fp_triangulation_cache.clear()

    def layout_control_axes(self):
        tri_rows = [
            (self.stitch3d_slider_ax, [0.12, 0.795, 0.20, 0.022]),
            (self.stitch2d_slider_ax, [0.40, 0.795, 0.20, 0.022]),
            (self.stitchdepth_slider_ax, [0.68, 0.795, 0.20, 0.022]),
            (self.stitchmid_slider_ax, [0.12, 0.750, 0.20, 0.022]),
            (self.stitchfar_slider_ax, [0.40, 0.750, 0.20, 0.022]),
            (self.stitcharea_slider_ax, [0.68, 0.750, 0.20, 0.022]),
            (self.commonmult_slider_ax, [0.12, 0.705, 0.20, 0.022]),
            (self.knnradius_slider_ax, [0.40, 0.705, 0.20, 0.022]),
            (self.knnk_slider_ax, [0.68, 0.705, 0.20, 0.022]),
        ]
        base_positions = [
            (self.voxel_slider_ax, [0.12, 0.660, 0.20, 0.022]),
            (self.maxvox_slider_ax, [0.40, 0.660, 0.20, 0.022]),
            (self.midvoxmul_slider_ax, [0.68, 0.660, 0.20, 0.022]),
            (self.nearspread_slider_ax, [0.12, 0.615, 0.20, 0.022]),
            (self.midspread_slider_ax, [0.40, 0.615, 0.20, 0.022]),
            (self.farspread_slider_ax, [0.68, 0.615, 0.20, 0.022]),
            (self.nearbias_slider_ax, [0.12, 0.570, 0.20, 0.022]),
            (self.fppoints_slider_ax, [0.40, 0.570, 0.20, 0.022]),
            (self.dotsize_slider_ax, [0.68, 0.570, 0.20, 0.022]),
            (self.nearband_slider_ax, [0.12, 0.525, 0.20, 0.022]),
            (self.midband_slider_ax, [0.40, 0.525, 0.20, 0.022]),
            (self.farvoxmul_slider_ax, [0.68, 0.525, 0.20, 0.022]),
            (self.nearcolor_slider_ax, [0.12, 0.480, 0.20, 0.022]),
            (self.farcolor_slider_ax, [0.40, 0.480, 0.20, 0.022]),
            (self.pointlife_slider_ax, [0.68, 0.480, 0.20, 0.022]),
            (self.whitepoints_check_ax, [0.68, 0.420, 0.18, 0.100]),
        ]
        compact_positions = [
            (self.voxel_slider_ax, [0.12, 0.795, 0.20, 0.022]),
            (self.maxvox_slider_ax, [0.40, 0.795, 0.20, 0.022]),
            (self.midvoxmul_slider_ax, [0.68, 0.795, 0.20, 0.022]),
            (self.nearspread_slider_ax, [0.12, 0.750, 0.20, 0.022]),
            (self.midspread_slider_ax, [0.40, 0.750, 0.20, 0.022]),
            (self.farspread_slider_ax, [0.68, 0.750, 0.20, 0.022]),
            (self.nearbias_slider_ax, [0.12, 0.705, 0.20, 0.022]),
            (self.fppoints_slider_ax, [0.40, 0.705, 0.20, 0.022]),
            (self.dotsize_slider_ax, [0.68, 0.705, 0.20, 0.022]),
            (self.nearband_slider_ax, [0.12, 0.660, 0.20, 0.022]),
            (self.midband_slider_ax, [0.40, 0.660, 0.20, 0.022]),
            (self.farvoxmul_slider_ax, [0.68, 0.660, 0.20, 0.022]),
            (self.nearcolor_slider_ax, [0.12, 0.615, 0.20, 0.022]),
            (self.farcolor_slider_ax, [0.40, 0.615, 0.20, 0.022]),
            (self.pointlife_slider_ax, [0.68, 0.615, 0.20, 0.022]),
            (self.whitepoints_check_ax, [0.68, 0.545, 0.18, 0.100]),
        ]
        if self.use_triangulation:
            for ax, pos in tri_rows:
                ax.set_position(pos)
            for ax, pos in base_positions:
                ax.set_position(pos)
        else:
            for ax, pos in tri_rows:
                ax.set_position(pos)
            for ax, pos in compact_positions:
                ax.set_position(pos)

    def update_control_visibility(self, redraw=True):
        show_tri = bool(self.use_triangulation)
        self.layout_control_axes()
        for ax in self.triangulation_control_axes:
            ax.set_visible(show_tri)
        if redraw:
            self.fig.canvas.draw_idle()

    @staticmethod
    def style_slider(slider):
        slider.label.set_fontsize(9)
        slider.valtext.set_fontsize(9)
        slider.label.set_ha("left")
        slider.label.set_va("bottom")
        slider.label.set_position((0.0, 1.35))
        slider.valtext.set_ha("right")
        slider.valtext.set_va("bottom")
        slider.valtext.set_position((1.0, 1.35))

    def register_control_help(self, axes, text):
        self.control_help[axes] = text

    def on_hover_tooltip(self, event):
        message = self.control_help.get(event.inaxes)
        changed = False
        if message:
            if self.tooltip.get_text() != message:
                self.tooltip.set_text(message)
                changed = True
            if not self.tooltip.get_visible():
                self.tooltip.set_visible(True)
                changed = True
        else:
            if self.tooltip.get_visible():
                self.tooltip.set_visible(False)
                changed = True
        if changed:
            self.fig.canvas.draw_idle()

    def on_distance_slider(self, value):
        self.max_display_distance_m = float(value)

    def update_record_play_labels(self):
        self.record_button.label.set_text("Stop Rec" if self.recording_active else "Record")
        self.play_button.label.set_text("Stop Play" if self.playback_active else "Play Loop")
        self.fig.canvas.draw_idle()

    def auto_save_recording(self):
        if not self.recorded_payloads:
            self.last_recording_path = None
            self.last_recording_error = "nothing to save"
            return
        try:
            RECORDINGS_DIR.mkdir(parents=True, exist_ok=True)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            path = RECORDINGS_DIR / f"tof_clip_{stamp}.jsonl"
            with path.open("w", encoding="utf-8") as f:
                for payload in self.recorded_payloads:
                    f.write(json.dumps(payload, separators=(",", ":")))
                    f.write("\n")
            self.last_recording_path = str(path)
            self.last_recording_error = None
        except OSError as exc:
            self.last_recording_path = None
            self.last_recording_error = str(exc)

    def on_record_toggle(self, _event):
        if self.recording_active:
            self.recording_active = False
            self.auto_save_recording()
        else:
            self.recorded_payloads = []
            self.last_recording_path = None
            self.last_recording_error = None
            self.recording_active = True
            self.playback_active = False
            self.playback_start_perf = None
            self.playback_loop_duration_ms = 0
            self.playback_index = 0
        self.update_record_play_labels()

    def on_play_toggle(self, _event):
        if self.playback_active:
            self.playback_active = False
            self.playback_start_perf = None
            self.playback_index = 0
            self.update_record_play_labels()
            return
        if not self.recorded_payloads:
            return
        self.recording_active = False
        self.playback_active = True
        self.playback_start_perf = time.perf_counter()
        self.playback_index = 0
        first_tick = self.recorded_payloads[0].get("tick_ms", 0)
        last_tick = self.recorded_payloads[-1].get("tick_ms", first_tick)
        self.playback_loop_duration_ms = max(1, int(last_tick) - int(first_tick))
        self.world_map.clear()
        self.clear_map_visual_cache()
        self.map_frame_rot = None
        self.update_state_from_payload(copy.deepcopy(self.recorded_payloads[0]))
        self.update_record_play_labels()

    def on_stitch3d_slider(self, value):
        self.stitch_span_near = float(value)

    def on_stitchmid_slider(self, value):
        self.stitch_span_mid = float(value)

    def on_stitchfar_slider(self, value):
        self.stitch_span_far = float(value)

    def on_stitch2d_slider(self, value):
        self.stitch_view_gap_frac = float(value)

    def on_stitchdepth_slider(self, value):
        self.depth_threshold_cm = float(value)

    def on_stitcharea_slider(self, value):
        self.stitch_min_area = float(value)

    def on_knnradius_slider(self, value):
        self.knn_radius = float(value)
        self.fp_triangulation_cache.clear()

    def on_knnk_slider(self, value):
        self.knn_neighbors = max(2, int(round(float(value))))
        self.knnk_slider.valtext.set_text(f"{self.knn_neighbors}")
        self.fp_triangulation_cache.clear()

    def on_voxel_slider(self, value):
        self.world_map.set_azimuth_resolution(float(value))
        self.fp_component_cache_sig = None
        self.fp_component_cache = None
        self.fp_triangulation_cache.clear()

    def on_maxvox_slider(self, value):
        self.world_map.set_elevation_resolution(float(value))
        self.fp_component_cache_sig = None
        self.fp_component_cache = None
        self.fp_triangulation_cache.clear()

    def on_nearspread_slider(self, value):
        self.world_map.near_spread_scale = float(value)
        self.world_map._rebuild_kernels()
        self.world_map.clear()
        self.clear_map_visual_cache()

    def on_midspread_slider(self, value):
        self.world_map.mid_spread_scale = float(value)
        self.world_map._rebuild_kernels()
        self.world_map.clear()
        self.clear_map_visual_cache()

    def on_farspread_slider(self, value):
        self.world_map.far_spread_scale = float(value)
        self.world_map._rebuild_kernels()
        self.world_map.clear()
        self.clear_map_visual_cache()

    def on_nearband_slider(self, value):
        self.world_map.near_band_max = min(float(value), self.world_map.mid_band_max - 0.05)
        self.nearband_slider.valtext.set_text(f"{self.world_map.near_band_max:.2f}")
        self.world_map._rebuild_kernels()
        self.world_map.clear()
        self.clear_map_visual_cache()

    def on_midband_slider(self, value):
        self.world_map.mid_band_max = max(float(value), self.world_map.near_band_max + 0.05)
        self.midband_slider.valtext.set_text(f"{self.world_map.mid_band_max:.2f}")
        self.world_map._rebuild_kernels()
        self.world_map.clear()
        self.clear_map_visual_cache()

    def on_midvoxmul_slider(self, value):
        self.connection_range_bins = max(1, int(round(float(value))))
        self.midvoxmul_slider.valtext.set_text(f"{self.connection_range_bins}")

    def on_farvoxmul_slider(self, value):
        self.min_render_probability = float(value)

    def stitch_span_for_distance(self, dist_m):
        dist = np.asarray(dist_m, dtype=float)
        return np.where(
            dist < self.world_map.near_band_max,
            self.stitch_span_near,
            np.where(dist < self.world_map.mid_band_max, self.stitch_span_mid, self.stitch_span_far),
        )

    def on_pointlife_slider(self, value):
        self.world_map.decay_tau = float(value)

    def on_nearcolor_slider(self, value):
        self.near_color_level = float(value)

    def on_farcolor_slider(self, value):
        self.far_color_level = float(value)

    def on_whitepoints_toggle(self, _label):
        status = self.whitepoints_check.get_status()
        self.points_start_white = bool(status[0])
        self.use_triangulation = bool(status[1])
        self.show_all_fp_points = bool(status[2])
        self.update_control_visibility()

    def on_commonmult_slider(self, value):
        self.common_connect_mult = float(value)

    def on_nearbias_slider(self, value):
        self.fp_distance_bias = float(value)

    def on_fppoints_slider(self, value):
        self.fp_max_render_points = int(round(float(value)))

    def on_dotsize_slider(self, value):
        self.dot_size_scale = float(value)

    def distance_color_rgba(self, distances, alpha=1.0):
        return distance_gray_rgba(
            distances,
            self.max_display_distance_m,
            min_distance=TOF_MIN_RANGE_M,
            alpha=alpha,
            near_level=self.near_color_level,
            far_level=self.far_color_level,
        )

    def ttl_point_color_rgba(self, distances, weights, alpha=1.0):
        return ttl_color_rgba(
            distances,
            self.max_display_distance_m,
            weights,
            self.world_map.decay_tau,
            MAP_MIN_WEIGHT,
            alpha=alpha,
            start_white=self.points_start_white,
            near_level=self.near_color_level,
            far_level=self.far_color_level,
        )

    def on_view_behind(self, _event):
        self.view_mode = "behind"

    def on_view_device(self, _event):
        self.view_mode = "device"

    def on_learn_axis(self, app_axis_idx):
        tick_ms = self.latest.get("tick_ms") if self.latest else 0
        self.axis_mapper.start_capture(app_axis_idx, tick_ms)
        log_axis_mapping(f"start learn {self.axis_mapper.APP_AXIS_LABELS[app_axis_idx]}", self.axis_mapper)

    def on_flip_axis(self, app_axis_idx):
        self.axis_mapper.flip(app_axis_idx)
        log_axis_mapping(f"flip {self.axis_mapper.APP_AXIS_LABELS[app_axis_idx]}", self.axis_mapper)
        self.world_map.clear()
        self.clear_map_visual_cache()
        self.map_frame_rot = None
        tick_ms = self.latest.get("tick_ms") if self.latest else 0
        self.pose.calibrate(tick_ms=tick_ms)
        self.calibration_status = "axis sign changed; calibrate 5s"

    @staticmethod
    def get_primary_motion_raw_from_payload(payload):
        motion = payload.get("motion", {})
        return motion.get("lsm6dsv16x_acc_mg"), motion.get("lsm6dsv16x_gyro_mdps"), motion.get("lis2mdl_mag_mgauss")

    def get_primary_motion(self):
        acc_raw, gyro_raw, mag_raw = self.get_primary_motion_raw_from_payload(self.latest)
        return self.axis_mapper.transform(acc_raw), self.axis_mapper.transform(gyro_raw), self.axis_mapper.transform(mag_raw)

    def read_frames(self):
        if self.playback_active or self.ser is None:
            return

        try:
            pending = self.ser.in_waiting
            if pending > SERIAL_BACKLOG_DROP_BYTES:
                self.ser.reset_input_buffer()
                self.rx_buffer = ""
                self.serial_backlog_drops += 1
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
            try:
                if not line.startswith("{"):
                    continue
                payload = json.loads(line)
            except json.JSONDecodeError:
                continue

            if ("frame" not in payload) and ("frames" not in payload):
                continue

            if self.recording_active:
                self.recorded_payloads.append(copy.deepcopy(payload))
            self.process_payload(payload)

    def playback_step(self):
        if not self.playback_active or not self.recorded_payloads:
            return
        if len(self.recorded_payloads) == 1:
            payload = copy.deepcopy(self.recorded_payloads[0])
            self.process_payload(payload)
            return
        if self.playback_start_perf is None:
            self.playback_start_perf = time.perf_counter()
        first_tick = int(self.recorded_payloads[0].get("tick_ms", 0))
        elapsed_ms = int((time.perf_counter() - self.playback_start_perf) * 1000.0)
        loop_elapsed_ms = elapsed_ms % max(1, self.playback_loop_duration_ms)
        target_tick = first_tick + loop_elapsed_ms
        while (
            self.playback_index + 1 < len(self.recorded_payloads)
            and int(self.recorded_payloads[self.playback_index + 1].get("tick_ms", first_tick)) <= target_tick
        ):
            self.playback_index += 1
        if target_tick < int(self.recorded_payloads[self.playback_index].get("tick_ms", first_tick)):
            self.playback_index = 0
        payload = copy.deepcopy(self.recorded_payloads[self.playback_index])
        self.process_payload(payload)

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

    def process_payload(self, payload):
        expanded = self.expand_payload(payload)
        if not expanded:
            return
        for subpayload in expanded:
            self.latest = subpayload
            self.update_state_from_payload(subpayload)

    def update_state_from_payload(self, payload):
        tof = payload.get("tof", {})
        dist = tof.get("dist_mm", [])
        if isinstance(dist, list) and len(dist) == TOF_SIZE * TOF_SIZE:
            arr = np.array(
                [np.nan if (v is None or float(v) < 0) else float(v) for v in dist],
                dtype=float,
            )
            self.frame = arr.reshape((TOF_SIZE, TOF_SIZE))

        acc_raw, gyro_raw, mag_raw = self.get_primary_motion_raw_from_payload(payload)
        axis_changed = self.axis_mapper.update_capture(payload.get("tick_ms"), gyro_raw)
        if axis_changed:
            log_axis_mapping("capture complete", self.axis_mapper)
            self.world_map.clear()
            self.clear_map_visual_cache()
            self.map_frame_rot = None
            self.pose.calibrate(tick_ms=payload.get("tick_ms"))
            self.calibration_status = "axis map updated; calibrate 5s"

        acc_xyz = self.axis_mapper.transform(acc_raw)
        gyro_xyz = self.axis_mapper.transform(gyro_raw)
        mag_xyz = self.axis_mapper.transform(mag_raw)
        if acc_xyz is not None:
            self.cal_acc_samples.append(acc_xyz)
        if gyro_xyz is not None:
            self.cal_gyro_samples.append(gyro_xyz)
        if mag_xyz is not None:
            self.cal_mag_samples.append(mag_xyz)
        self.update_calibration_samples(payload.get("tick_ms"), acc_xyz, gyro_xyz, mag_xyz)
        self.pose.update_orientation(acc_xyz, gyro_xyz, mag_xyz, payload.get("tick_ms"))
        self.update_world_map(payload.get("tick_ms"))

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
        progress = float(np.clip(elapsed / CALIBRATION_DURATION_MS, 0.0, 1.0))
        self.calibration_stats = {
            "elapsed_ms": elapsed,
            "progress": progress,
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
        self.world_map.clear()
        self.clear_map_visual_cache()
        self.map_frame_rot = None
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
            "pose_match": pose_match,
        }

    def update_world_map(self, tick_ms):
        rot_current = rotation_matrix(self.pose.roll, self.pose.pitch, self.pose.yaw)
        if self.map_frame_rot is None:
            self.map_frame_rot = rot_current
        local_points = project_tof_to_local_points(self.frame)
        if local_points.size == 0:
            return
        points_from_device = local_points + DEVICE_SENSOR_FACE_LOCAL
        points_anchor = points_from_device @ rot_current.T @ self.map_frame_rot
        self.world_map.add_points(points_anchor, tick_ms=tick_ms)

    @staticmethod
    def first_valid(*values):
        for value in values:
            if value is not None:
                return value
        return None

    def render_text(self):
        payload = self.latest
        cal_lines = self.render_calibration_lines()
        axis_lines = self.render_axis_lines()
        playback_mode = "playback" if self.playback_active else "live"
        record_state = "recording" if self.recording_active else "idle"
        save_line = f"  saved clip   : {self.last_recording_path}" if self.last_recording_path else (
            f"  save err     : {self.last_recording_error}" if self.last_recording_error else "  saved clip   : -"
        )
        if not payload:
            if self.serial_error is not None:
                self.text.set_text(
                    "Serial error\n"
                    f"  port : {PORT}\n"
                    f"  baud : {BAUD}\n"
                    f"  err  : {self.serial_error}\n"
                    f"  mode : {playback_mode}\n"
                    f"  rec  : {record_state}\n"
                    f"{save_line}\n"
                )
            else:
                self.text.set_text(
                    "Waiting for frames...\n"
                    f"  port : {PORT}\n"
                    f"  baud : {BAUD}\n"
                    f"  mode : {playback_mode}\n"
                    f"  rec  : {record_state}\n"
                    f"  clip : {len(self.recorded_payloads)} frames\n"
                    f"{save_line}\n"
                    "\n".join([""] + axis_lines + [""] + cal_lines)
                )
            return

        present = payload.get("present", {})
        env = payload.get("env", {})
        motion = payload.get("motion", {})
        tof = payload.get("tof", {})

        roll_deg = math.degrees(self.pose.roll)
        pitch_deg = math.degrees(self.pose.pitch)
        yaw_deg = math.degrees(self.pose.yaw)
        az_res_deg, el_res_deg = self.world_map.angular_resolution_summary()

        lines = [
            f"frame          : {payload.get('frame')}",
            f"tick_ms        : {payload.get('tick_ms')}",
            "",
            *axis_lines,
            "",
            *cal_lines,
            "",
            "pose",
            f"  origin       : x={self.pose.pos[0]: .2f} y={self.pose.pos[1]: .2f} z={self.pose.pos[2]: .2f}",
            f"  lin acc      : {fmt_xyz((self.pose.linear_acc_world * 1000.0).round(1).tolist())}",
            f"  rot R/F      : {roll_deg: .1f} / {pitch_deg: .1f} deg",
            f"  rot U        : {yaw_deg: .1f} deg",
            f"  calibrated   : {int(self.pose.calibrated)}",
            f"  stationary   : {int(self.pose.stationary)}",
            f"  gyro bias    : {fmt_xyz((self.pose.gyro_bias * 1000.0).round(1).tolist())}",
            f"  acc sens bias: {fmt_xyz((self.pose.accel_sensor_bias * 1000.0).round(1).tolist())}",
            f"  accel bias   : {fmt_xyz((self.pose.accel_world_bias * 1000.0).round(1).tolist())}",
            f"  acc noise mg : {fmt_xyz((self.pose.accel_noise_std * 1000.0).round(1).tolist())}",
            f"  gyro noise   : {fmt_xyz((self.pose.gyro_noise_std * 1000.0).round(1).tolist())}",
            f"  deadband mg  : {self.pose.accel_deadband_g * 1000.0: .1f}",
            "",
            "map",
            "  mode         : egocentric first-person",
            f"  source       : {playback_mode}",
            f"  record       : {record_state}",
            f"  clip frames  : {len(self.recorded_payloads)}",
            save_line,
            f"  3d view      : {self.view_mode}",
            f"  tof diag fov : {TOF_DIAGONAL_FOV_DEG: .1f} deg",
            f"  sphere bins  : {self.world_map.active_count()}",
            f"  max display  : {self.max_display_distance_m: .2f} m",
            f"  az/el res    : {az_res_deg: .2f} / {el_res_deg: .2f} deg",
            f"  near/mid end : {self.world_map.near_band_max: .2f} / {self.world_map.mid_band_max: .2f} m",
            f"  lifetime tau : {self.world_map.decay_tau: .1f} s",
            f"  backlog drops: {self.serial_backlog_drops}",
            f"  near fill    : {self.world_map.near_spread_scale: .2f}x",
            f"  mid fill     : {self.world_map.mid_spread_scale: .2f}x",
            f"  far fill     : {self.world_map.far_spread_scale: .2f}x",
            f"  near prio    : {self.fp_distance_bias: .2f}",
            f"  view points  : {self.fp_max_render_points}",
            f"  point size   : {self.dot_size_scale: .2f}x",
            f"  min prob     : {self.min_render_probability: .2f}",
            f"  near/far tone: {self.near_color_level: .2f} / {self.far_color_level: .2f}",
            f"  white start  : {int(self.points_start_white)}",
            f"  show all pts : {int(self.show_all_fp_points)}",
            f"  fp mode      : {'knn surfaces' if self.use_triangulation else 'connected bins'}",
            f"  conn range   : {self.connection_range_bins} bin",
            f"  knn k/r      : {self.knn_neighbors} / {self.knn_radius: .2f}",
            f"  span n/m/f   : {self.stitch_span_near: .2f} / {self.stitch_span_mid: .2f} / {self.stitch_span_far: .2f}",
            f"  screen gap   : {self.stitch_view_gap_frac: .2f}",
            f"  split depth  : {self.depth_threshold_cm: .0f} cm",
            f"  min patch    : {self.stitch_min_area: .4f}",
            f"  bridge limit : {self.common_connect_mult: .2f}x",
            "",
            "present",
            f"  tof          : {present.get('tof')}",
            f"  lsm6dsv16x   : {present.get('lsm6dsv16x')}",
            f"  lsm6dso16is  : {present.get('lsm6dso16is')}",
            f"  lis2duxs12   : {present.get('lis2duxs12')}",
            f"  lis2mdl      : {present.get('lis2mdl')}",
            "",
            "environment",
            f"  sht40 temp   : {env.get('sht40_temp_c')}",
            f"  sht40 rh     : {env.get('sht40_rh')}",
            f"  lps22 temp   : {env.get('lps22_temp_c')}",
            f"  pressure     : {env.get('lps22_press_hpa')}",
            f"  stts22 temp  : {env.get('stts22_temp_c')}",
            "",
            "motion xyz",
            f"  dsv16x acc   : {fmt_xyz(motion.get('lsm6dsv16x_acc_mg'))}",
            f"  dsv16x gyro  : {fmt_xyz(motion.get('lsm6dsv16x_gyro_mdps'))}",
            f"  lis2mdl mag  : {fmt_xyz(motion.get('lis2mdl_mag_mgauss'))}",
            "",
            f"tof ok         : {tof.get('ok')}",
        ]
        self.text.set_text("\n".join(lines))

    def render_axis_lines(self):
        lines = [
            "axes",
            f"  map          : {self.axis_mapper.mapping_text()}",
            f"  status       : {self.axis_mapper.status}",
        ]
        progress = self.axis_mapper.capture_progress(self.latest.get("tick_ms") if self.latest else None)
        if progress is not None and self.axis_mapper.capture_axis is not None:
            lines.append(
                f"  learning     : {self.axis_mapper.APP_AXIS_LABELS[self.axis_mapper.capture_axis]} {100.0 * progress: .0f}%"
            )
        else:
            lines.append("  controls     : Learn/Flip R,F,U")
        return lines

    def render_calibration_lines(self):
        stats = self.calibration_stats or {}
        lines = ["calibration", f"  status       : {self.calibration_status}"]
        lines.append("  pose         : flat, still, front face up")
        if self.calibration_active:
            elapsed_s = stats.get("elapsed_ms", 0) / 1000.0
            progress_pct = 100.0 * stats.get("progress", 0.0)
            lines.extend(
                [
                    f"  elapsed      : {elapsed_s: .1f} / 5.0 s",
                    f"  progress     : {progress_pct: .0f}%",
                    f"  still samples: {stats.get('still_count', 0)} / {stats.get('total_count', 0)}",
                ]
            )
        elif stats:
            still_ratio = 100.0 * stats.get("still_ratio", 0.0)
            lines.extend(
                [
                    f"  still ratio  : {still_ratio: .0f}%",
                    f"  used samples : a={stats.get('acc_used', 0)} g={stats.get('gyro_used', 0)} m={stats.get('mag_used', 0)}",
                ]
            )
        else:
            lines.append("  hint         : calibration removes bias/noise only")
        return lines

    def render_first_person_view(self):
        self.ax_fp.cla()
        self.ax_fp.set_title("First-Person View")
        self.ax_fp.set_xlim(-1.1, 1.1)
        self.ax_fp.set_ylim(-0.9, 0.9)
        self.ax_fp.set_facecolor(FP_BG_COLOR)
        self.ax_fp.grid(True, alpha=0.12, color="black")
        self.ax_fp.axhline(0.0, color="black", lw=0.8, alpha=0.35)
        self.ax_fp.axvline(0.0, color="black", lw=0.8, alpha=0.35)
        self.ax_fp.set_xticks([])
        self.ax_fp.set_yticks([])
        nearest = np.full(8, np.inf, dtype=float)

        rot_current = rotation_matrix(self.pose.roll, self.pose.pitch, self.pose.yaw)
        current_tick_ms = self.latest.get("tick_ms") if self.latest else None
        current_points_local = project_tof_to_local_points(self.frame)
        if current_points_local.size:
            current_points_local = current_points_local + DEVICE_SENSOR_FACE_LOCAL
            current_distances = np.linalg.norm(current_points_local, axis=1)
            current_visible = (current_distances <= self.max_display_distance_m) & (current_points_local[:, 1] > 0.15)
            current_points_local = current_points_local[current_visible]
            current_distances = current_distances[current_visible]
        else:
            current_distances = np.empty((0,), dtype=float)

        points, weights, keys, distances = self.world_map.query_local_points(
            rot_current,
            self.map_frame_rot,
            self.max_display_distance_m,
            tick_ms=current_tick_ms,
            min_forward=0.15,
            max_points=self.fp_max_render_points,
            score_bias=self.fp_distance_bias,
            min_probability=self.min_render_probability,
        )
        all_points, all_weights, _all_keys, all_distances = self.world_map.query_local_points(
            rot_current,
            self.map_frame_rot,
            self.max_display_distance_m,
            tick_ms=current_tick_ms,
            min_forward=0.15,
            max_points=None,
            score_bias=self.fp_distance_bias,
            min_probability=self.min_render_probability,
        )
        if points.size:
            forward = np.clip(points[:, 1], 0.15, 4.0)
            proj_x = np.clip(points[:, 0] / forward, -1.2, 1.2)
            proj_y = np.clip(points[:, 2] / forward, -1.0, 1.0)

            sort_idx = np.lexsort((proj_x, proj_y))
            points = points[sort_idx]
            weights = weights[sort_idx]
            forward = forward[sort_idx]
            proj_x = proj_x[sort_idx]
            proj_y = proj_y[sort_idx]
            distances = distances[sort_idx]
            keys = [keys[idx] for idx in sort_idx]

            sample_spacing = self.world_map.sample_spacing_for_distance(distances)

            half_span = np.clip(0.5 * sample_spacing / np.clip(forward, 0.15, 4.0), 0.004, 0.090)
            patch_quads = np.stack(
                [
                    np.column_stack([proj_x - half_span, proj_y - half_span]),
                    np.column_stack([proj_x + half_span, proj_y - half_span]),
                    np.column_stack([proj_x + half_span, proj_y + half_span]),
                    np.column_stack([proj_x - half_span, proj_y + half_span]),
                ],
                axis=1,
            )
        else:
            forward = np.empty((0,), dtype=float)
            proj_x = np.empty((0,), dtype=float)
            proj_y = np.empty((0,), dtype=float)
            patch_quads = np.empty((0, 4, 2), dtype=float)

        component_scores = []
        if points.size:
            key_to_sorted_idx = {key: idx for idx, key in enumerate(keys)}
            component_sig = (self.connection_range_bins, tuple(sorted(keys)))
            if component_sig == self.fp_component_cache_sig and self.fp_component_cache is not None:
                components = self.fp_component_cache
            else:
                raw_components = self.world_map.connected_components_26(keys, connect_range=self.connection_range_bins)
                components = [tuple(sorted(component)) for component in raw_components if component]
                self.fp_component_cache_sig = component_sig
                self.fp_component_cache = components
            for component in components:
                indices = [key_to_sorted_idx[key] for key in component if key in key_to_sorted_idx]
                if not indices:
                    continue
                comp_idx = np.asarray(indices, dtype=int)
                comp_score = float(np.sum(weights[comp_idx] / np.clip(distances[comp_idx], 0.25, self.max_display_distance_m)))
                component_scores.append((comp_score, comp_idx))
            component_scores.sort(key=lambda item: item[0], reverse=True)

        surface_count = 0
        if not self.use_triangulation:
            for _cluster_score, comp_idx in component_scores:
                if surface_count >= FP_MAX_SURFACES or comp_idx.size < 1:
                    continue
                cluster_quads = patch_quads[comp_idx]
                cluster_weights = weights[comp_idx]
                cluster_distances = distances[comp_idx]
                cluster_colors = self.distance_color_rgba(
                    cluster_distances,
                    alpha=np.clip(0.14 + 0.50 * cluster_weights, 0.14, 0.58),
                )
                cluster_collection = PolyCollection(
                    cluster_quads,
                    closed=True,
                    facecolors=cluster_colors,
                    edgecolors="none",
                )
                self.ax_fp.add_collection(cluster_collection)
                surface_count += len(cluster_quads)
        else:
            tri_cache_key = (
                "knn",
                tuple(sorted(keys)),
                self.knn_neighbors,
                round(self.knn_radius, 3),
                round(self.pose.roll, 4),
                round(self.pose.pitch, 4),
                round(self.pose.yaw, 4),
            )
            cached_tri = self.fp_triangulation_cache.get(tri_cache_key)
            if cached_tri is None:
                tri_source_idx = np.arange(len(proj_x), dtype=int)
                tri_triangles = knn_fp_triangles(proj_x, proj_y, k=self.knn_neighbors, max_radius=self.knn_radius)
                self.fp_triangulation_cache.clear()
                self.fp_triangulation_cache[tri_cache_key] = (tri_source_idx, tri_triangles)
            else:
                tri_source_idx, tri_triangles = cached_tri

            if tri_source_idx.size and tri_triangles.size:
                proj_xy = np.stack([proj_x, proj_y], axis=1)[tri_source_idx]
                tri_idx = tri_source_idx[tri_triangles]
                tri_proj = proj_xy[tri_triangles]
                tri_points = points[tri_idx]
                tri_forward = forward[tri_idx]
                tri_weights = weights[tri_idx]
                tri_distances = distances[tri_idx]

                edge3_a = np.linalg.norm(tri_points[:, 0] - tri_points[:, 1], axis=1)
                edge3_b = np.linalg.norm(tri_points[:, 1] - tri_points[:, 2], axis=1)
                edge3_c = np.linalg.norm(tri_points[:, 2] - tri_points[:, 0], axis=1)

                edge2_a = np.linalg.norm(tri_proj[:, 0] - tri_proj[:, 1], axis=1)
                edge2_b = np.linalg.norm(tri_proj[:, 1] - tri_proj[:, 2], axis=1)
                edge2_c = np.linalg.norm(tri_proj[:, 2] - tri_proj[:, 0], axis=1)

                tri_mean_forward = np.mean(tri_forward, axis=1)
                tri_depth_span = np.max(tri_forward, axis=1) - np.min(tri_forward, axis=1)
                tri_mean_weight = np.mean(tri_weights, axis=1)
                tri_mean_distance = np.mean(tri_distances, axis=1)
                tri_area = 0.5 * np.abs(
                    (tri_proj[:, 1, 0] - tri_proj[:, 0, 0]) * (tri_proj[:, 2, 1] - tri_proj[:, 0, 1])
                    - (tri_proj[:, 1, 1] - tri_proj[:, 0, 1]) * (tri_proj[:, 2, 0] - tri_proj[:, 0, 0])
                )
                arc_gain = fp_arc_spread_gain(tri_proj, tri_mean_forward)
                ray_spacing = self.world_map._ray_spacing_m(tri_mean_forward)
                angular_spacing = self.world_map.sample_spacing_for_distance(tri_mean_forward)
                local_sample_spacing = np.maximum(ray_spacing, angular_spacing)
                span_frac = self.stitch_span_for_distance(tri_mean_forward)
                edge3_limit = np.clip(span_frac * local_sample_spacing, 0.008, 2.50)
                edge2_limit = np.clip(self.stitch_view_gap_frac * (2.0 * math.tan(TOF_AXIS_HALF_RAD)) * arc_gain, 0.02, 1.20)
                depth_limit = np.clip(self.depth_threshold_cm / 100.0, 0.01, 1.00)
                bridge_edge3_limit = np.clip(self.common_connect_mult * edge3_limit, 0.02, 6.00)

                edge_ab_ok = (edge3_a < edge3_limit) & (edge2_a < edge2_limit)
                edge_bc_ok = (edge3_b < edge3_limit) & (edge2_b < edge2_limit)
                edge_ca_ok = (edge3_c < edge3_limit) & (edge2_c < edge2_limit)
                edge_ab_bridge = edge3_a < bridge_edge3_limit
                edge_bc_bridge = edge3_b < bridge_edge3_limit
                edge_ca_bridge = edge3_c < bridge_edge3_limit

                strict_connect = edge_ab_ok & edge_bc_ok & edge_ca_ok
                common_connect = (
                    (edge_ab_ok & edge_ca_ok & edge_bc_bridge)
                    | (edge_ab_ok & edge_bc_ok & edge_ca_bridge)
                    | (edge_bc_ok & edge_ca_ok & edge_ab_bridge)
                )
                connectivity_ok = strict_connect | common_connect
                tri_valid = connectivity_ok & (tri_depth_span < depth_limit) & (tri_area > self.stitch_min_area)

                if np.any(tri_valid):
                    tri_score = tri_mean_weight[tri_valid] / np.clip(tri_mean_distance[tri_valid], 0.25, self.max_display_distance_m)
                    keep_order = np.argsort(tri_score)[::-1][:FP_MAX_SURFACES]
                    kept_triangles = tri_triangles[tri_valid][keep_order]
                    tri_proj = tri_proj[tri_valid][keep_order]
                    tri_mean_weight = tri_mean_weight[tri_valid][keep_order]
                    tri_mean_distance = tri_mean_distance[tri_valid][keep_order]
                    tri_facecolors = self.distance_color_rgba(
                        tri_mean_distance,
                        alpha=np.clip(0.08 + 0.42 * tri_mean_weight, 0.08, 0.40),
                    )
                    tri_collection = PolyCollection(
                        tri_proj,
                        closed=True,
                        facecolors=tri_facecolors,
                        edgecolors="none",
                    )
                    self.ax_fp.add_collection(tri_collection)

                    boundary_edges = {}
                    for tri_vertices in kept_triangles:
                        for a, b in ((0, 1), (1, 2), (2, 0)):
                            i = int(tri_vertices[a])
                            j = int(tri_vertices[b])
                            key = (i, j) if i < j else (j, i)
                            boundary_edges[key] = boundary_edges.get(key, 0) + 1

                    outline_segments = []
                    for (i, j), count in boundary_edges.items():
                        if count != 1:
                            continue
                        outline_segments.append([proj_xy[i], proj_xy[j]])

                    if outline_segments:
                        outline_colors = np.ones((len(outline_segments), 4), dtype=float)
                        outline_colors[:, 3] = 0.80
                        outline_widths = np.full(len(outline_segments), 0.55, dtype=float)
                        outline_collection = LineCollection(
                            outline_segments,
                            colors=outline_colors,
                            linewidths=outline_widths,
                        )
                        self.ax_fp.add_collection(outline_collection)

        if self.show_all_fp_points and all_points.size:
            all_forward = np.clip(all_points[:, 1], 0.15, 4.0)
            all_proj_x = np.clip(all_points[:, 0] / all_forward, -1.2, 1.2)
            all_proj_y = np.clip(all_points[:, 2] / all_forward, -1.0, 1.0)
            colors = self.ttl_point_color_rgba(
                all_distances,
                all_weights,
                alpha=np.clip(0.28 + 0.70 * all_weights, 0.28, 0.95),
            )
            sizes = np.maximum(
                1.5,
                self.dot_size_scale * ((26.0 / np.clip(all_distances, 0.2, 4.0)) + 16.0 * all_weights),
            )
            self.ax_fp.scatter(all_proj_x, all_proj_y, s=sizes, c=colors, linewidths=0.0)
        if self.show_all_fp_points and current_points_local.size:
            current_forward = np.clip(current_points_local[:, 1], 0.15, 4.0)
            current_proj_x = np.clip(current_points_local[:, 0] / current_forward, -1.2, 1.2)
            current_proj_y = np.clip(current_points_local[:, 2] / current_forward, -1.0, 1.0)
            current_colors = self.distance_color_rgba(
                current_distances,
                alpha=np.full(current_distances.shape, 0.95, dtype=float),
            )
            current_sizes = np.maximum(
                8.0,
                self.dot_size_scale * (34.0 / np.clip(current_distances, 0.2, 4.0) + 22.0),
            )
            self.ax_fp.scatter(current_proj_x, current_proj_y, s=current_sizes, c=current_colors, linewidths=0.0)

        hist_edges = np.linspace(-1.0, 1.0, 9)
        nearest = np.full(8, np.inf, dtype=float)
        if all_points.size:
            hist_forward = np.clip(all_points[:, 1], 0.15, 4.0)
            hist_proj_x = np.clip(all_points[:, 0] / hist_forward, -1.2, 1.2)
            hist_bins = np.clip(np.digitize(hist_proj_x, hist_edges) - 1, 0, 7)
            for idx in range(8):
                mask = hist_bins == idx
                if np.any(mask):
                    nearest[idx] = min(nearest[idx], float(np.min(all_distances[mask])))
        if current_points_local.size:
            current_forward = np.clip(current_points_local[:, 1], 0.15, 4.0)
            current_proj_x = np.clip(current_points_local[:, 0] / current_forward, -1.2, 1.2)
            current_bins = np.clip(np.digitize(current_proj_x, hist_edges) - 1, 0, 7)
            for idx in range(8):
                mask = current_bins == idx
                if np.any(mask):
                    nearest[idx] = min(nearest[idx], float(np.min(current_distances[mask])))

        self.ax_fp.text(0.02, 0.96, "near", color="black", transform=self.ax_fp.transAxes, fontsize=8, va="top")
        self.ax_fp.text(0.50, 0.02, "forward", color="black", transform=self.ax_fp.transAxes, fontsize=8, ha="center")
        self.render_nearest_histogram(nearest)

    def render_nearest_histogram(self, nearest):
        self.ax_hist.cla()
        self.ax_hist.set_title("Nearest Object By Slice")
        self.ax_hist.set_facecolor("white")
        self.ax_hist.set_xlim(-0.5, 7.5)
        self.ax_hist.set_ylim(self.max_display_distance_m, 0.0)
        self.ax_hist.set_xticks(range(8))
        self.ax_hist.set_xticklabels([str(i + 1) for i in range(8)], fontsize=8)
        self.ax_hist.set_ylabel("m", fontsize=8)
        self.ax_hist.tick_params(axis="y", labelsize=8)
        self.ax_hist.grid(True, axis="y", alpha=0.20, color="black")

        xs = np.arange(8, dtype=float)
        finite_mask = np.isfinite(nearest)
        display_vals = np.where(finite_mask, nearest, self.max_display_distance_m)
        colors = np.tile(np.array([[0.0, 0.0, 0.0, 0.10]], dtype=float), (8, 1))
        if np.any(finite_mask):
            colors[finite_mask] = self.distance_color_rgba(
                display_vals[finite_mask],
                alpha=np.full(np.count_nonzero(finite_mask), 0.88, dtype=float),
            )
        self.ax_hist.bar(xs, display_vals, width=0.78, color=colors, edgecolor=(1.0, 1.0, 1.0, 0.55), linewidth=0.6)
        for idx, val in enumerate(display_vals):
            if finite_mask[idx]:
                self.ax_hist.text(idx, max(0.05, val - 0.08), f"{val:.2f}", ha="center", va="top", fontsize=7, color="black")
            else:
                self.ax_hist.text(idx, self.max_display_distance_m - 0.06, "-", ha="center", va="top", fontsize=7, color="#555555")

    def render_3d_view(self):
        self.ax_3d.cla()
        title = "Behind View" if self.view_mode == "behind" else "Device View"
        self.ax_3d.set_title(title)
        self.ax_3d.set_xlim(-2.0, 2.0)
        self.ax_3d.set_ylim(-0.5, 4.0)
        self.ax_3d.set_zlim(-0.2, 2.0)
        self.ax_3d.set_xlabel("Right")
        self.ax_3d.set_ylabel("Forward")
        self.ax_3d.set_zlabel("Up")
        if self.view_mode != self.last_render_mode:
            if self.view_mode == "behind":
                self.ax_3d.view_init(elev=10, azim=90)
            else:
                self.ax_3d.view_init(elev=18, azim=-90)
            self.last_render_mode = self.view_mode

        rot_current = rotation_matrix(self.pose.roll, self.pose.pitch, self.pose.yaw)

        current_points_local = project_tof_to_local_points(self.frame)
        if current_points_local.size:
            current_points_local = current_points_local + DEVICE_SENSOR_FACE_LOCAL
            current_distances = np.linalg.norm(current_points_local, axis=1)
            current_visible = current_distances <= self.max_display_distance_m
            current_points_local = current_points_local[current_visible]
            current_distances = current_distances[current_visible]
        else:
            current_distances = np.empty((0,), dtype=float)

        current_tick_ms = self.latest.get("tick_ms") if self.latest else None
        map_points_local, map_weights, _map_keys, map_distances_local = self.world_map.query_local_points(
            rot_current,
            self.map_frame_rot,
            self.max_display_distance_m,
            tick_ms=current_tick_ms,
            min_forward=None,
            max_points=max(20, self.fp_max_render_points),
            score_bias=1.0,
            min_probability=self.min_render_probability,
        )
        if self.view_mode == "behind":
            rot = np.eye(3, dtype=float)
            center = np.array([0.0, -0.08, DEVICE_CENTER_UP_M], dtype=float)
            plot_points = map_points_local + center if map_points_local.size else map_points_local
            current_plot_points = current_points_local + center if current_points_local.size else current_points_local
        else:
            rot = np.eye(3, dtype=float)
            center = np.array([0.0, 0.0, DEVICE_CENTER_UP_M], dtype=float)
            plot_points = map_points_local + center if map_points_local.size else map_points_local
            current_plot_points = current_points_local + center if current_points_local.size else current_points_local
        sensor_origin = center + (DEVICE_SENSOR_FACE_LOCAL @ rot.T)
        front_mask = map_points_local[:, 1] >= 0.0 if map_points_local.size else np.array([], dtype=bool)
        back_mask = ~front_mask if map_points_local.size else np.array([], dtype=bool)

        box = np.array(
            [
                [-DEVICE_HALF_WIDTH_M, -DEVICE_HALF_DEPTH_M, -DEVICE_HALF_HEIGHT_M],
                [DEVICE_HALF_WIDTH_M, -DEVICE_HALF_DEPTH_M, -DEVICE_HALF_HEIGHT_M],
                [DEVICE_HALF_WIDTH_M, DEVICE_HALF_DEPTH_M, -DEVICE_HALF_HEIGHT_M],
                [-DEVICE_HALF_WIDTH_M, DEVICE_HALF_DEPTH_M, -DEVICE_HALF_HEIGHT_M],
                [-DEVICE_HALF_WIDTH_M, -DEVICE_HALF_DEPTH_M, DEVICE_HALF_HEIGHT_M],
                [DEVICE_HALF_WIDTH_M, -DEVICE_HALF_DEPTH_M, DEVICE_HALF_HEIGHT_M],
                [DEVICE_HALF_WIDTH_M, DEVICE_HALF_DEPTH_M, DEVICE_HALF_HEIGHT_M],
                [-DEVICE_HALF_WIDTH_M, DEVICE_HALF_DEPTH_M, DEVICE_HALF_HEIGHT_M],
            ],
            dtype=float,
        )
        verts = (box @ rot.T) + center

        faces = [
            [verts[i] for i in [0, 1, 2, 3]],
            [verts[i] for i in [4, 5, 6, 7]],
            [verts[i] for i in [0, 1, 5, 4]],
            [verts[i] for i in [2, 3, 7, 6]],
            [verts[i] for i in [1, 2, 6, 5]],
            [verts[i] for i in [0, 3, 7, 4]],
        ]

        def draw_points(mask):
            if not plot_points.size or mask.size == 0 or not np.any(mask):
                return
            pts = plot_points[mask]
            w = map_weights[mask]
            d = np.linalg.norm(pts - center, axis=1)
            colors = self.ttl_point_color_rgba(
                d,
                w,
                alpha=np.clip(0.15 + 0.8 * w, 0.15, 0.95),
            )
            sizes = self.dot_size_scale * ((20.0 / np.clip(d, 0.25, 4.0)) + 10.0 * w)
            self.ax_3d.scatter(
                pts[:, 0],
                pts[:, 1],
                pts[:, 2],
                s=sizes,
                c=colors,
                depthshade=False,
            )

        draw_points(back_mask)
        body = Poly3DCollection(faces, facecolors="#d8a23b", edgecolors="#1c1c1c", linewidths=1.0, alpha=0.85)
        self.ax_3d.add_collection3d(body)

        if current_plot_points.size:
            ray_colors = self.distance_color_rgba(
                current_distances,
                alpha=np.full(len(current_plot_points), 0.22, dtype=float),
            )
            endpoint_colors = self.distance_color_rgba(
                current_distances,
                alpha=np.full(len(current_plot_points), 0.95, dtype=float),
            )
            for endpoint, ray_color in zip(current_plot_points, ray_colors):
                self.ax_3d.plot(
                    [sensor_origin[0], endpoint[0]],
                    [sensor_origin[1], endpoint[1]],
                    [sensor_origin[2], endpoint[2]],
                    color=ray_color,
                    lw=0.9,
                )
            ray_sizes = self.dot_size_scale * (14.0 / np.clip(current_distances, 0.25, 4.0))
            self.ax_3d.scatter(
                current_plot_points[:, 0],
                current_plot_points[:, 1],
                current_plot_points[:, 2],
                s=ray_sizes,
                c=endpoint_colors,
                depthshade=False,
                linewidths=0.0,
            )

        draw_points(front_mask)

        axes_len = 0.45
        origin = center
        local_axes = rot @ np.eye(3)
        colors = ["#d1495b", "#2a9d8f", "#3a86ff"]
        for idx in range(3):
            vec = local_axes[:, idx] * axes_len
            self.ax_3d.quiver(origin[0], origin[1], origin[2], vec[0], vec[1], vec[2], color=colors[idx], linewidth=2.0)

        self.ax_3d.scatter([sensor_origin[0]], [sensor_origin[1]], [sensor_origin[2]], color="black", s=22)

    def update(self, _):
        self.read_frames()
        self.playback_step()
        self.im.set_data(self.frame)
        self.render_text()
        now = time.perf_counter()
        if now - self.last_heavy_render_s >= HEAVY_RENDER_INTERVAL_S:
            self.render_first_person_view()
            self.render_3d_view()
            self.last_heavy_render_s = now
        return [self.im, self.text]


def main():
    dashboard = SensorDashboard(PORT, BAUD)
    dashboard.ani = FuncAnimation(dashboard.fig, dashboard.update, interval=50, blit=False, cache_frame_data=False)
    plt.subplots_adjust(left=0.030, right=0.985, top=0.43, bottom=0.055, wspace=0.16, hspace=0.16)
    plt.show()


if __name__ == "__main__":
    main()
