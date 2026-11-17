"""backend/assets_gen/pbr_builder.py
Pipeline de Génération de Textures PBR et Maillages 3D GLTF (.glb).
"""

import os
from pathlib import Path
from typing import Dict
import cv2
import numpy as np
from PIL import Image
import trimesh

from backend.config.constants import (
    STATIC_ASSETS_DIR,
    CHASSIS_LENGTH_TOP_CM,
    CHASSIS_LENGTH_BOTTOM_CM,
    CHASSIS_BODY_WIDTH_CM,
    TIRE_DIAMETER_CM,
    TIRE_WIDTH_CM,
)

ASSETS_DIR: str = str(STATIC_ASSETS_DIR)


def heightmap_to_normal_map(heightmap: np.ndarray, strength: float = 3.0) -> np.ndarray:
    gx = cv2.Sobel(heightmap, cv2.CV_32F, 1, 0, ksize=3) * strength
    gy = cv2.Sobel(heightmap, cv2.CV_32F, 0, 1, ksize=3) * strength
    gz = np.ones_like(heightmap, dtype=np.float32)
    norm = np.sqrt(gx * gx + gy * gy + gz * gz) + 1e-6
    nx = (-gx / norm) * 0.5 + 0.5
    ny = (-gy / norm) * 0.5 + 0.5
    nz = (gz / norm) * 0.5 + 0.5
    normal_rgb = np.stack([nx, ny, nz], axis=-1)
    return np.clip(normal_rgb * 255.0, 0, 255).astype(np.uint8)


def generate_soil_pbr(size: int = 512) -> None:
    rng = np.random.default_rng(42)
    x = np.linspace(0, 8 * np.pi, size, dtype=np.float32)
    y = np.linspace(0, 8 * np.pi, size, dtype=np.float32)
    xx, _ = np.meshgrid(x, y)
    furrows = 0.5 + 0.5 * np.cos(xx * 2.0)
    fine_noise = rng.normal(0.0, 1.0, (size, size)).astype(np.float32)
    clods_large = cv2.GaussianBlur(fine_noise, (15, 15), 4.0)
    clods_med = cv2.GaussianBlur(fine_noise, (7, 7), 1.8)
    micro_grain = cv2.GaussianBlur(fine_noise, (3, 3), 0.7)
    height = np.clip(0.45 * furrows + 0.35 * clods_large + 0.15 * clods_med + 0.05 * micro_grain, 0.0, 1.0)
    r = np.clip(148 + height * 68 + micro_grain * 14, 118, 228).astype(np.uint8)
    g = np.clip(88 + height * 48 + micro_grain * 10, 66, 152).astype(np.uint8)
    b = np.clip(42 + height * 32 + micro_grain * 8, 28, 92).astype(np.uint8)
    albedo = np.stack([r, g, b], axis=-1)
    normal = heightmap_to_normal_map(height, strength=4.5)
    roughness = np.clip(225 - height * 30 + micro_grain * 15, 180, 252).astype(np.uint8)
    Image.fromarray(albedo).save(os.path.join(ASSETS_DIR, "soil_albedo.jpg"), quality=92)
    Image.fromarray(normal).save(os.path.join(ASSETS_DIR, "soil_normal.jpg"), quality=92)
    Image.fromarray(roughness).save(os.path.join(ASSETS_DIR, "soil_roughness.jpg"), quality=92)


def generate_chassis_metal_pbr(size: int = 512) -> None:
    rng = np.random.default_rng(101)
    base_noise = rng.normal(0.0, 1.0, (size, size)).astype(np.float32)
    brushed_h = cv2.GaussianBlur(base_noise, (31, 3), 5.0)
    brushed_v = cv2.GaussianBlur(base_noise, (3, 31), 5.0)
    scratches = np.zeros((size, size), dtype=np.float32)
    for _ in range(90):
        x1, y1 = int(rng.integers(0, size)), int(rng.integers(0, size))
        angle = float(rng.uniform(0, np.pi * 2))
        length = int(rng.integers(15, 80))
        x2 = int(np.clip(x1 + np.cos(angle) * length, 0, size - 1))
        y2 = int(np.clip(y1 + np.sin(angle) * length, 0, size - 1))
        cv2.line(scratches, (x1, y1), (x2, y2), float(rng.uniform(0.4, 1.0)), 1, lineType=cv2.LINE_AA)
    height = np.clip(0.5 + brushed_h * 0.15 + brushed_v * 0.1 - scratches * 0.25, 0.0, 1.0)
    r = np.clip(24 + height * 18 + scratches * 120, 15, 220).astype(np.uint8)
    g = np.clip(98 + height * 32 + scratches * 95, 65, 230).astype(np.uint8)
    b = np.clip(38 + height * 16 + scratches * 110, 22, 220).astype(np.uint8)
    albedo = np.stack([r, g, b], axis=-1)
    normal = heightmap_to_normal_map(height, strength=2.2)
    roughness = np.clip(85 + (1.0 - height) * 60 - scratches * 35, 35, 190).astype(np.uint8)
    Image.fromarray(albedo).save(os.path.join(ASSETS_DIR, "chassis_albedo.jpg"), quality=92)
    Image.fromarray(normal).save(os.path.join(ASSETS_DIR, "chassis_normal.jpg"), quality=92)
    Image.fromarray(roughness).save(os.path.join(ASSETS_DIR, "chassis_roughness.jpg"), quality=92)


def generate_tire_rubber_pbr(size: int = 512) -> None:
    rng = np.random.default_rng(77)
    height = np.full((size, size), 0.3, dtype=np.float32)
    for row in range(0, size, 48):
        cv2.line(height, (0, row), (size // 2, row + 24), 0.95, 14)
        cv2.line(height, (size, row + 12), (size // 2, row + 36), 0.95, 14)
    height = cv2.GaussianBlur(height, (7, 7), 1.5)
    dirt_mask = cv2.GaussianBlur(rng.uniform(0, 1, (size, size)).astype(np.float32), (19, 19), 5.0)
    dirt_mask = np.clip((dirt_mask - 0.48) * 3.0, 0.0, 1.0) * (1.0 - height * 0.5)
    r = np.clip(24 + dirt_mask * 48 + height * 8, 18, 95).astype(np.uint8)
    g = np.clip(24 + dirt_mask * 32 + height * 8, 18, 75).astype(np.uint8)
    b = np.clip(26 + dirt_mask * 20 + height * 8, 20, 60).astype(np.uint8)
    albedo = np.stack([r, g, b], axis=-1)
    normal = heightmap_to_normal_map(height, strength=5.0)
    roughness = np.clip(215 + dirt_mask * 30 - height * 15, 185, 250).astype(np.uint8)
    Image.fromarray(albedo).save(os.path.join(ASSETS_DIR, "tire_albedo.jpg"), quality=92)
    Image.fromarray(normal).save(os.path.join(ASSETS_DIR, "tire_normal.jpg"), quality=92)
    Image.fromarray(roughness).save(os.path.join(ASSETS_DIR, "tire_roughness.jpg"), quality=92)


def generate_rock_And_solar_pbr(size: int = 512) -> None:
    rng = np.random.default_rng(202)
    noise = rng.normal(0.0, 1.0, (size, size)).astype(np.float32)
    rock_h = (
        cv2.GaussianBlur(noise, (31, 31), 8.0) * 0.6
        + cv2.GaussianBlur(noise, (11, 11), 2.5) * 0.3
        + cv2.GaussianBlur(noise, (3, 3), 0.8) * 0.1
    )
    rock_h = np.clip(0.5 + rock_h, 0.0, 1.0)
    r = np.clip(78 + rock_h * 55, 45, 155).astype(np.uint8)
    g = np.clip(84 + rock_h * 52, 50, 160).astype(np.uint8)
    b = np.clip(88 + rock_h * 48, 52, 165).astype(np.uint8)
    rock_albedo = np.stack([r, g, b], axis=-1)
    rock_normal = heightmap_to_normal_map(rock_h, strength=6.0)
    rock_rough = np.clip(210 - rock_h * 35, 165, 248).astype(np.uint8)
    Image.fromarray(rock_albedo).save(os.path.join(ASSETS_DIR, "rock_albedo.jpg"), quality=92)
    Image.fromarray(rock_normal).save(os.path.join(ASSETS_DIR, "rock_normal.jpg"), quality=92)
    Image.fromarray(rock_rough).save(os.path.join(ASSETS_DIR, "rock_roughness.jpg"), quality=92)

    solar_albedo = np.zeros((size, size, 3), dtype=np.uint8)
    solar_albedo[:, :] = [14, 46, 112]
    solar_h = np.ones((size, size), dtype=np.float32) * 0.5
    step = 64
    for i in range(0, size, step):
        solar_albedo[max(0, i - 1) : min(size, i + 2), :] = [190, 215, 240]
        solar_albedo[:, max(0, i - 1) : min(size, i + 2)] = [190, 215, 240]
        solar_h[max(0, i - 1) : min(size, i + 2), :] = 0.2
        solar_h[:, max(0, i - 1) : min(size, i + 2)] = 0.2
    solar_normal = heightmap_to_normal_map(solar_h, strength=2.5)
    Image.fromarray(solar_albedo).save(os.path.join(ASSETS_DIR, "solar_albedo.jpg"), quality=92)
    Image.fromarray(solar_normal).save(os.path.join(ASSETS_DIR, "solar_normal.jpg"), quality=92)


def generate_glb_models() -> Dict[str, str]:
    l_top = CHASSIS_LENGTH_TOP_CM / 100.0
    l_bot = CHASSIS_LENGTH_BOTTOM_CM / 100.0
    w_body = CHASSIS_BODY_WIDTH_CM / 100.0

    left_top_rail = trimesh.creation.box(extents=[0.05, 0.06, l_top])
    left_top_rail.apply_translation([-0.38, 0.58, 0.0])
    left_top_rail.visual.vertex_colors = [27, 110, 43, 255]

    right_top_rail = trimesh.creation.box(extents=[0.05, 0.06, l_top])
    right_top_rail.apply_translation([0.38, 0.58, 0.0])
    right_top_rail.visual.vertex_colors = [27, 110, 43, 255]

    left_bot_rail = trimesh.creation.box(extents=[0.05, 0.06, l_bot])
    left_bot_rail.apply_translation([-0.38, 0.28, 0.0])
    left_bot_rail.visual.vertex_colors = [27, 110, 43, 255]

    right_bot_rail = trimesh.creation.box(extents=[0.05, 0.06, l_bot])
    right_bot_rail.apply_translation([0.38, 0.28, 0.0])
    right_bot_rail.visual.vertex_colors = [27, 110, 43, 255]

    front_wall = trimesh.creation.box(extents=[w_body, 0.34, 0.05])
    front_wall.apply_translation([0.0, 0.43, 0.78])
    front_wall.visual.vertex_colors = [27, 110, 43, 255]

    rear_wall = trimesh.creation.box(extents=[w_body, 0.34, 0.05])
    rear_wall.apply_translation([0.0, 0.43, -0.78])
    rear_wall.visual.vertex_colors = [27, 110, 43, 255]

    hopper_drum = trimesh.creation.cylinder(radius=0.19, height=0.44, sections=24)
    rot_hopper = trimesh.transformations.rotation_matrix(0.32, [1, 0, 0])
    hopper_drum.apply_transform(rot_hopper)
    hopper_drum.apply_translation([0.0, 0.52, -0.92])
    hopper_drum.visual.vertex_colors = [27, 110, 43, 255]

    tube_left = trimesh.creation.cylinder(radius=0.018, height=0.52, sections=12)
    tube_left.apply_translation([-0.23, 0.22, -1.05])
    tube_left.visual.vertex_colors = [27, 110, 43, 255]

    tube_right = trimesh.creation.cylinder(radius=0.018, height=0.52, sections=12)
    tube_right.apply_translation([0.23, 0.22, -1.05])
    tube_right.visual.vertex_colors = [27, 110, 43, 255]

    chassis_mesh = trimesh.util.concatenate([
        left_top_rail, right_top_rail, left_bot_rail, right_bot_rail,
        front_wall, rear_wall, hopper_drum, tube_left, tube_right
    ])
    chassis_glb_path = os.path.join(ASSETS_DIR, "robot_chassis.glb")
    chassis_mesh.export(chassis_glb_path)

    r_tire = (TIRE_DIAMETER_CM / 100.0) / 2.0
    w_tire = TIRE_WIDTH_CM / 100.0
    rim_cyl = trimesh.creation.cylinder(radius=r_tire * 0.9, height=w_tire, sections=24)
    rot_y = trimesh.transformations.rotation_matrix(np.pi / 2.0, [0, 1, 0])
    rim_cyl.apply_transform(rot_y)
    rim_cyl.visual.vertex_colors = [27, 110, 43, 255]

    paddles = [rim_cyl]
    for i in range(12):
        ang = (i / 12.0) * 2.0 * np.pi
        paddle = trimesh.creation.box(extents=[w_tire, 0.028, 0.012])
        rot_x = trimesh.transformations.rotation_matrix(ang, [1, 0, 0])
        paddle.apply_translation([0.0, r_tire * 0.95, 0.0])
        paddle.apply_transform(rot_x)
        paddle.visual.vertex_colors = [34, 125, 50, 255]
        paddles.append(paddle)

    wheel_mesh = trimesh.util.concatenate(paddles)
    wheel_glb_path = os.path.join(ASSETS_DIR, "wheel_agricultural.glb")
    wheel_mesh.export(wheel_glb_path)

    rock_mesh = trimesh.creation.icosphere(subdivisions=2, radius=0.85)
    rng = np.random.default_rng(55)
    disp = 1.0 + rng.uniform(-0.18, 0.22, size=(len(rock_mesh.vertices), 1))
    rock_mesh.vertices = rock_mesh.vertices * disp
    rock_mesh.visual.vertex_colors = [95, 108, 115, 255]
    rock_glb_path = os.path.join(ASSETS_DIR, "rock_boulder.glb")
    rock_mesh.export(rock_glb_path)

    return {
        "chassis_glb": chassis_glb_path,
        "wheel_glb": wheel_glb_path,
        "rock_glb": rock_glb_path,
    }


def ensure_all_pbr_assets() -> Dict[str, str]:
    os.makedirs(ASSETS_DIR, exist_ok=True)
    generate_soil_pbr()
    generate_chassis_metal_pbr()
    generate_tire_rubber_pbr()
    generate_rock_And_solar_pbr()
    return generate_glb_models()

