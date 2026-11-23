"""tests/test_config_and_specs.py
Validation des dimensions mécaniques réelles (169 cm, 121 cm, 46 cm, 50 cm),
de la cohérence agronomique et des modèles PBR.
"""

import os
from fastapi.testclient import TestClient

from backend.config.constants import (
    CHASSIS_LENGTH_TOP_CM,
    CHASSIS_LENGTH_BOTTOM_CM,
    TOTAL_WIDTH_WITH_WHEELS_CM,
    CHASSIS_BODY_WIDTH_CM,
    GROUND_CLEARANCE_CM,
    TIRE_DIAMETER_CM,
    SEEDER_ROW_SPACING_CM,
    ROW_SKIP_THRESHOLD_CM,
)
from backend.assets_gen.pbr_builder import ASSETS_DIR


def test_robot_dimensions_constants_and_no_magic_numbers():
    assert CHASSIS_LENGTH_TOP_CM == 169.0
    assert CHASSIS_LENGTH_BOTTOM_CM == 130.0
    assert TOTAL_WIDTH_WITH_WHEELS_CM == 121.0
    assert CHASSIS_BODY_WIDTH_CM == 81.0
    assert GROUND_CLEARANCE_CM == 24.0
    assert TIRE_DIAMETER_CM == 26.5
    assert SEEDER_ROW_SPACING_CM == 46.0
    assert ROW_SKIP_THRESHOLD_CM == 50.0


def test_pbr_assets_generated():
    expected_files = [
        "soil_albedo.jpg",
        "soil_normal.jpg",
        "soil_roughness.jpg",
        "chassis_albedo.jpg",
        "chassis_normal.jpg",
        "chassis_roughness.jpg",
        "tire_albedo.jpg",
        "tire_normal.jpg",
        "tire_roughness.jpg",
        "rock_albedo.jpg",
        "rock_normal.jpg",
        "rock_roughness.jpg",
        "solar_albedo.jpg",
        "solar_normal.jpg",
        "robot_chassis.glb",
        "wheel_agricultural.glb",
        "rock_boulder.glb",
    ]
    for fname in expected_files:
        fpath = os.path.join(ASSETS_DIR, fname)
        assert os.path.isfile(fpath), f"Fichier PBR manquant: {fname}"
        assert os.path.getsize(fpath) > 100


def test_robot_specs_survey_defaults_and_live_edit(client: TestClient):
    resp_reset = client.post("/api/robot-specs/reset")
    assert resp_reset.status_code == 200
    specs = resp_reset.json()["robot_specs"]

    assert specs["chassis_length_top_cm"] == 169.0
    assert specs["total_width_with_wheels_cm"] == 121.0
    assert specs["chassis_body_width_cm"] == 81.0
    assert specs["ground_clearance_cm"] == 24.0
    assert specs["wheelbase_cm"] == 110.0
    assert specs["track_width_cm"] == 103.0
    assert specs["seeder_row_spacing_cm"] == 46.0

    resp_upd = client.post(
        "/api/robot-specs",
        json={"chassis_length_top_cm": 175.0, "tire_diameter_cm": 30.0},
    )
    assert resp_upd.status_code == 200
    upd_specs = resp_upd.json()["robot_specs"]
    assert upd_specs["chassis_length_top_cm"] == 175.0
    assert upd_specs["tire_diameter_cm"] == 30.0

