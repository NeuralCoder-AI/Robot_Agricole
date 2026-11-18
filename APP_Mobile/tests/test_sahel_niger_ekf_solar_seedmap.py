"""tests/test_sahel_niger_ekf_solar_seedmap.py
Tests unitaires et d'intégration validant l'alignement sur les chapitres validés (Niger / Sahel) :
1. Constantes agronomiques nigériennes (Bakoye et al., 2020) : 30–40 cm théorique vs 46 cm mécanique,
   intra-rang 10–20 cm (préréglage 15 cm, compatibilité 20 cm), profondeur 4–6 cm (cible 5 cm).
2. Fusion GPS RTK (u-blox ZED-F9P bi-antenne) + IMU + Vision locale par Filtre de Kalman Étendu (`RtkImuEkfLocalizer`).
3. Géo-mémorisation matricielle des graines type FarmDroid FD20 (`SeedMapMemory`).
4. Modèle énergétique photovoltaïque sahélien (5–7 kWh/m²/j, 35–45 °C) & BMS (`SolarBMS`) + endpoints `/api/status`, `/api/hardware/status`.
"""

import math
from pathlib import Path
from fastapi.testclient import TestClient

from backend.config.constants import (
    SAHEL_THEORETICAL_INTER_ROW_CM_RANGE,
    MECHANICAL_NOZZLE_SPACING_CM,
    SEED_DEPTH_TARGET_CM,
    SEED_DEPTH_CM_RANGE,
    SAHEL_INTRA_ROW_SPACING_CM_DEFAULT,
    INTRA_ROW_SPACING_CM_RANGE,
)
from backend.config.robot_config import RobotConfigManager
from backend.hardware.solar_bms import SolarBMS
from backend.navigation.agri_nav_engine import AgriNavEngine, RtkImuEkfLocalizer
from backend.navigation.reseeding_planner import ReseedingPlanner, SeedMapMemory
from backend.navigation.robot_controller import RobotController


def test_niger_sahel_agronomic_constants_and_dynamic_presets(tmp_path: Path):
    assert SAHEL_THEORETICAL_INTER_ROW_CM_RANGE == (30.0, 40.0)
    assert MECHANICAL_NOZZLE_SPACING_CM == 46.0
    assert SEED_DEPTH_TARGET_CM == 5.0
    assert SEED_DEPTH_CM_RANGE == (4.0, 6.0)
    assert SAHEL_INTRA_ROW_SPACING_CM_DEFAULT == 15.0
    assert INTRA_ROW_SPACING_CM_RANGE == (10.0, 20.0)

    cfg = RobotConfigManager(sqlite_path=tmp_path / "sahel.sqlite", json_path=tmp_path / "sahel.json")
    specs = cfg.get_specs()
    assert specs["seed_depth_cm"] == 5.0
    assert specs["intra_row_seed_spacing_cm"] == 15.0
    assert specs["tool_spacing_m"] == 0.46
    assert specs["configurable_tool_spacing_m_range"] == [0.30, 0.50]

    # Plage bornée 10–20 cm intra-rang et 4–6 cm profondeur
    upd = cfg.update_specs({"intra_row_seed_spacing_cm": 20.0, "seed_depth_cm": 5.5})
    assert upd["intra_row_seed_spacing_cm"] == 20.0
    assert upd["seed_depth_cm"] == 5.5

    planner_sahel = ReseedingPlanner.from_niger_sahel_preset()
    assert planner_sahel.nominal_seed_gap_cm == 15.0
    assert planner_sahel.seed_depth_cm == 5.0


def test_rtk_imu_ekf_localizer_fusion_and_vision_fallback():
    engine = AgriNavEngine()
    ekf: RtkImuEkfLocalizer = engine.ekf_localizer
    ekf.reset(x=0.25, z=0.0, theta=0.05)

    # 1. Prédiction odométrique + IMU
    pred = ekf.predict(dt=0.5, v_odom_mps=0.60, omega_imu_rad_s=0.0)
    assert pred["z"] > 0.25

    # 2. Correction centimétrique GPS RTK ZED-F9P bi-antenne
    st_rtk = ekf.update_rtk(x_gps=0.01, z_gps=0.31, theta_dual_ant_rad=0.0, rtk_fix=True, age_s=0.05)
    assert st_rtk["rtk_available"] is True
    assert st_rtk["localization_mode"] == "RTK_EKF_FUSION"
    assert abs(st_rtk["x"] - 0.01) < 0.04
    assert st_rtk["pos_std_m"] < 0.03

    # 3. Occultation RTK (age > 1.5s / rtk_fix=False) -> bascule résiliente Vision Locale + EKF
    st_lost = ekf.update_rtk(x_gps=9.9, z_gps=9.9, rtk_fix=False, age_s=2.0)
    assert st_lost["rtk_available"] is False
    assert st_lost["localization_mode"] == "LOCAL_VISION_FALLBACK"

    st_vis = ekf.update_vision(lateral_err_m=-0.02, heading_err_rad=0.01, row_center_x_m=0.0, line_angle_rad=0.0)
    assert st_vis["localization_mode"] == "LOCAL_VISION_EKF"
    assert abs(st_vis["x"]) < 0.05


def test_seed_map_memory_farmdroid_geo_recording_and_protection(tmp_path: Path):
    mem_path = tmp_path / "seed_map.json"
    seed_map = SeedMapMemory(storage_path=mem_path)

    # Robot en (X=0.0, Z=2.0) orienté +Z (yaw=0) -> buses à (-0.23, 1.15) et (+0.23, 1.15)
    added = seed_map.record_nozzle_pass(
        robot_x=0.0,
        robot_z=2.0,
        robot_yaw_rad=0.0,
        seeder_row_spacing_m=0.46,
        seeder_rear_offset_m=0.85,
        depth_cm=5.0,
        spacing_cm=15.0,
    )
    assert len(added) == 2
    assert math.isclose(added[0]["x"], -0.23, abs_tol=1e-3)
    assert math.isclose(added[0]["z"], 1.15, abs_tol=1e-3)
    assert math.isclose(added[1]["x"], 0.23, abs_tol=1e-3)
    assert added[0]["depth_cm"] == 5.0
    assert added[0]["spacing_cm"] == 15.0

    # Déduplication stricte au même emplacement
    dup = seed_map.record_nozzle_pass(robot_x=0.0, robot_z=2.0, robot_yaw_rad=0.0)
    assert len(dup) == 0

    # Protection des poquets en pré/post-émergence
    assert seed_map.is_near_memorized_seed(-0.23, 1.15, protection_radius_m=0.08) is True
    assert seed_map.is_near_memorized_seed(0.0, 1.15, protection_radius_m=0.08) is False
    nearby = seed_map.query_nearby_seeds(0.0, 1.20, radius_m=0.50)
    assert len(nearby) == 2

    seed_map.save()
    reloaded = SeedMapMemory(storage_path=mem_path)
    assert reloaded.summary()["total_memorized_seeds"] == 2


def test_sahel_solar_bms_power_balance_and_api_exposure(client: TestClient):
    bms = SolarBMS(initial_soc=0.80, sahel_daily_irradiance_kwh_m2=6.2)

    # 1. Production PV sous soleil sahélien (40 °C, panneaux déployés à 100 %, 850 W/m²)
    pv = bms.compute_pv_power_w(solar_deploy_ratio=1.0, irradiance_w_m2=850.0, ambient_temp_c=40.0)
    assert pv["thermal_derating_factor"] < 1.0  # Déclassement thermique réel sous climat chaud
    assert pv["cell_temp_c"] > 40.0
    assert pv["pv_power_w"] > 350.0
    assert bms.estimate_daily_harvest_wh(solar_deploy_ratio=1.0) > 2500.0

    # 2. Surplus solaire à l'arrêt/veille -> SoC augmente
    st_charge = bms.step(dt=3600.0, motor_pwms=[0] * 8, solar_deploy_ratio=1.0, irradiance_w_m2=900.0, ambient_temp_c=40.0)
    assert st_charge["net_power_w"] > 0.0
    assert st_charge["soc_pct"] > 80.0
    assert "RECHARGE_SOLAIRE" in st_charge["autonomy_label"]

    # 3. Pic de courant M5/M6 sous forte charge en sol sec + faible ensoleillement
    st_load = bms.step(
        dt=60.0,
        motor_pwms=[80, 80, 80, 80, 90, 95, 0, 0],
        solar_deploy_ratio=0.0,
        irradiance_w_m2=0.0,
        ambient_temp_c=42.0,
        soil_load_multiplier_m5_m6=2.5,
    )
    assert st_load["net_power_w"] < 0.0
    assert st_load["motors_power_w"] > 700.0
    assert st_load["autonomy_hours"] is not None

    # 4. Vérification endpoints REST /api/status et /api/hardware/status
    r_status = client.get("/api/status")
    assert r_status.status_code == 200
    s_json = r_status.json()
    assert "solar_bms" in s_json
    assert "ekf_localization" in s_json
    assert "seed_map" in s_json

    r_hw = client.get("/api/hardware/status")
    assert r_hw.status_code == 200
    assert "solar_bms" in r_hw.json()

