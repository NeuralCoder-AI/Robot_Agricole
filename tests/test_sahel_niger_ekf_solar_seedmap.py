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
from backend.hardware.gps_rtk_reader import (
    DEFAULT_ZED_F9P_BAUDRATE,
    GpsRtkReader,
    LocalEnuFrame,
    compute_nmea_checksum,
    parse_nmea_sentence,
    verify_nmea_checksum,
)
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
    hw_bms = r_hw.json()["solar_bms"]
    assert hw_bms["battery_voltage_v"] == 12.0
    assert hw_bms["battery_capacity_ah"] == 200.0
    assert hw_bms["soc_min_cutoff_pct"] == 40.0
    assert hw_bms["pv_stc_power_w"] == 530.0
    assert hw_bms["electronics_power_w"] == 12.0


def _wrap_nmea(body: str) -> str:
    return f"${body}*{compute_nmea_checksum(body)}"


def test_gps_rtk_nmea_decoder_enu_fix_modes_and_stanley():
    assert DEFAULT_ZED_F9P_BAUDRATE == 38400

    # Origine parcelle à Niamey (13°30.0000' N, 002°06.0000' E -> 13.5000° N, 2.1000° E)
    frame = LocalEnuFrame(lat0=13.5, lon0=2.1)

    # 1. Décodage $GNGGA qualité 4 (RTK Fixe) + checksum XOR strict
    gga_fixed = _wrap_nmea("GNGGA,120000.00,1330.0054,N,00206.0000,E,4,18,0.6,210.5,M,0.0,M,0.8,0000")
    assert verify_nmea_checksum(gga_fixed) is True
    assert verify_nmea_checksum(gga_fixed[:-2] + "00") is False

    pos = parse_nmea_sentence(gga_fixed)
    assert pos is not None
    assert pos.quality == 4
    assert pos.is_rtk_fixed is True
    assert pos.quality_label == "RTK_FIXED"
    assert pos.num_satellites == 18
    assert pos.hdop == 0.6

    # Projection ENU : +0.0054' Nord = +0.00009 deg ≈ +10.018 m
    x_e, y_n = frame.to_xy(pos.lat, pos.lon)
    assert abs(x_e) < 0.01
    assert 9.95 < y_n < 10.08
    lat_rt, lon_rt = frame.to_latlon(x_e, y_n)
    assert math.isclose(lat_rt, pos.lat, abs_tol=1e-9)
    assert math.isclose(lon_rt, pos.lon, abs_tol=1e-9)

    # 2. Décodage $GNVTG vitesse (1.8 km/h = 0.50 m/s >= 0.15 m/s -> cap GPS fiable)
    vtg = _wrap_nmea("GNVTG,12.5,T,,M,0.97,N,1.80,K,D")
    pos_vtg = parse_nmea_sentence(vtg, current=pos)
    assert pos_vtg is not None
    assert math.isclose(pos_vtg.speed_mps, 0.50, abs_tol=1e-3)
    assert pos_vtg.has_reliable_heading() is True

    # 3. Flux GpsRtkReader en rejeu + injection EKF -> bascule RTK_FLOAT (qual=5) vers Vision Locale
    gga_float = _wrap_nmea("GNGGA,120001.00,1330.0054,N,00206.0000,E,5,14,1.2,210.5,M,0.0,M,1.1,0000")
    reader = GpsRtkReader()
    assert reader.feed_sentence(gga_fixed) is not None
    assert reader.wait_for_fix(min_quality=4, timeout_s=0.2) is True
    assert reader.get_status()["gga_quality_label"] == "RTK_FIXED"

    ekf = RtkImuEkfLocalizer()
    st_fixed = ekf.update_from_nmea(gga_fixed, enu_frame=frame, imu_heading_rad=0.0, now_monotonic=100.0)
    assert st_fixed is not None
    assert st_fixed["rtk_available"] is True
    assert st_fixed["gga_quality"] == 4
    assert st_fixed["localization_mode"] == "RTK_EKF_FUSION"

    st_float = ekf.update_from_nmea(gga_float, enu_frame=frame, imu_heading_rad=0.0, now_monotonic=101.0)
    assert st_float is not None
    assert st_float["rtk_available"] is False
    assert st_float["gga_quality"] == 5
    assert st_float["gga_quality_label"] == "RTK_FLOAT"
    assert st_float["localization_mode"] == "LOCAL_VISION_FALLBACK"

    # 4. Loi de commande Stanley GPS-RTK (k_cap=0.8, k_lat=40, k_e=2.0, borne ±35 %)
    cmd_corr = RtkImuEkfLocalizer.compute_stanley_rtk_command(
        e_lat_m=0.10, delta_cap_deg=-5.0, v_mps=0.30, dist_remaining_m=4.0, v_cmd_nominal=20
    )
    assert -35 <= cmd_corr["d"] <= 35
    assert cmd_corr["d"] < 0  # Robot décalé à droite (+0.10 m) & cap trop à droite -> braque à gauche
    assert cmd_corr["arrived"] is False
    assert cmd_corr["a"] == 20

    cmd_slow = RtkImuEkfLocalizer.compute_stanley_rtk_command(e_lat_m=0.0, delta_cap_deg=0.0, dist_remaining_m=0.80, v_cmd_nominal=20)
    assert cmd_slow["a"] == 15 and cmd_slow["arrived"] is False
    cmd_stop = RtkImuEkfLocalizer.compute_stanley_rtk_command(e_lat_m=0.0, delta_cap_deg=0.0, dist_remaining_m=0.25, v_cmd_nominal=20)
    assert cmd_stop["arrived"] is True and cmd_stop["a"] == 0 and cmd_stop["cmd_str"] == "STOP"


def test_solar_bms_real_hardware_gel_12v_530wc_and_overcurrent():
    bms = SolarBMS(initial_soc=0.80)
    st0 = bms.get_status()
    assert st0["battery_voltage_v"] == 12.0
    assert st0["battery_capacity_ah"] == 200.0
    assert st0["battery_energy_wh"] == 2400.0
    assert st0["soc_min_cutoff_pct"] == 40.0
    assert st0["pv_stc_power_w"] == 530.0
    assert st0["electronics_power_w"] == 12.0

    # Vérification Éq. (III.17) : G = 850 W/m², T_amb = 40 °C, P_STC = 530 Wc, η_MPPT = 0.95 -> ~370 W
    pv = bms.compute_pv_power_w(solar_deploy_ratio=1.0, irradiance_w_m2=850.0, ambient_temp_c=40.0)
    assert math.isclose(pv["cell_temp_c"], 58.7, abs_tol=0.1)
    assert math.isclose(pv["thermal_derating_factor"], 0.8652, abs_tol=1e-3)
    assert 368.0 <= pv["pv_power_w"] <= 372.0
    assert pv["pv_charge_current_a"] < 40.0  # Sous la limite Sohigh PCA-4840 (40 A)

    # Protection surintensité bus 12 V (> 40 A = 480 W) sur pic M5/M6
    st_over = bms.step(
        dt=10.0,
        motor_pwms=[70, 70, 70, 70, 90, 95, 0, 0],
        solar_deploy_ratio=0.0,
        irradiance_w_m2=0.0,
        soil_load_multiplier_m5_m6=2.0,
    )
    assert st_over["battery_current_a"] > 40.0
    assert st_over["overcurrent_protection_active"] is True
    assert st_over["m5_m6_load_clamped"] is True

    # Protection batterie basse Gel 12 V (SoC <= 40 %)
    bms_low = SolarBMS(initial_soc=0.39)
    assert bms_low.get_status()["low_battery_cutoff"] is True
    assert bms_low.get_status()["usable_energy_wh"] == 0.0


