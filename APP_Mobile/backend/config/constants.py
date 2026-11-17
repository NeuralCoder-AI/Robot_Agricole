"""backend/config/constants.py
Constantes physiques et agronomiques du Robot Agricole ESP32-S3 (Relevé 26/10/2026).
"""

from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent.parent.parent
STATIC_ASSETS_DIR = BASE_DIR / "static" / "assets"
NVS_ROBOT_PATH = BASE_DIR / "nvs_robot.json"
ROBOT_SPECS_PATH = BASE_DIR / "robot_specs.json"

# Dimensions physiques relevé terrain (26/10/2026)
CHASSIS_LENGTH_TOP_CM = 169.0
CHASSIS_LENGTH_BOTTOM_CM = 130.0
CHASSIS_SLOPE_LENGTH_CM = 30.0
TOTAL_WIDTH_WITH_WHEELS_CM = 121.0
CHASSIS_BODY_WIDTH_CM = 81.0
TOTAL_HEIGHT_FOLDED_CM = 88.0
GROUND_CLEARANCE_CM = 24.0
INNER_TUNNEL_WIDTH_CM = 79.5

# Pneumatiques & Train roulant
TIRE_DIAMETER_CM = 26.5
TIRE_CIRCUMFERENCE_CM = 83.25
TIRE_WIDTH_CM = 19.0
TRACK_WIDTH_CM = 103.0
WHEELBASE_CM = 110.0

# Outils agronomiques
TINES_BAR_WIDTH_CM = 84.0
TINES_TOTAL_COUNT = 4
FRONT_SMALL_TINE_WIDTH_CM = 13.0
REAR_LARGE_TINE_WIDTH_CM = 25.0
TILLER_WIDTH_CM = 23.0
SEEDER_NOZZLES_COUNT = 2
SEEDER_ROW_SPACING_CM = 46.0
ROW_SKIP_THRESHOLD_CM = 50.0  # 2 × grosse dent (25 cm) = 50 cm

# Encombrement hors-tout pour la navigation
FRONT_OVERHANG_M = 1.05
REAR_OVERHANG_M = 1.15
TOTAL_ROBOT_LENGTH_M = 2.20
TOTAL_ROBOT_WIDTH_M = 1.21
SAFETY_MARGIN_M = 0.35

