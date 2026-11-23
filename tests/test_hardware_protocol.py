"""tests/test_hardware_protocol.py
Tests du protocole ESP32-S3 Firmware v2 :
- Blocage initial ERR: ESTOP
- Armement ARM et arrêt d'urgence ESTOP
- Checksum XOR *HH
- Formule DRIVE et TANK
- Événement temporisé #D n et Watchdog UWDT
- Verrouillage de la simulation avant connexion physique en mode couplé
- Filtrage strict 100% de PING/PONG/#T dans les logs console
"""

import socket
import threading
import time
from backend.hardware.esp32_simulator import VirtualESP32FirmwareV2
from backend.hardware.hardware_bridge import (
    calculate_xor_checksum,
    format_command_with_optional_crc,
    is_console_noise_line,
)
from backend.navigation.robot_controller import RobotController


def test_estop_on_startup(sim: VirtualESP32FirmwareV2):
    assert sim.armed == 0
    assert sim.send_line("M1:50") == ["ERR: ESTOP"]
    assert sim.send_line("DRIVE:30,0") == ["ERR: ESTOP"]


def test_arm_and_estop(sim: VirtualESP32FirmwareV2):
    assert sim.send_line("ARM") == ["OK ARMED"]
    assert sim.armed == 1
    assert sim.send_line("M1:50") == ["OK M1 50"]

    assert sim.send_line("ESTOP") == ["OK ESTOP"]
    assert sim.armed == 0
    assert sim.send_line("M1:50") == ["ERR: ESTOP"]


def test_checksum_xor(sim: VirtualESP32FirmwareV2):
    assert VirtualESP32FirmwareV2.calculate_checksum("PING") == "10"
    assert VirtualESP32FirmwareV2.calculate_checksum("M1:50") == "43"
    assert VirtualESP32FirmwareV2.calculate_checksum("ARM") == "5E"
    assert VirtualESP32FirmwareV2.calculate_checksum("ESTOP") == "5D"

    assert sim.send_line("PING*10") == ["PONG"]
    assert sim.send_line("ARM*5E") == ["OK ARMED"]
    assert sim.send_line("PING*99") == ["ERR: CRC"]


def test_drive_formula(sim: VirtualESP32FirmwareV2):
    sim.send_line("ARM")
    res = sim.send_line("DRIVE:50,20")
    assert res == ["OK DRIVE 70,30"]
    assert sim.motors[0].target_speed == 70.0
    assert sim.motors[1].target_speed == 30.0


def test_timed_action_done_event(sim: VirtualESP32FirmwareV2):
    sim.send_line("ARM")
    assert sim.send_line("PULSE:5:50:60") == ["OK M5 50 60"]

    found_done = False
    deadline = time.perf_counter() + 0.5
    while time.perf_counter() < deadline:
        line = sim.readline(timeout=0.05).decode("ascii", errors="replace").strip()
        if line == "#D 5":
            found_done = True
            break
    assert found_done


def test_uwdt_watchdog(sim: VirtualESP32FirmwareV2):
    sim.send_line("ARM")
    sim.send_line("UWDT:100")
    sim.send_line("M1:80")
    time.sleep(0.02)
    assert sim.motors[0].target_speed == 80.0
    time.sleep(0.15)
    assert sim.motors[0].target_speed == 0.0


def test_console_noise_filtering():
    assert is_console_noise_line("PING") is True
    assert is_console_noise_line("PING*10") is True
    assert is_console_noise_line("PONG") is True
    assert is_console_noise_line("#T 1 40,40,40,40,0,70,0,0 12345") is True
    assert is_console_noise_line("DRIVE:40,0:5000") is False
    assert is_console_noise_line("#D 1") is False


def test_coupled_mode_lock_and_cloning():
    ctrl = RobotController(use_simulator=True)
    try:
        ctrl.set_execution_mode("COUPLED")
        assert ctrl.is_simulation_locked() is True
        assert ctrl.drive(40, 0, ms=2000) == "ERR: NOT_CONNECTED"
    finally:
        ctrl.close()


def test_estop_kills_active_mission():
    """Vérifie la priorité absolue de l'ESTOP : coupe la mission, force speed à 0,
    passe is_armed à False, mission_running à False et fige strictement la position (X, Z)."""
    ctrl = RobotController(use_simulator=True)
    try:
        ctrl.robot_x = 0.0
        ctrl.robot_z = 0.0
        ctrl.arm()
        assert ctrl.is_armed is True
        assert ctrl.armed == 1
        assert ctrl.estop_active is False

        # Lance une mission autonome
        started = ctrl.start_mission_async("weeding", field_length_m=10.0, field_width_m=4.6)
        assert started is True
        time.sleep(0.15)
        assert ctrl.mission_running is True

        # Envoi de l'ESTOP en plein milieu du mouvement
        res = ctrl.estop()
        assert res in ("OK ESTOP", "OK")
        assert ctrl.speed_kmh == 0.0
        assert ctrl.is_armed is False
        assert ctrl.armed == 0
        assert ctrl.estop_active is True
        assert ctrl.mission_running is False

        # Mémorisation position
        frozen_x = ctrl.robot_x
        frozen_z = ctrl.robot_z

        # 1 seconde plus tard, vérifie que le robot est resté rigoureusement immobile
        time.sleep(1.0)
        assert abs(ctrl.robot_x - frozen_x) < 1e-4
        assert abs(ctrl.robot_z - frozen_z) < 1e-4
        assert ctrl.speed_kmh == 0.0

        # Vérifie le refus catégorique de mouvement tant que ARM n'est pas envoyé
        assert ctrl.drive(30, 0) == "ERR: ESTOP"
        assert ctrl.tank(30, -30) == "ERR: ESTOP"
        assert ctrl.motor(6, 70) == "ERR: ESTOP"
        assert ctrl.pulse(5, 50, 1000) == "ERR: ESTOP"
    finally:
        ctrl.close()

