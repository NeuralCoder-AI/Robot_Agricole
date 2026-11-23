"""tests/test_uc_edge_mode_and_autodeploy.py
Tests unitaires et d'intégration pour :
1. Le profil `DEPLOYMENT_TARGET=ROBOT_UC` : `/api/system/info` désactive la simulation et force `REAL_ZERO_SIM`.
2. `/api/system/version` : retourne le hash Git (ou fallback local propre), message, horodatage et mode.
3. Le `Safe-Reload Guard` (`AutoDeployManager` + `POST /api/system/trigger-update`) :
   - Bloque tout redémarrage et met en file d'attente (`pending_update=True`) pendant qu'une mission est active (`is_mission_running=True`).
   - Applique et vide la file dès que `is_mission_running` passe à `False`.
"""

from pathlib import Path
from fastapi.testclient import TestClient

from backend.api import server_context
from backend.system.auto_deploy_manager import AutoDeployManager


def test_robot_uc_mode_disables_simulation_and_forces_real_zero_sim(client: TestClient, monkeypatch):
    """1. En mode `DEPLOYMENT_TARGET=ROBOT_UC`, `/api/system/info` désactive bien la simulation
    et force le mode `REAL_ZERO_SIM`.
    """
    prev_mode = server_context.robot.execution_mode
    try:
        # En mode PC dev par défaut (DEV_FULL), la simulation est active
        monkeypatch.setenv("DEPLOYMENT_TARGET", "DEV_FULL")
        res_dev = client.get("/api/system/info")
        assert res_dev.status_code == 200
        d_dev = res_dev.json()
        assert d_dev["deployment_target"] == "DEV_FULL"
        assert d_dev["is_robot_uc"] is False
        assert d_dev["simulation_enabled"] is True

        # En mode ROBOT_UC, simulation désactivée et mode forcé sur REAL_ZERO_SIM
        monkeypatch.setenv("DEPLOYMENT_TARGET", "ROBOT_UC")
        res_uc = client.get("/api/system/info")
        assert res_uc.status_code == 200
        d_uc = res_uc.json()
        assert d_uc["deployment_target"] == "ROBOT_UC"
        assert d_uc["is_robot_uc"] is True
        assert d_uc["simulation_enabled"] is False
        assert d_uc["forced_execution_mode"] == "REAL_ZERO_SIM"
        assert d_uc["execution_mode"] == "REAL_ZERO_SIM"
        assert d_uc["auto_hardware_boot"] is True

        # Vérifier qu'on ne peut pas basculer en SIMULATION_ONLY lorsque DEPLOYMENT_TARGET=ROBOT_UC
        hw_res = server_context.robot.set_execution_mode("SIMULATION_ONLY")
        assert hw_res["execution_mode"] == "REAL_ZERO_SIM"
    finally:
        monkeypatch.delenv("DEPLOYMENT_TARGET", raising=False)
        server_context.robot.set_execution_mode(prev_mode)


def test_system_version_endpoint_returns_git_info(client: TestClient, monkeypatch):
    """2. `/api/system/version` retourne le hash, le message de commit, l'horodatage et le mode."""
    monkeypatch.setenv("DEPLOYMENT_TARGET", "ROBOT_UC")
    res = client.get("/api/system/version")
    assert res.status_code == 200
    ver = res.json()
    assert isinstance(ver.get("commit_hash"), str) and len(ver["commit_hash"]) > 0
    assert "commit_message" in ver
    assert "updated_at" in ver
    assert ver["deployment_target"] == "ROBOT_UC"
    assert "badge_text" in ver


def test_safe_reload_guard_blocks_during_mission_and_flushes_after_stop(client: TestClient, tmp_path: Path):
    """3. Safe-Reload Guard : bloque le redémarrage quand `is_mission_running=True`,
    passe `pending_update=True`, puis exécute la mise à jour dès que `is_mission_running=False`.
    """
    mission_state = {"running": True}
    restarts = {"count": 0}

    mgr = AutoDeployManager(
        repo_dir=tmp_path,
        branch="main",
        is_mission_running_fn=lambda: mission_state["running"],
        restart_callback=lambda: restarts.__setitem__("count", restarts["count"] + 1),
    )

    # A. Mission active -> la mise à jour DOIT être bloquée et mise en attente
    res_queued = mgr.trigger_update(branch="main", commit_hash="abc1234", force_apply_local=False)
    assert res_queued["status"] == "queued"
    assert res_queued["pending_update"] is True
    assert res_queued["restarted"] is False
    assert restarts["count"] == 0
    assert "Mise à jour mise en attente" in res_queued["message"]

    # Même un flush explicite tant que mission_running=True ne doit rien redémarrer
    assert mgr.check_and_flush_pending_update() is None
    assert restarts["count"] == 0

    # B. La mission s'arrête (mission_running = False) -> le flush exécute immédiatement la mise à jour
    mission_state["running"] = False
    res_flushed = mgr.check_and_flush_pending_update()
    assert res_flushed is not None
    assert res_flushed["status"] == "updated"
    assert res_flushed["pending_update"] is False
    assert res_flushed["restarted"] is True
    assert restarts["count"] == 1

    # C. Vérification via l'endpoint FastAPI POST /api/system/trigger-update
    prev_running = server_context.robot.mission_running
    try:
        server_context.robot.mission_running = True
        api_res_q = client.post("/api/system/trigger-update", json={"commit_hash": "deadbeef", "force_apply_local": False})
        assert api_res_q.status_code == 200
        q_data = api_res_q.json()
        assert q_data["status"] == "queued"
        assert q_data["pending_update"] is True
        assert q_data["restarted"] is False

        server_context.robot.mission_running = False
        api_res_ok = client.post("/api/system/trigger-update", json={"commit_hash": "deadbeef", "force_apply_local": False})
        assert api_res_ok.status_code == 200
        ok_data = api_res_ok.json()
        assert ok_data["status"] == "updated"
        assert ok_data["pending_update"] is False
    finally:
        server_context.robot.mission_running = prev_running

