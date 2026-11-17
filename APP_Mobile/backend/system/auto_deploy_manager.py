"""backend/system/auto_deploy_manager.py
Gestionnaire de Déploiement Automatique (Git Auto-Pull + Webhook + Safe-Reload Guard)
pour l'Unité Centrale (UC) embarquée du Robot Agricole.

Fonctionnalités :
1. Expose les informations Git actives (`get_version_info()` : hash court, message, horodatage, mode).
2. Reçoit un déclenchement (`POST /api/system/trigger-update`) OU surveille périodiquement le dépôt
   toutes les 10 secondes (`GitWatcher`) en mode `ROBOT_UC`.
3. Safe-Reload Guard :
   - Si `is_mission_running() == True` : bloque tout redémarrage, passe `pending_update = True` et affiche
     `⏳ Nouveau commit reçu — Mise à jour mise en attente jusqu'à l'arrêt du robot`.
   - Dès que `is_mission_running() == False` : applique `git fetch` + `git reset --hard origin/<branch>`,
     installe les dépendances si `requirements.txt` a changé, puis relance proprement Uvicorn/Systemd.
"""

import logging
import os
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

from backend.config.robot_config import get_deployment_target

logger = logging.getLogger("AutoDeployManager")


class AutoDeployManager:
    """Gestionnaire thread-safe de mise à jour Git avec garde-fou anti-interruption de mission."""

    def __init__(
        self,
        repo_dir: Optional[Path] = None,
        branch: Optional[str] = None,
        is_mission_running_fn: Optional[Callable[[], bool]] = None,
        restart_callback: Optional[Callable[[], None]] = None,
        on_status_change_fn: Optional[Callable[[Dict[str, Any]], None]] = None,
    ):
        self._lock = threading.RLock()
        default_app_dir = Path(__file__).resolve().parent.parent.parent
        # Trouver la racine .git si présente dans le répertoire parent
        if repo_dir is not None:
            self.repo_dir = Path(repo_dir)
        elif (default_app_dir.parent / ".git").exists():
            self.repo_dir = default_app_dir.parent
        else:
            self.repo_dir = default_app_dir

        self.app_dir = default_app_dir
        self.branch = branch or os.getenv("GIT_DEPLOY_BRANCH", "main")
        self.is_mission_running_fn = is_mission_running_fn
        self.restart_callback = restart_callback
        self.on_status_change_fn = on_status_change_fn

        self.pending_update: bool = False
        self.pending_commit_hash: Optional[str] = None
        self.last_status: str = "À jour"
        self.last_message: str = "Système prêt"
        self.last_updated_at: str = datetime.now(timezone.utc).isoformat()
        self.last_error: str = ""
        self.update_history: List[Dict[str, Any]] = []

        self._watcher_stop = threading.Event()
        self._watcher_thread: Optional[threading.Thread] = None
        self.watcher_active: bool = False

    def set_mission_running_guard(self, fn: Callable[[], bool]) -> None:
        self.is_mission_running_fn = fn

    def is_mission_running(self) -> bool:
        if self.is_mission_running_fn is None:
            return False
        try:
            return bool(self.is_mission_running_fn())
        except Exception:
            return False

    def _run_git(self, args: List[str], timeout: float = 12.0) -> subprocess.CompletedProcess:
        return subprocess.run(
            ["git", *args],
            cwd=str(self.repo_dir),
            capture_output=True,
            text=True,
            timeout=timeout,
        )

    def _has_git_repo(self) -> bool:
        return (self.repo_dir / ".git").exists()

    def get_version_info(self) -> Dict[str, Any]:
        """Retourne le hash court, le message du dernier commit, l'heure de mise à jour et l'état du Safe-Reload."""
        commit_hash = "local-dev"
        commit_message = "Copie locale (aucun dépôt .git configuré)"
        branch_name = self.branch

        if self._has_git_repo():
            try:
                res_hash = self._run_git(["rev-parse", "--short", "HEAD"], timeout=3.0)
                if res_hash.returncode == 0 and res_hash.stdout.strip():
                    commit_hash = res_hash.stdout.strip()
                res_msg = self._run_git(["log", "-1", "--pretty=%s"], timeout=3.0)
                if res_msg.returncode == 0 and res_msg.stdout.strip():
                    commit_message = res_msg.stdout.strip()
                res_br = self._run_git(["rev-parse", "--abbrev-ref", "HEAD"], timeout=3.0)
                if res_br.returncode == 0 and res_br.stdout.strip():
                    branch_name = res_br.stdout.strip()
            except Exception as e:
                logger.debug("Lecture version git impossible : %s", e)

        target = get_deployment_target()
        with self._lock:
            pending = self.pending_update
            status_txt = (
                "⏳ Nouveau commit reçu — Mise à jour mise en attente jusqu'à l'arrêt du robot"
                if pending
                else self.last_status
            )
            badge = f"📦 Version: #{commit_hash} ({'Mise à jour en attente' if pending else 'À jour'})"
            return {
                "commit_hash": commit_hash,
                "commit_message": commit_message,
                "branch": branch_name,
                "updated_at": self.last_updated_at,
                "deployment_target": target,
                "is_robot_uc": target == "ROBOT_UC",
                "pending_update": pending,
                "pending_commit_hash": self.pending_commit_hash,
                "up_to_date": not pending,
                "last_status": status_txt,
                "last_message": self.last_message,
                "last_error": self.last_error,
                "watcher_active": self.watcher_active,
                "badge_text": badge,
            }

    def check_remote_for_updates(self, branch: Optional[str] = None) -> Dict[str, Any]:
        """Exécute `git fetch origin` et compare HEAD vs `origin/<branch>` sans modifier l'arbre de travail."""
        target_branch = branch or self.branch
        if not self._has_git_repo():
            return {"has_update": False, "reason": "NO_GIT_REPO"}

        try:
            remotes = self._run_git(["remote"], timeout=4.0)
            if remotes.returncode != 0 or "origin" not in remotes.stdout.split():
                return {"has_update": False, "reason": "NO_ORIGIN_REMOTE"}

            fetch_res = self._run_git(["fetch", "origin", target_branch], timeout=12.0)
            if fetch_res.returncode != 0:
                return {"has_update": False, "reason": "FETCH_FAILED", "stderr": fetch_res.stderr.strip()}

            local_sha = self._run_git(["rev-parse", "HEAD"], timeout=3.0).stdout.strip()
            remote_sha = self._run_git(["rev-parse", f"origin/{target_branch}"], timeout=3.0).stdout.strip()
            if local_sha and remote_sha and local_sha != remote_sha:
                short_remote = remote_sha[:7]
                return {"has_update": True, "local_sha": local_sha, "remote_sha": remote_sha, "short_remote": short_remote}
            return {"has_update": False, "local_sha": local_sha, "remote_sha": remote_sha}
        except Exception as e:
            return {"has_update": False, "reason": "ERROR", "error": str(e)}

    def _apply_git_reset_and_deps(self, branch: str) -> Dict[str, Any]:
        """Applique `git reset --hard origin/<branch>` et réinstalle `requirements.txt` si modifié."""
        if not self._has_git_repo():
            return {"git_updated": False, "requirements_changed": False, "reason": "NO_GIT_REPO"}

        remotes = self._run_git(["remote"], timeout=4.0)
        if remotes.returncode != 0 or "origin" not in remotes.stdout.split():
            return {"git_updated": False, "requirements_changed": False, "reason": "LOCAL_REPO_ONLY"}

        old_head = self._run_git(["rev-parse", "HEAD"], timeout=3.0).stdout.strip()
        self._run_git(["fetch", "origin", branch], timeout=15.0)
        reset_res = self._run_git(["reset", "--hard", f"origin/{branch}"], timeout=12.0)
        new_head = self._run_git(["rev-parse", "HEAD"], timeout=3.0).stdout.strip()

        req_changed = False
        if old_head and new_head and old_head != new_head:
            diff_res = self._run_git(["diff", "--name-only", old_head, new_head], timeout=5.0)
            changed_files = diff_res.stdout.splitlines() if diff_res.returncode == 0 else []
            req_changed = any(f.endswith("requirements.txt") for f in changed_files)
            if req_changed:
                req_path = self.app_dir / "requirements.txt"
                if req_path.is_file():
                    subprocess.run(
                        [sys.executable, "-m", "pip", "install", "-r", str(req_path)],
                        cwd=str(self.app_dir),
                        capture_output=True,
                        text=True,
                        timeout=120.0,
                    )

        return {
            "git_updated": reset_res.returncode == 0,
            "old_head": old_head,
            "new_head": new_head,
            "requirements_changed": req_changed,
        }

    def _trigger_safe_server_restart(self) -> bool:
        """Relance Uvicorn / Systemd uniquement hors environnement de test unitaire."""
        if self.restart_callback is not None:
            self.restart_callback()
            return True

        # Ne jamais tuer le processus si lancé via pytest
        if os.getenv("PYTEST_CURRENT_TEST"):
            logger.info("Safe-Reload: redémarrage ignoré sous pytest.")
            return False

        def _delayed_restart():
            time.sleep(0.6)
            # Sous Systemd (`Restart=always`), sortir proprement relance immédiatement le service à neuf
            if os.getenv("ROBOT_UC_SYSTEMD", "").lower() in ("1", "true", "yes"):
                os._exit(0)
            else:
                os.execv(sys.executable, [sys.executable, *sys.argv])

        threading.Thread(target=_delayed_restart, name="SafeUvicornRestart", daemon=True).start()
        return True

    def trigger_update(
        self,
        branch: Optional[str] = None,
        commit_hash: Optional[str] = None,
        force_apply_local: bool = True,
    ) -> Dict[str, Any]:
        """Déclenche une mise à jour sécurisée avec Safe-Reload Guard."""
        target_branch = branch or self.branch
        with self._lock:
            mission_active = self.is_mission_running()
            if mission_active:
                self.pending_update = True
                self.pending_commit_hash = commit_hash or self.pending_commit_hash or "pending"
                self.last_status = "Mise à jour en attente"
                self.last_message = "⏳ Nouveau commit reçu — Mise à jour mise en attente jusqu'à l'arrêt du robot"
                payload = {
                    "status": "queued",
                    "accepted": True,
                    "pending_update": True,
                    "restarted": False,
                    "mission_running": True,
                    "message": self.last_message,
                    "version": self.get_version_info(),
                }
                self._notify(payload)
                return payload

            # Robot à l'arrêt : exécution immédiate et sécurisée
            apply_res = self._apply_git_reset_and_deps(target_branch) if force_apply_local else {"git_updated": False}
            self.pending_update = False
            self.pending_commit_hash = None
            self.last_status = "À jour"
            self.last_updated_at = datetime.now(timezone.utc).isoformat()
            self.last_message = f"✅ Mise à jour appliquée sur {target_branch}"
            restarted = self._trigger_safe_server_restart()
            payload = {
                "status": "updated",
                "accepted": True,
                "pending_update": False,
                "restarted": restarted,
                "mission_running": False,
                "apply_details": apply_res,
                "message": self.last_message,
                "version": self.get_version_info(),
            }
            self.update_history.append({"ts": self.last_updated_at, "status": "updated", "details": apply_res})
            self._notify(payload)
            return payload

    def check_and_flush_pending_update(self) -> Optional[Dict[str, Any]]:
        """Si une mise à jour était en attente et que la mission vient de s'arrêter, l'applique immédiatement."""
        with self._lock:
            if not self.pending_update:
                return None
            if self.is_mission_running():
                return None
        return self.trigger_update()

    def _notify(self, payload: Dict[str, Any]) -> None:
        if self.on_status_change_fn is not None:
            try:
                self.on_status_change_fn(payload)
            except Exception as e:
                logger.debug("Erreur notification AutoDeploy : %s", e)

    def start_watcher(self, interval_sec: float = 10.0) -> None:
        """Démarre la boucle de surveillance Auto-Pull toutes les 10s sur l'UC."""
        if self._watcher_thread is not None and self._watcher_thread.is_alive():
            return
        self._watcher_stop.clear()
        self.watcher_active = True

        def _loop():
            while not self._watcher_stop.is_set():
                try:
                    # 1. Flusher toute mise à jour en attente dès que la mission s'arrête
                    self.check_and_flush_pending_update()
                    # 2. Vérifier si origin a un nouveau commit
                    chk = self.check_remote_for_updates(self.branch)
                    if chk.get("has_update"):
                        self.trigger_update(branch=self.branch, commit_hash=chk.get("short_remote"))
                except Exception as e:
                    logger.debug("GitWatcher erreur : %s", e)
                self._watcher_stop.wait(timeout=max(2.0, float(interval_sec)))

        self._watcher_thread = threading.Thread(target=_loop, name="GitWatcher", daemon=True)
        self._watcher_thread.start()

    def stop_watcher(self) -> None:
        self._watcher_stop.set()
        self.watcher_active = False


auto_deploy_manager = AutoDeployManager()

