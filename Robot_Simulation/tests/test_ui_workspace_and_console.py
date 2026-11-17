"""
test_ui_workspace_and_console.py
Tests automatisés de la structure UI : Canvas 3D, Grille Workspace 380px, et Console Permanente avec filtre anti-PONG.
"""

import os
import re
from fastapi.testclient import TestClient


def test_main_workspace_and_canvas_dom_structure(client: TestClient):
    """Vérifie que index.html contient la grille .main-workspace, le canvas 3D et la console permanente."""
    response = client.get("/")
    assert response.status_code == 200
    html = response.text

    # 1. Grille principale .main-workspace et #mainWorkspace
    assert 'id="mainWorkspace"' in html, "L'élément #mainWorkspace doit être présent dans le DOM"
    assert 'class="container main-workspace"' in html or 'main-workspace' in html

    # 2. Viewport et Canvas WebGL
    assert 'id="viewportContainer"' in html, "Le conteneur #viewportContainer doit être présent"
    assert 'id="webglCanvas"' in html, "Le canvas #webglCanvas doit être présent dans le DOM"

    # 3. Zone Haute des Missions
    assert 'class="mission-top-zone"' in html, "La zone haute .mission-top-zone doit être présente"
    assert 'id="panelWeedingMission"' in html
    assert 'id="panelSeedingMission"' in html
    assert 'id="panelReseedingMission"' in html
    assert 'id="panelManualMission"' in html

    # 4. Zone Basse Permanente : Console de Commandes
    assert 'class="console-bottom-zone"' in html, "La zone basse .console-bottom-zone doit être présente de manière permanente"
    assert 'id="toggle-hide-pong"' in html, "Le toggle #toggle-hide-pong doit être présent"
    assert re.search(r'id="toggle-hide-pong"[^>]*checked', html), "Le toggle #toggle-hide-pong doit être coché par défaut"
    assert 'id="consoleLogBox"' in html, "La boîte #consoleLogBox doit être présente"
    assert 'id="consoleCmdInput"' in html, "Le champ #consoleCmdInput doit être présent pour la saisie directe"


def test_css_grid_and_sidebar_380px_dimensions():
    """Vérifie que les styles CSS définissent minmax(0, 1fr) 380px et aucune coupure."""
    css_files = ["APP_Mobile/static/css/main.css", "APP_Mobile/static/css/dashboard.css"]
    combined_css = ""
    for p in css_files:
        assert os.path.exists(p), f"Fichier {p} introuvable"
        with open(p, "r", encoding="utf-8") as f:
            combined_css += f.read() + "\n"

    # Vérification de la grille 380px
    assert "minmax(0, 1fr) 380px" in combined_css, "La grille principale doit utiliser 'minmax(0, 1fr) 380px'"
    assert "380px" in combined_css, "La largeur de 380px doit être fixée pour le panneau latéral"
    assert "console-bottom-zone" in combined_css, "Le style .console-bottom-zone doit être défini"
    assert "min-height: 220px" in combined_css, "La console basse doit avoir une hauteur minimale garantie (220px)"


def test_console_logger_filtering_and_formatting():
    """Vérifie la logique de filtrage du module console_logger.js."""
    logger_path = "APP_Mobile/static/js/ui/console_logger.js"
    assert os.path.exists(logger_path), "console_logger.js doit exister"
    with open(logger_path, "r", encoding="utf-8") as f:
        js_code = f.read()

    assert "isConsoleNoise" in js_code
    assert "ignorePong" in js_code
    assert "setIgnorePong" in js_code
    assert "formatLogEntry" in js_code
    assert "📤 [TX]" in js_code
    assert "📥 [RX]" in js_code
    assert "🏁 [FIN]" in js_code


def test_threejs_init_and_resize_observer():
    """Vérifie que Three.js est initialisé avec ResizeObserver et la couleur de fond sablonneuse lumineuse."""
    index_path = "APP_Mobile/static/index.html"
    with open(index_path, "r", encoding="utf-8") as f:
        html = f.read()

    # Vérification couleur horizon / ciel sablonneux lumineux (0xd8b88a) et jamais noir
    assert "0xd8b88a" in html, "scene.background doit être initialisé avec 0xd8b88a (jamais noir)"
    assert "THREE.AmbientLight" in html, "Une lumière ambiante AmbientLight doit être ajoutée pour éviter l'écran noir"
    assert "ResizeObserver" in html, "Un ResizeObserver doit être installé sur le conteneur du canvas 3D"
    assert "window.onWindowResize = onWindowResize;" in html, "onWindowResize doit être exposé sur window pour les transitions de vue"
