import asyncio
import json
import subprocess
import time
import urllib.request
import websockets

async def run_debug():
    # 1. Lancer Chromium en headless avec port de debug
    proc = subprocess.Popen([
        "/usr/bin/chromium",
        "--headless=new",
        "--no-sandbox",
        "--disable-gpu",
        "--remote-debugging-port=9222",
        "about:blank"
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    
    try:
        await asyncio.sleep(1.5)
        
        # 2. Obtenir l'URL WebSocket de la page
        with urllib.request.urlopen("http://127.0.0.1:9222/json") as resp:
            targets = json.loads(resp.read().decode())
        ws_url = targets[0]["webSocketDebuggerUrl"]
        print(f"Connecté à Chromium via CDP : {ws_url}")
        
        async with websockets.connect(ws_url) as ws:
            msg_id = 1
            
            async def send_cmd(method, params=None):
                nonlocal msg_id
                cid = msg_id
                msg_id += 1
                await ws.send(json.dumps({"id": cid, "method": method, "params": params or {}}))
                while True:
                    raw = await ws.recv()
                    data = json.loads(raw)
                    if data.get("id") == cid:
                        return data.get("result", {})
            
            # Activer Console, Runtime, Page
            await ws.send(json.dumps({"id": 991, "method": "Console.enable"}))
            await ws.send(json.dumps({"id": 992, "method": "Runtime.enable"}))
            await ws.send(json.dumps({"id": 993, "method": "Page.enable"}))
            
            # Naviguer vers http://localhost:8000
            print("Navigation vers http://localhost:8000 ...")
            await send_cmd("Page.navigate", {"url": "http://127.0.0.1:8000"})
            
            # Attendre 3 secondes et écouter les logs console
            console_logs = []
            exceptions = []
            
            async def listen_events():
                while True:
                    try:
                        raw = await asyncio.wait_for(ws.recv(), timeout=0.5)
                        ev = json.loads(raw)
                        method = ev.get("method", "")
                        if "console" in method.lower() or "exception" in method.lower():
                            print(f"[CDP EVENT] {ev}")
                    except asyncio.TimeoutError:
                        break
            
            await asyncio.sleep(2.5)
            await listen_events()
            
            # Évaluer des expressions JS pour inspecter l'état
            eval_script = """
            (() => {
                const canvas = document.getElementById("webglCanvas");
                const container = document.getElementById("viewportContainer");
                const gl = canvas ? canvas.getContext("webgl") || canvas.getContext("experimental-webgl") : null;
                return {
                    container_w: container ? container.clientWidth : -1,
                    container_h: container ? container.clientHeight : -1,
                    canvas_w: canvas ? canvas.width : -1,
                    canvas_h: canvas ? canvas.height : -1,
                    canvas_cw: canvas ? canvas.clientWidth : -1,
                    canvas_ch: canvas ? canvas.clientHeight : -1,
                    has_three: typeof THREE !== 'undefined',
                    has_scene: typeof scene !== 'undefined' && scene !== null,
                    has_renderer: typeof renderer !== 'undefined' && renderer !== null,
                    scene_bg: (typeof scene !== 'undefined' && scene && scene.background) ? scene.background.getHexString() : null,
                    gl_context: gl !== null,
                    gl_error: gl ? gl.getError() : -1,
                    children_count: (typeof scene !== 'undefined' && scene) ? scene.children.length : -1,
                    active_mission: window.currentActiveMission || null
                };
            })()
            """
            res = await send_cmd("Runtime.evaluate", {"expression": eval_script, "returnByValue": True})
            print("\n--- ÉTAT JS RUNTIME DANS CHROMIUM ---")
            print(json.dumps(res.get("result", {}).get("value", {}), indent=2))
            
            # Simuler le clic pour choisir la mission Désherbage si le wizard est là
            click_script = """
            (() => {
                if (window.wizardNav) {
                    window.wizardNav.selectMissionAndOpenWorkspace('weeding');
                    return "Mission weeding selectionnee via wizardNav";
                }
                return "wizardNav non trouve";
            })()
            """
            click_res = await send_cmd("Runtime.evaluate", {"expression": click_script, "returnByValue": True})
            print(f"\nAction Wizard: {click_res.get('result', {}).get('value')}")
            
            await asyncio.sleep(1.0)
            res_after = await send_cmd("Runtime.evaluate", {"expression": eval_script, "returnByValue": True})
            print("\n--- ÉTAT JS RUNTIME APRÈS OUVERTURE MISSION ---")
            print(json.dumps(res_after.get("result", {}).get("value", {}), indent=2))
            
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except Exception:
            proc.kill()

if __name__ == "__main__":
    asyncio.run(run_debug())

