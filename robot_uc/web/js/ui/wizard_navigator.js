/**
 * web/js/ui/wizard_navigator.js — Gestionnaire de navigation fluide par Sous-Fenêtres (Wizard Épuré).
 * Étape 1 : Choix du Mode d'Exécution (Simulation Uniquement, Couplé, ou Réel Zéro Simulation).
 * Étape 2 : Connexion Matérielle ESP32-S3 (si mode Couplé sélectionné).
 * Étape 3 : Menu Principal — Choix de la Mission (1. Semence Initiale, 2. Révision Semence, 3. Désherbage IA, 4. Conduite Manuelle).
 * Étape 4 : Espace de Travail Épuré (Cockpit Caméra USB 78° / Scène 3D + Panneau unique de la mission choisie).
 */

export class WizardNavigator {
    constructor(options = {}) {
        this.currentStep = 1; // 1, 2, 3 ou 4 (4 = Workspace)
        this.executionMode = 'SIMULATION_ONLY'; // 'SIMULATION_ONLY' ou 'COUPLED'
        this.selectedMission = 'seeding'; // 'seeding', 'reseeding', 'weeding', 'manual'
        this.hwConnected = false;
        this.onMissionChange = options.onMissionChange || null;
        this.onModeChange = options.onModeChange || null;
        this.onHardwareConnect = options.onHardwareConnect || null;

        this.overlayEl = null;
        this.progressionDirection = 1; // +1 = Droite, -1 = Gauche
        this.robotUCMode = false;
    }

    init() {
        this.createWizardDOM();
        this.bindGlobalEvents();
        if (window.isRobotUCMode || this.robotUCMode) {
            this.enableRobotUCEmbeddedMode();
        } else {
            // Afficher l'étape 1 au lancement
            this.goToStep(1);
        }
    }

    enableRobotUCEmbeddedMode() {
        this.robotUCMode = true;
        this.executionMode = 'REAL_ZERO_SIM';
        this.createWizardDOM();
        // En mode ROBOT_UC : parcours guidé 4 étapes dès le démarrage :
        // Étape 1 (Connexion ESP32 + Calibration Moteurs) -> Étape 2 (Réglage Angle Caméra) -> Étape 3 (Menu Missions) -> Étape 4 (Cockpit)
        this.goToStep(1);
    }

    createWizardDOM() {
        if (document.getElementById('wizardOverlay')) return;

        const overlay = document.createElement('div');
        overlay.id = 'wizardOverlay';
        overlay.className = 'wizard-overlay';
        overlay.innerHTML = `
            <div class="wizard-box" style="max-width: 940px;">
                <div class="wizard-header">
                    <div class="wizard-title-group">
                        <h2 id="wizardStepTitle">🔌 Connexion Robot &amp; Calibration</h2>
                        <p id="wizardStepSubtitle">Préparez le robot physique avant de lancer une mission agricole</p>
                    </div>
                    <div class="wizard-steps-indicator">
                        <div class="wizard-step-dot" id="wizardDot1" title="Étape 1 : Connexion Robot & Calibration"></div>
                        <div class="wizard-step-dot" id="wizardDot2" title="Étape 2 : Réglage & Calibration Caméra"></div>
                        <div class="wizard-step-dot" id="wizardDot3" title="Étape 3 : Menu Principal Missions"></div>
                    </div>
                </div>

                <div class="wizard-body" id="wizardBody">
                    <!-- Contenu injecté dynamiquement selon l'étape -->
                </div>

                <div class="wizard-footer" id="wizardFooter">
                    <!-- Boutons injectés dynamiquement -->
                </div>
            </div>
        `;
        document.body.appendChild(overlay);
        this.overlayEl = overlay;
    }

    goToStep(step) {
        if (window.isMissionRunning && step !== 4) {
            if (typeof window.showMissionLockAlert === 'function') {
                window.showMissionLockAlert();
            }
            return;
        }
        const isUC = Boolean(this.robotUCMode || window.isRobotUCMode);
        this.currentStep = step;
        const body = document.getElementById('wizardBody');
        const footer = document.getElementById('wizardFooter');
        const title = document.getElementById('wizardStepTitle');
        const subtitle = document.getElementById('wizardStepSubtitle');

        // Toujours afficher les 3 pastilles de préparation (Étapes 1, 2, 3) avant le Cockpit (Étape 4)
        for (let i = 1; i <= 3; i++) {
            const dot = document.getElementById(`wizardDot${i}`);
            if (dot) {
                dot.style.display = '';
                dot.className = 'wizard-step-dot' + (i === step ? ' active' : (i < step ? ' done' : ''));
            }
        }

        if (step === 1 && isUC) {
            this._renderUCStep1RobotConnection(title, subtitle, body, footer);
            return;
        }
        if (step === 2 && isUC) {
            this._renderUCStep2CameraCalibration(title, subtitle, body, footer);
            return;
        }

        if (step === 1) {
            this.overlayEl.style.display = 'flex';
            title.innerHTML = "⚙️ Étape 1 — Choix du Mode d'Exécution";
            subtitle.innerText = "Sélectionnez votre mode : Simulation 3D, Hybride Couplé ou 100 % Réel Zéro Simulation (Caméra USB (78°))";

            body.innerHTML = `
                <div class="wizard-grid-2" style="grid-template-columns: repeat(3, 1fr);">
                    <div class="wizard-card-option ${this.executionMode === 'SIMULATION_ONLY' ? 'active' : ''}" id="optSimOnly">
                        <div>
                            <div class="wizard-card-icon">💻</div>
                            <div class="wizard-card-title">
                                <span>Simulation Uniquement</span>
                                <span class="wizard-card-tag">100% Virtuel</span>
                            </div>
                            <div class="wizard-card-desc">
                                Jumeau numérique 3D complet avec physique du terrain, rochers, cultures d'arachide et caméra IA virtuelle. Aucun robot physique requis.
                            </div>
                        </div>
                        <div style="color: #00e5ff; font-weight: 700; font-size: 13px;">Lancer la simulation directe ➔</div>
                    </div>

                    <div class="wizard-card-option ${this.executionMode === 'COUPLED' ? 'active' : ''}" id="optCoupled">
                        <div>
                            <div class="wizard-card-icon">🤖</div>
                            <div class="wizard-card-title">
                                <span>Couplé au Robot Réel</span>
                                <span class="wizard-card-tag" style="color:#f59e0b; border-color:#f59e0b; background:rgba(245,158,11,0.12);">Hybride 3D + ESP32</span>
                            </div>
                            <div class="wizard-card-desc">
                                Scène 3D active + commandes envoyées en temps réel au robot physique ESP32-S3 (8 × BTS7960 : M1-M4 roues, M5 dents, M6 fraise, M7 panneaux, M8 semoir).
                            </div>
                        </div>
                        <div style="color: #f59e0b; font-weight: 700; font-size: 13px;">Configurer la liaison ESP32 ➔</div>
                    </div>

                    <div class="wizard-card-option ${this.executionMode === 'REAL_ZERO_SIM' ? 'active' : ''}" id="optRealZeroSim">
                        <div>
                            <div class="wizard-card-icon">👁️</div>
                            <div class="wizard-card-title">
                                <span>Zéro Simulation (100% Réel)</span>
                                <span class="wizard-card-tag" style="color:#10b981; border-color:#10b981; background:rgba(16,185,129,0.14);">Caméra T'nB • ESP32 Optionnel</span>
                            </div>
                            <div class="wizard-card-desc">
                                Coupe à 100 % le rendu 3D et affiche en grand format le flux réel de la webcam USB externe (T'nB INCAM1080A /dev/video2) avec pipeline Vision IA OpenCV et Mode Test Vision sans ESP32.
                            </div>
                        </div>
                        <div style="color: #6ee7b7; font-weight: 700; font-size: 13px;">Ouvrir le Cockpit Vision IA Direct ➔</div>
                    </div>
                </div>
            `;

            footer.innerHTML = `
                <div></div>
                <div style="font-size: 12px; color: #94a3b8;">Sélectionnez l'un des 3 modes pour continuer</div>
            `;

            document.getElementById('optSimOnly').onclick = () => {
                this.executionMode = 'SIMULATION_ONLY';
                if (this.onModeChange) this.onModeChange('SIMULATION_ONLY');
                this.goToStep(3);
            };

            document.getElementById('optCoupled').onclick = () => {
                this.executionMode = 'COUPLED';
                if (this.onModeChange) this.onModeChange('COUPLED');
                this.goToStep(2);
            };

            document.getElementById('optRealZeroSim').onclick = () => {
                this.executionMode = 'REAL_ZERO_SIM';
                if (this.onModeChange) this.onModeChange('REAL_ZERO_SIM');
                this.goToStep(3);
            };

        } else if (step === 2) {
            this.overlayEl.style.display = 'flex';
            title.innerHTML = "🔌 Étape 2 — Connexion au Robot Physique ESP32-S3";
            subtitle.innerText = "Connectez votre robot ESP32-S3 maintenant ou passez au Cockpit pour configurer le Robot et la Caméra USB (78°)";

            body.innerHTML = `
                <div class="hw-connection-panel">
                    <div style="display:grid; grid-template-columns: 1fr 2fr; gap: 16px;">
                        <div class="hw-connection-field">
                            <label>Liaison Matérielle</label>
                            <select id="wizHwTransport">
                                <option value="USB_UART">USB-UART Série (/dev/ttyACM0 ou /dev/ttyUSB0)</option>
                                <option value="WIFI_HTTP">Wi-Fi HTTP (Serveur ESP32 192.168.4.1)</option>
                                <option value="SOCKET_TCP_WS">Socket TCP / WebSocket</option>
                            </select>
                        </div>
                        <div class="hw-connection-field">
                            <label>Cible (Port Série ou IP:Port)</label>
                            <input type="text" id="wizHwTarget" value="/dev/ttyACM0" placeholder="/dev/ttyACM0, /dev/ttyUSB0 ou 192.168.4.1">
                        </div>
                    </div>

                    <div style="display:flex; gap:10px; margin-top:8px;">
                        <button type="button" class="btn-wizard-next" id="btnWizTestHw" style="flex:1;">
                            ⚡ Tester & Établir la Liaison (PING -> PONG + VER)
                        </button>
                    </div>

                    <div id="wizHwStatusBox" class="hw-status-box ${this.hwConnected ? 'connected' : 'disconnected'}">
                        <span id="wizHwStatusText">
                            ${this.hwConnected ? '✅ ROBOT PHYSIQUE CONNECTÉ & OPÉRATIONNEL (Firmware v2)' : '🛑 NON CONNECTÉ — En attente du signal ESP32-S3'}
                        </span>
                        <span id="wizHwPingBadge" style="font-family:monospace; font-size:11px;">
                            ${this.hwConnected ? 'PING: OK' : 'En attente'}
                        </span>
                    </div>
                </div>
            `;

            footer.innerHTML = `
                <button type="button" class="btn-wizard-back" id="btnWizBackToStep1">⬅ Retour au Mode</button>
                <div style="display:flex; gap:10px;">
                    <button type="button" class="btn-wizard-back" id="btnWizSkipToStep3" style="border-color:#38bdf8; color:#7dd3fc;">
                        Configurer depuis le Cockpit ➔
                    </button>
                    <button type="button" class="btn-wizard-next" id="btnWizContinueToStep3" ${!this.hwConnected ? 'disabled' : ''}>
                        Continuer vers le Choix de Mission ➔
                    </button>
                </div>
            `;

            document.getElementById('btnWizBackToStep1').onclick = () => this.goToStep(1);
            document.getElementById('btnWizSkipToStep3').onclick = () => this.goToStep(3);

            const transportSelect = document.getElementById('wizHwTransport');
            const targetInput = document.getElementById('wizHwTarget');
            transportSelect.onchange = () => {
                if (transportSelect.value === 'USB_UART') {
                    targetInput.value = '/dev/ttyACM0';
                } else if (transportSelect.value === 'SOCKET_TCP_WS') {
                    targetInput.value = '192.168.4.1:80';
                } else {
                    targetInput.value = '192.168.4.1';
                }
            };

            const testBtn = document.getElementById('btnWizTestHw');
            const continueBtn = document.getElementById('btnWizContinueToStep3');
            const statusBox = document.getElementById('wizHwStatusBox');
            const statusText = document.getElementById('wizHwStatusText');

            testBtn.onclick = async () => {
                statusBox.className = 'hw-status-box testing';
                statusText.innerText = "⏳ Tentative de connexion à l'ESP32-S3 (PING)...";
                testBtn.disabled = true;

                if (this.onHardwareConnect) {
                    try {
                        const res = await this.onHardwareConnect({
                            transport: transportSelect.value,
                            target: targetInput.value,
                            execution_mode: this.executionMode || 'COUPLED',
                        });
                        if (res && res.connected) {
                            this.hwConnected = true;
                            statusBox.className = 'hw-status-box connected';
                            statusText.innerText = "✅ LIAISON ACTIVE AVEC L'ESP32-S3 (Firmware v2 détecté)";
                            continueBtn.disabled = false;
                        } else {
                            this.hwConnected = false;
                            statusBox.className = 'hw-status-box disconnected';
                            const reason = (res && (res.last_error || res.error)) ? ` (${res.last_error || res.error})` : "";
                            statusText.innerText = `❌ ÉCHEC CONNEXION${reason} — Vérifiez le port USB/Wi-Fi ou cliquez sur 'Configurer depuis le Cockpit'`;
                            continueBtn.disabled = true;
                        }
                    } catch (err) {
                        this.hwConnected = false;
                        statusBox.className = 'hw-status-box disconnected';
                        statusText.innerText = "❌ ERREUR RÉSEAU / PORT : " + err.message;
                        continueBtn.disabled = true;
                    }
                }
                testBtn.disabled = false;
            };

            continueBtn.onclick = () => {
                if (this.hwConnected) {
                    this.goToStep(3);
                }
            };

        } else if (step === 3) {
            const isUC = Boolean(this.robotUCMode || window.isRobotUCMode);
            this.overlayEl.style.display = 'flex';
            title.innerHTML = isUC
                ? "🌾 Étape 3 — Menu Principal des Missions (ROBOT_UC)"
                : "🌾 Étape 3 — Choix du Type de Mission";
            subtitle.innerText = "Sélectionnez la mission agricole ou la conduite manuelle (détection & évitement d'obstacles IA actifs en permanence)";

            body.innerHTML = `
                <div class="wizard-grid-4">
                    <!-- Carte 1 : Semence Initiale -->
                    <div class="wizard-card-option ${this.selectedMission === 'seeding' ? 'active' : ''}" data-mission="seeding">
                        <div>
                            <div class="wizard-card-icon">🌱</div>
                            <div class="wizard-card-title">
                                <span>🌱 1. Semence Initiale</span>
                                <span class="wizard-card-tag" style="color:#00e5ff; border-color:#00e5ff; background:rgba(0,229,255,0.12);">Semoir M8 • 46 cm</span>
                            </div>
                            <div class="wizard-card-desc">
                                Parcelle de terre vierge. Les 2 tuyaux de semence arrière M8 (-23 cm et +23 cm) déposent 2 rangées simultanées écartées de 46 cm avec évitement d'obstacles actif.
                            </div>
                        </div>
                        <div style="color: #00e5ff; font-weight: 700; font-size: 13px;">Ouvrir 1. Semence Initiale ➔</div>
                    </div>

                    <!-- Carte 2 : Révision Semence -->
                    <div class="wizard-card-option ${this.selectedMission === 'reseeding' ? 'active' : ''}" data-mission="reseeding">
                        <div>
                            <div class="wizard-card-icon">🔍</div>
                            <div class="wizard-card-title">
                                <span>🔍 2. Révision Semence</span>
                                <span class="wizard-card-tag" style="color:#eab308; border-color:#eab308; background:rgba(234,179,8,0.12);">Resemence Ciblée</span>
                            </div>
                            <div class="wizard-card-desc">
                                Scan IA par Caméra USB (78°) : détection des manques de germination (> 25 cm) et des plants absents. Resemence chirurgicale synchronisée avec les tuyaux arrière M8.
                            </div>
                        </div>
                        <div style="color: #eab308; font-weight: 700; font-size: 13px;">Ouvrir 2. Révision Semence ➔</div>
                    </div>

                    <!-- Carte 3 : Désherbage IA -->
                    <div class="wizard-card-option ${this.selectedMission === 'weeding' ? 'active' : ''}" data-mission="weeding">
                        <div>
                            <div class="wizard-card-icon">🌿</div>
                            <div class="wizard-card-title">
                                <span>🌿 3. Désherbage IA</span>
                                <span class="wizard-card-tag" style="color:#10b981; border-color:#10b981; background:rgba(16,185,129,0.12);">Fraise M6 + Dents M5</span>
                            </div>
                            <div class="wizard-card-desc">
                                Rangs d'arachide vivants espacés de 46 cm avec présence d'adventices. La fraise M6 et les dents M5 travaillent les couloirs inter-rangs avec protection obstacles permanente.
                            </div>
                        </div>
                        <div style="color: #10b981; font-weight: 700; font-size: 13px;">Ouvrir 3. Désherbage IA ➔</div>
                    </div>

                    <!-- Carte 4 : Conduite Manuelle -->
                    <div class="wizard-card-option ${this.selectedMission === 'manual' ? 'active' : ''}" data-mission="manual">
                        <div>
                            <div class="wizard-card-icon">🕹️</div>
                            <div class="wizard-card-title">
                                <span>🕹️ 4. Conduite Manuelle</span>
                                <span class="wizard-card-tag" style="color:#94a3b8; border-color:#94a3b8; background:rgba(148,163,184,0.12);">Pilotage Direct</span>
                            </div>
                            <div class="wizard-card-desc">
                                Pilotage manuel direct des moteurs M1 à M8, déploiement des panneaux solaires M7, vidange M8, calibration NVS et retour vidéo Caméra T'nB (78°) avec détection d'obstacles.
                            </div>
                        </div>
                        <div style="color: #94a3b8; font-weight: 700; font-size: 13px;">Ouvrir 4. Conduite Manuelle ➔</div>
                    </div>
                </div>
            `;

            if (isUC) {
                footer.innerHTML = `
                    <div style="display:flex; gap:8px; flex-wrap:wrap;">
                        <button type="button" class="btn-wizard-back" id="btnWizUcBackToRobot">🔌 Étape 1 : Robot &amp; Calibration</button>
                        <button type="button" class="btn-wizard-back" id="btnWizUcBackToCamera" style="border-color:#38bdf8; color:#7dd3fc;">📷 Étape 2 : Angle Caméra</button>
                    </div>
                    <div style="font-size: 12px; color: #6ee7b7; font-weight: 700;">🤖 Mode Embarqué ROBOT_UC — Caméra T'nB (78°)</div>
                `;
                document.getElementById('btnWizUcBackToRobot').onclick = () => this.goToStep(1);
                document.getElementById('btnWizUcBackToCamera').onclick = () => this.goToStep(2);
            } else {
                footer.innerHTML = `
                    <button type="button" class="btn-wizard-back" id="btnWizBackToStep2">⬅ Retour</button>
                    <div style="font-size: 12px; color: #94a3b8;">Cliquez sur la mission souhaitée</div>
                `;

                document.getElementById('btnWizBackToStep2').onclick = () => {
                    if (this.executionMode === 'COUPLED' || this.executionMode === 'REAL_ZERO_SIM') {
                        this.goToStep(2);
                    } else {
                        this.goToStep(1);
                    }
                };
            }

            const cards = body.querySelectorAll('.wizard-card-option');
            cards.forEach(card => {
                card.onclick = () => {
                    const mission = card.getAttribute('data-mission');
                    this.selectMissionAndOpenWorkspace(mission);
                };
            });

        } else if (step === 4) {
            // Étape 4 : Fermer le modal du wizard et afficher l'espace de travail épuré
            this.overlayEl.style.display = 'none';

            // Recalibrer immédiatement le canvas 3D WebGL pour éviter l'écran 0x0
            requestAnimationFrame(() => {
                window.dispatchEvent(new Event('resize'));
                if (typeof window.onWindowResize === 'function') {
                    window.onWindowResize();
                }
            });
        }
    }

    _renderUCStep1RobotConnection(title, subtitle, body, footer) {
        this.overlayEl.style.display = 'flex';
        title.innerHTML = "🔌 Étape 1/4 — Connexion au Vrai Robot (ESP32-S3) &amp; Calibration";
        subtitle.innerText = "Établissez la liaison ESP32-S3 (USB 115200 bauds ou Wi-Fi 192.168.4.1) et vérifiez la logique de déplacement & calibration NVS avant le champ";

        body.innerHTML = `
            <div class="hw-connection-panel">
                <div style="display:grid; grid-template-columns: 1fr 1.6fr; gap: 14px;">
                    <div class="hw-connection-field">
                        <label>Canal de Liaison ESP32-S3</label>
                        <select id="wizUcTransport">
                            <option value="USB_UART" selected>🔌 USB-UART Série (115 200 bauds)</option>
                            <option value="WIFI_HTTP">📶 Wi-Fi HTTP (192.168.4.1)</option>
                            <option value="SOCKET_TCP_WS">🌐 Socket TCP / WebSocket</option>
                        </select>
                    </div>
                    <div class="hw-connection-field">
                        <label>Port Série Détecté (/dev/ttyACM* / /dev/ttyUSB*) ou IP ESP32</label>
                        <div style="display:flex; gap:6px;">
                            <select id="wizUcPortSelect" style="flex:1;">
                                <option value="/dev/ttyACM0">/dev/ttyACM0 (Scan en cours...)</option>
                            </select>
                            <input type="text" id="wizUcTargetInput" value="/dev/ttyACM0" placeholder="/dev/ttyACM0 ou 192.168.4.1" style="flex:1;">
                            <button type="button" class="btn-wizard-back" id="btnWizUcScanPorts" style="padding:8px 12px;" title="Scanner les ports USB">🔄</button>
                        </div>
                    </div>
                </div>

                <div style="display:flex; gap:10px; flex-wrap:wrap; margin-top:6px;">
                    <button type="button" class="btn-wizard-next" id="btnWizUcConnectVerify" style="flex:2; min-width:240px;">
                        ⚡ Connecter &amp; Vérifier (PING → PONG, VER, STATUS)
                    </button>
                    <button type="button" class="btn-wizard-back" id="btnWizUcOpenCalibration" style="flex:1.4; min-width:220px; border-color:#f59e0b; color:#fde68a; background:rgba(245,158,11,0.12); font-weight:800;">
                        ⚙️ Réglage Logique Déplacement &amp; Calibration NVS (M1–M8)
                    </button>
                </div>

                <div id="wizHwStatusBox" class="hw-status-box ${this.hwConnected ? 'connected' : 'disconnected'}" style="margin-top:12px;">
                    <span id="wizHwStatusText">
                        ${this.hwConnected ? "✅ ESP32-S3 CONNECTÉ &amp; VÉRIFIÉ (Firmware v2.1)" : "🛑 NON CONNECTÉ — Branchez l'ESP32-S3 en USB ou Wi-Fi puis cliquez sur Connecter"}
                    </span>
                    <span id="wizHwPingBadge" style="font-family:monospace; font-size:11px;">
                        ${this.hwConnected ? "PING: PONG OK" : "En attente"}
                    </span>
                </div>

                <div id="wizUcVerifyDetails" style="margin-top:10px; padding:10px 12px; border-radius:8px; background:#050b14; border:1px solid #1e293b; font-family:Consolas,monospace; font-size:11.5px; color:#94a3b8; line-height:1.5;">
                    Vérification avant le champ : testez ARM/ESTOP, sens INV:n:0|1, seuils MIN/MAX, RAMP, TMAX:5 / TMAX:7, tests M1..M8 à 20% et DRIVE:30,0 / DRIVE:0,30 dans le panneau de calibration.
                </div>

                <div style="display:flex; justify-content:space-between; align-items:center; margin-top:10px; gap:10px; flex-wrap:wrap;">
                    <button type="button" class="btn-wizard-back" id="btnWizUcOpenGeometry" style="font-size:12px; padding:7px 12px; border-color:#10b981; color:#6ee7b7;">
                        📐 Géométrie Châssis &amp; Outils (robot_config)
                    </button>
                    <span style="font-size:11.5px; color:#64748b;">Sauvegarde NVS persistante sur Flash ESP32-S3 (SAVE)</span>
                </div>
            </div>
        `;

        footer.innerHTML = `
            <button type="button" class="btn-wizard-back" id="btnWizUcVisionDryRun" style="border-color:transparent; color:#94a3b8; font-size:12px; text-decoration:underline;">
                Continuer en Mode Test Vision sans ESP32
            </button>
            <button type="button" class="btn-wizard-next" id="btnWizUcNextCamera" ${!this.hwConnected ? 'disabled' : ''}>
                Suivant : Réglage Caméra →
            </button>
        `;

        const transportEl = document.getElementById('wizUcTransport');
        const portSel = document.getElementById('wizUcPortSelect');
        const targetInp = document.getElementById('wizUcTargetInput');
        const scanBtn = document.getElementById('btnWizUcScanPorts');
        const connectBtn = document.getElementById('btnWizUcConnectVerify');
        const nextCamBtn = document.getElementById('btnWizUcNextCamera');
        const statusBox = document.getElementById('wizHwStatusBox');
        const statusText = document.getElementById('wizHwStatusText');
        const pingBadge = document.getElementById('wizHwPingBadge');
        const verifyBox = document.getElementById('wizUcVerifyDetails');

        const scanPorts = async () => {
            try {
                const res = await fetch('/api/hardware/ports', { cache: 'no-store' });
                if (!res.ok) return;
                const data = await res.json();
                const ports = Array.isArray(data.ports) ? data.ports : [];
                if (!portSel) return;
                portSel.innerHTML = '';
                if (ports.length === 0) {
                    const opt = document.createElement('option');
                    opt.value = '/dev/ttyACM0';
                    opt.textContent = '/dev/ttyACM0 (aucun port série détecté)';
                    portSel.appendChild(opt);
                } else {
                    ports.forEach((p) => {
                        const opt = document.createElement('option');
                        opt.value = p.device;
                        opt.textContent = `${p.device} (${p.description || 'UART'})`;
                        portSel.appendChild(opt);
                    });
                    if (transportEl.value === 'USB_UART') {
                        portSel.value = ports[0].device;
                        targetInp.value = ports[0].device;
                    }
                }
            } catch (_) {}
        };

        const syncCurrentHwStatus = async () => {
            try {
                const res = await fetch('/api/hardware/status', { cache: 'no-store' });
                if (!res.ok) return;
                const hw = await res.json();
                if (hw && hw.connected) {
                    this.hwConnected = true;
                    statusBox.className = 'hw-status-box connected';
                    statusText.innerText = `✅ ESP32-S3 CONNECTÉ (${hw.transport === 'USB_UART' ? hw.serial_port : hw.host}) — ${hw.firmware_ver || 'Firmware v2.1'}`;
                    pingBadge.innerText = `PING: ${hw.latency_ms || 0} ms`;
                    if (nextCamBtn) nextCamBtn.disabled = false;
                    if (verifyBox) {
                        verifyBox.style.color = '#6ee7b7';
                        verifyBox.innerText = `VER: ${hw.firmware_ver || 'ESP32-S3 v2.1'} | ARMED=${hw.armed ? 1 : 0} | Liaison active et vérifiée.`;
                    }
                }
            } catch (_) {}
        };

        portSel.onchange = () => {
            if (portSel.value) targetInp.value = portSel.value;
        };
        transportEl.onchange = () => {
            if (transportEl.value === 'USB_UART') {
                portSel.style.display = '';
                targetInp.value = portSel.value || '/dev/ttyACM0';
            } else if (transportEl.value === 'WIFI_HTTP') {
                portSel.style.display = 'none';
                targetInp.value = '192.168.4.1';
            } else {
                portSel.style.display = 'none';
                targetInp.value = '192.168.4.1:80';
            }
        };
        scanBtn.onclick = scanPorts;

        document.getElementById('btnWizUcOpenCalibration').onclick = () => {
            if (typeof window.openHardwareConfigModal === 'function') {
                window.openHardwareConfigModal();
            }
        };
        document.getElementById('btnWizUcOpenGeometry').onclick = () => {
            if (typeof window.openDynamicSpecsModal === 'function') {
                window.openDynamicSpecsModal();
            }
        };
        document.getElementById('btnWizUcVisionDryRun').onclick = () => {
            this.goToStep(2);
        };
        nextCamBtn.onclick = () => {
            if (this.hwConnected) {
                this.goToStep(2);
            }
        };

        connectBtn.onclick = async () => {
            statusBox.className = 'hw-status-box testing';
            statusText.innerText = "⏳ Connexion & Vérification PING → PONG, VER, STATUS...";
            pingBadge.innerText = "...";
            connectBtn.disabled = true;
            try {
                const payload = {
                    transport: transportEl.value,
                    target: targetInp.value.trim(),
                    serial_port: transportEl.value === 'USB_UART' ? targetInp.value.trim() : undefined,
                    host: transportEl.value !== 'USB_UART' ? targetInp.value.trim().split(':')[0] : undefined,
                    baudrate: 115200,
                    execution_mode: 'REAL_ZERO_SIM',
                };
                const res = this.onHardwareConnect
                    ? await this.onHardwareConnect(payload)
                    : await (await fetch('/api/hardware/connect', {
                        method: 'POST',
                        headers: { 'Content-Type': 'application/json' },
                        body: JSON.stringify(payload),
                    })).json();

                if (res && res.connected) {
                    this.hwConnected = true;
                    statusBox.className = 'hw-status-box connected';
                    statusText.innerText = `✅ ESP32-S3 CONNECTÉ & VÉRIFIÉ (${res.firmware_ver || 'Firmware v2.1'})`;
                    pingBadge.innerText = `PONG (${res.latency_ms || 0} ms)`;
                    nextCamBtn.disabled = false;
                    verifyBox.style.color = '#6ee7b7';
                    verifyBox.innerText = `PONG OK (${res.latency_ms || 0} ms) | VER: ${res.firmware_ver || 'v2.1'} | STATUS: ARMED=${res.armed ? 1 : 0}`;
                } else {
                    this.hwConnected = false;
                    statusBox.className = 'hw-status-box disconnected';
                    const reason = (res && (res.last_error || res.error)) ? ` (${res.last_error || res.error})` : '';
                    statusText.innerText = `❌ ÉCHEC CONNEXION${reason}`;
                    pingBadge.innerText = "ÉCHEC";
                    nextCamBtn.disabled = true;
                    verifyBox.style.color = '#fda4af';
                    verifyBox.innerText = `Impossible de joindre l'ESP32-S3 sur ${targetInp.value}. Vérifiez le câble USB (/dev/ttyACM0 ou /dev/ttyUSB0) ou utilisez « Mode Test Vision sans ESP32 ».`;
                }
            } catch (err) {
                this.hwConnected = false;
                statusBox.className = 'hw-status-box disconnected';
                statusText.innerText = "❌ ERREUR RÉSEAU / PORT : " + err.message;
                nextCamBtn.disabled = true;
            }
            connectBtn.disabled = false;
        };

        scanPorts();
        syncCurrentHwStatus();
    }

    _renderUCStep2CameraCalibration(title, subtitle, body, footer) {
        this.overlayEl.style.display = 'flex';
        title.innerHTML = "📷 Étape 2/4 — Réglage &amp; Calibration de la Caméra (T'nB INCAM1080A)";
        subtitle.innerText = "Sélectionnez /dev/video0, vérifiez l'aperçu vidéo en direct et choisissez l'angle d'inclinaison caméra (manuel ou auto)";

        body.innerHTML = `
            <div style="display:grid; grid-template-columns: 1.05fr 1.15fr; gap: 16px; align-items: start;">
                <!-- COLONNE GAUCHE : PÉRIPHÉRIQUE & ANGLE CAMÉRA -->
                <div class="hw-connection-panel" style="padding:16px;">
                    <div class="hw-connection-field">
                        <label>Périphérique Vidéo USB (HP t530 : /dev/video0 prioritaire)</label>
                        <div style="display:flex; gap:6px;">
                            <select id="wizCamDeviceSelect" style="flex:1;">
                                <option value="0">/dev/video0 (Scan en cours...)</option>
                            </select>
                            <button type="button" class="btn-wizard-back" id="btnWizCamScan" style="padding:8px 10px;" title="Scanner caméras">🔄</button>
                            <button type="button" class="btn-wizard-next" id="btnWizCamConnect" style="padding:8px 12px; font-size:12px;">⚡ Connecter</button>
                        </div>
                    </div>

                    <div id="wizCamStatusLine" style="margin-bottom:12px; padding:8px 10px; border-radius:8px; background:rgba(15,23,42,0.8); border:1px solid #334155; font-size:11.5px; color:#cbd5e1; font-weight:700;">
                        ⏳ Lecture statut caméra...
                    </div>

                    <div class="hw-connection-field" style="margin-bottom:10px;">
                        <label>Mode de Réglage de l'Angle d'Inclinaison (<code>camera_pitch_deg</code>)</label>
                        <div style="display:grid; grid-template-columns: 1fr 1fr; gap:8px;">
                            <label style="display:flex; align-items:center; gap:6px; background:#0b1526; border:1px solid #243b61; border-radius:8px; padding:9px 10px; cursor:pointer; margin:0;">
                                <input type="radio" name="wizPitchMode" id="wizPitchModeManual" value="manual" style="width:auto; accent-color:#00e5ff;">
                                <span style="font-size:12px; color:#f8fafc;">📐 Angle Manuel</span>
                            </label>
                            <label style="display:flex; align-items:center; gap:6px; background:#0b1526; border:1px solid #243b61; border-radius:8px; padding:9px 10px; cursor:pointer; margin:0;">
                                <input type="radio" name="wizPitchMode" id="wizPitchModeAuto" value="auto" checked style="width:auto; accent-color:#10b981;">
                                <span style="font-size:12px; color:#f8fafc;">🎯 Calcul Automatique</span>
                            </label>
                        </div>
                    </div>

                    <!-- ZONE ANGLE MANUEL -->
                    <div id="wizManualPitchBox" style="padding:10px 12px; border-radius:8px; background:#07101f; border:1px solid #1e3a5f; margin-bottom:10px;">
                        <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:6px;">
                            <span style="font-size:12px; color:#cbd5e1; font-weight:600;">Angle d'inclinaison (5° – 80°)</span>
                            <div style="display:flex; align-items:center; gap:4px;">
                                <input type="number" id="wizCamPitchInput" min="5" max="80" step="0.5" value="32" style="width:72px; padding:5px 8px; text-align:right; font-family:monospace;">
                                <span style="font-size:12px; color:#00e5ff; font-weight:800;">°</span>
                            </div>
                        </div>
                        <input type="range" id="wizCamPitchSlider" min="5" max="80" step="0.5" value="32" style="width:100%; accent-color:#00e5ff; cursor:pointer;">
                    </div>

                    <!-- ZONE CALCUL AUTOMATIQUE -->
                    <div id="wizAutoPitchBox" style="padding:10px 12px; border-radius:8px; background:#061a14; border:1px solid #065f46; margin-bottom:10px;">
                        <div style="display:flex; gap:6px; flex-wrap:wrap; margin-bottom:8px;">
                            <button type="button" class="btn-wizard-next" id="btnWizCalibVanishing" style="flex:1; padding:8px 10px; font-size:11.5px; background:linear-gradient(135deg,#059669,#10b981); color:#02200c;">
                                🎯 Point de Fuite (y_vp)
                            </button>
                            <button type="button" class="btn-wizard-back" id="btnWizCalibMarker" style="flex:1; padding:8px 10px; font-size:11.5px; border-color:#10b981; color:#6ee7b7;">
                                📏 Mire au sol (1,00 m)
                            </button>
                        </div>
                        <div id="wizCamComputedAngleBadge" style="font-family:Consolas,monospace; font-size:12px; font-weight:800; color:#34d399;">
                            📐 Angle actif : 32.0° (Auto)
                        </div>
                    </div>

                    <!-- HAUTEUR CAMÉRA -->
                    <div class="hw-connection-field" style="margin-bottom:0;">
                        <div style="display:flex; justify-content:space-between; align-items:center;">
                            <label style="margin:0;">Hauteur caméra au sol (<code>camera_height_m</code>)</label>
                            <div style="display:flex; align-items:center; gap:5px;">
                                <input type="number" id="wizCamHeightInput" min="0.20" max="2.50" step="0.01" value="0.90" style="width:82px; padding:6px 8px; text-align:right; font-family:monospace;">
                                <span style="font-size:12px; color:#94a3b8; font-weight:700;">m</span>
                            </div>
                        </div>
                    </div>
                </div>

                <!-- COLONNE DROITE : FLUX MJPEG EN DIRECT -->
                <div class="hw-connection-panel" style="padding:14px; display:flex; flex-direction:column;">
                    <div style="font-size:12.5px; font-weight:800; color:#38bdf8; margin-bottom:8px; display:flex; justify-content:space-between;">
                        <span>👁️ Aperçu Live Caméra T'nB (78°) + Projection Sol</span>
                        <span id="wizCamLiveAngleChip" style="font-family:monospace; color:#34d399;">32.0°</span>
                    </div>
                    <div style="width:100%; aspect-ratio:16/9; background:#000; border-radius:10px; overflow:hidden; border:1px solid #1e293b; display:flex; align-items:center; justify-content:center;">
                        <img id="wizCamLiveStream" src="/api/camera/stream" style="width:100%; height:100%; object-fit:contain; display:block;" alt="Aperçu caméra temps réel" />
                    </div>
                    <div style="margin-top:8px; font-size:11px; color:#94a3b8; display:flex; justify-content:space-between;">
                        <span>Couloir bleu : Châssis 1,21 m</span>
                        <span style="color:#34d399;">Repères verts : Outils ±23 cm</span>
                    </div>
                </div>
            </div>
        `;

        footer.innerHTML = `
            <button type="button" class="btn-wizard-back" id="btnWizUcBackStep1">⬅ Retour : Connexion Robot</button>
            <button type="button" class="btn-wizard-next" id="btnWizUcSaveNextMission">
                Suivant : Choix de la Mission →
            </button>
        `;

        const devSel = document.getElementById('wizCamDeviceSelect');
        const statusLine = document.getElementById('wizCamStatusLine');
        const radioManual = document.getElementById('wizPitchModeManual');
        const radioAuto = document.getElementById('wizPitchModeAuto');
        const manualBox = document.getElementById('wizManualPitchBox');
        const autoBox = document.getElementById('wizAutoPitchBox');
        const pitchInp = document.getElementById('wizCamPitchInput');
        const pitchSlider = document.getElementById('wizCamPitchSlider');
        const heightInp = document.getElementById('wizCamHeightInput');
        const computedBadge = document.getElementById('wizCamComputedAngleBadge');
        const angleChip = document.getElementById('wizCamLiveAngleChip');

        const updateAngleModeUI = () => {
            const isAuto = Boolean(radioAuto && radioAuto.checked);
            if (manualBox) manualBox.style.opacity = isAuto ? '0.6' : '1.0';
            if (autoBox) autoBox.style.opacity = isAuto ? '1.0' : '0.65';
            const pVal = parseFloat(pitchInp.value) || 32.0;
            const hVal = parseFloat(heightInp.value) || 0.90;
            if (angleChip) angleChip.textContent = `${pVal.toFixed(1)}° (${isAuto ? 'Auto' : 'Manuel'}) • H=${hVal.toFixed(2)}m`;
            if (computedBadge) computedBadge.textContent = `📐 Angle actif : ${pVal.toFixed(1)}° (${isAuto ? 'Auto' : 'Manuel'}) • H=${hVal.toFixed(2)} m`;
        };

        const persistCameraAngleLive = async () => {
            const isAuto = Boolean(radioAuto && radioAuto.checked);
            const pitchDeg = Math.max(5.0, Math.min(80.0, parseFloat(pitchInp.value) || 32.0));
            const heightM = Math.max(0.20, Math.min(2.50, parseFloat(heightInp.value) || 0.90));
            updateAngleModeUI();
            const specsPayload = {
                camera_pitch_deg: pitchDeg,
                camera_height_m: heightM,
                auto_pitch: isAuto,
            };
            try {
                await Promise.all([
                    fetch('/api/config/specs', {
                        method: 'PUT',
                        headers: { 'Content-Type': 'application/json' },
                        body: JSON.stringify(specsPayload),
                    }),
                    fetch('/api/camera/config', {
                        method: 'POST',
                        headers: { 'Content-Type': 'application/json' },
                        body: JSON.stringify({
                            camera_pitch_deg: pitchDeg,
                            camera_tilt_deg: pitchDeg,
                            camera_height_m: heightM,
                            auto_pitch: isAuto,
                        }),
                    }),
                ]);
            } catch (_) {}
        };

        const loadDevicesAndConfig = async () => {
            try {
                const [devRes, specRes, camRes] = await Promise.all([
                    fetch('/api/camera/devices', { cache: 'no-store' }),
                    fetch('/api/config/specs', { cache: 'no-store' }),
                    fetch('/api/camera/status', { cache: 'no-store' }),
                ]);
                if (devRes.ok) {
                    const dJson = await devRes.json();
                    const devs = Array.isArray(dJson.devices) ? dJson.devices : [];
                    devSel.innerHTML = '';
                    devs.forEach((d) => {
                        const opt = document.createElement('option');
                        opt.value = d.index;
                        opt.textContent = d.name;
                        devSel.appendChild(opt);
                    });
                }
                if (specRes.ok) {
                    const sp = await specRes.json();
                    const p = Number(sp.camera_pitch_deg ?? 32.0);
                    const h = Number(sp.camera_height_m ?? 0.90);
                    const ap = Boolean(sp.auto_pitch ?? true);
                    pitchInp.value = p.toFixed(1);
                    pitchSlider.value = p.toFixed(1);
                    heightInp.value = h.toFixed(2);
                    radioAuto.checked = ap;
                    radioManual.checked = !ap;
                    updateAngleModeUI();
                }
                if (camRes.ok) {
                    const cam = await camRes.json();
                    if (typeof window.applyCameraStatusUI === 'function') {
                        window.applyCameraStatusUI(cam);
                    }
                    if (cam.connected && !cam.is_synthetic) {
                        statusLine.style.borderColor = '#10b981';
                        statusLine.style.color = '#6ee7b7';
                        statusLine.textContent = `🟢 Caméra réelle active : ${cam.device_name} (${cam.resolution})`;
                    } else if (cam.connected && cam.is_synthetic) {
                        statusLine.style.borderColor = '#f59e0b';
                        statusLine.style.color = '#fde68a';
                        statusLine.textContent = `⚠️ Flux synthétique de secours actif (${cam.device_path}) — Vérifiez le branchement USB`;
                    } else {
                        statusLine.style.borderColor = '#ef4444';
                        statusLine.style.color = '#fca5a5';
                        statusLine.textContent = `🔴 Caméra non connectée — Cliquez sur « ⚡ Connecter »`;
                    }
                }
            } catch (_) {}
        };

        pitchSlider.oninput = () => {
            pitchInp.value = parseFloat(pitchSlider.value).toFixed(1);
            radioManual.checked = true;
            radioAuto.checked = false;
            persistCameraAngleLive();
        };
        pitchInp.onchange = () => {
            const v = Math.max(5, Math.min(80, parseFloat(pitchInp.value) || 32));
            pitchInp.value = v.toFixed(1);
            pitchSlider.value = v.toFixed(1);
            radioManual.checked = true;
            radioAuto.checked = false;
            persistCameraAngleLive();
        };
        heightInp.onchange = () => persistCameraAngleLive();
        radioManual.onchange = () => persistCameraAngleLive();
        radioAuto.onchange = () => persistCameraAngleLive();

        document.getElementById('btnWizCamScan').onclick = loadDevicesAndConfig;

        document.getElementById('btnWizCamConnect').onclick = async () => {
            const idx = parseInt(devSel.value, 10);
            statusLine.textContent = `⏳ Connexion à /dev/video${Number.isNaN(idx) ? 0 : idx}...`;
            try {
                const res = await fetch('/api/camera/connect', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({
                        device_index: Number.isNaN(idx) ? 0 : idx,
                        width: 1280,
                        height: 720,
                        fps: 30,
                        camera_pitch_deg: parseFloat(pitchInp.value) || 32.0,
                        camera_height_m: parseFloat(heightInp.value) || 0.90,
                        auto_pitch: Boolean(radioAuto.checked),
                    }),
                });
                if (res.ok) {
                    const cam = await res.json();
                    if (typeof window.applyCameraStatusUI === 'function') {
                        window.applyCameraStatusUI(cam);
                    }
                    const img = document.getElementById('wizCamLiveStream');
                    if (img) img.src = `/api/camera/stream?t=${Date.now()}`;
                    await loadDevicesAndConfig();
                }
            } catch (e) {
                statusLine.textContent = `❌ Erreur caméra : ${e.message}`;
            }
        };

        const runAutoCalib = async (mode) => {
            radioAuto.checked = true;
            radioManual.checked = false;
            computedBadge.textContent = "⏳ Calibration automatique en cours...";
            try {
                const res = await fetch('/api/camera/calibrate-pitch', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({
                        mode: mode,
                        marker_distance_m: 1.0,
                        camera_height_m: parseFloat(heightInp.value) || 0.90,
                    }),
                });
                if (res.ok) {
                    const data = await res.json();
                    const p = Number(data.camera_pitch_deg ?? 32.0);
                    pitchInp.value = p.toFixed(1);
                    pitchSlider.value = p.toFixed(1);
                    updateAngleModeUI();
                    computedBadge.textContent = `✅ Angle calculé (${data.source || mode}) : ${p.toFixed(1)}° (H=${Number(data.camera_height_m || 0.90).toFixed(2)} m)`;
                }
            } catch (e) {
                computedBadge.textContent = `❌ Erreur calibration : ${e.message}`;
            }
        };

        document.getElementById('btnWizCalibVanishing').onclick = () => runAutoCalib('vanishing_point');
        document.getElementById('btnWizCalibMarker').onclick = () => runAutoCalib('ground_marker');

        document.getElementById('btnWizUcBackStep1').onclick = () => this.goToStep(1);
        document.getElementById('btnWizUcSaveNextMission').onclick = async () => {
            await persistCameraAngleLive();
            this.goToStep(3);
        };

        loadDevicesAndConfig();
    }

    selectMissionAndOpenWorkspace(missionName) {
        if (window.isMissionRunning) {
            if (typeof window.showMissionLockAlert === 'function') {
                window.showMissionLockAlert();
            }
            return;
        }
        this.selectedMission = missionName;
        this.goToStep(4); // Masquer le wizard

        if (this.onMissionChange) {
            this.onMissionChange(missionName);
        }

        // Mettre à jour l'affichage épuré de l'espace de travail
        this.applyCleanWorkspaceLayout(missionName);

        // Déclencher un deuxième recalibrage après injection DOM des panneaux
        setTimeout(() => {
            window.dispatchEvent(new Event('resize'));
            if (typeof window.onWindowResize === 'function') {
                window.onWindowResize();
            }
        }, 60);
    }

    applyCleanWorkspaceLayout(missionName) {
        // 1. Mise à jour du titre de la vue active dans le Header & badge lecture seule du cockpit vidéo
        const titles = {
            'seeding': '🌱 1. Semence Initiale',
            'reseeding': '🔍 2. Révision Semence',
            'weeding': '🌿 3. Désherbage IA',
            'manual': '🕹️ 4. Conduite Manuelle'
        };
        const activeLabel = titles[missionName] || '🌱 1. Semence Initiale';
        const titleEl = document.getElementById('hdrActiveViewTitle');
        if (titleEl) {
            titleEl.textContent = activeLabel;
        }
        const cockpitBadge = document.getElementById('realCockpitActiveMissionBadge');
        if (cockpitBadge) {
            cockpitBadge.textContent = `Mission active : ${activeLabel}`;
        }

        // 2. Gestion de l'affichage exclusif des panneaux latéraux
        const panelWeeding = document.getElementById('panelWeedingMission');
        const panelSeeding = document.getElementById('panelSeedingMission');
        const panelReseeding = document.getElementById('panelReseedingMission');
        const panelManual = document.getElementById('panelManualMission');
        const isManual = (missionName === 'manual');

        if (panelWeeding) panelWeeding.style.display = (missionName === 'weeding') ? 'flex' : 'none';
        if (panelSeeding) panelSeeding.style.display = (missionName === 'seeding') ? 'flex' : 'none';
        if (panelReseeding) panelReseeding.style.display = (missionName === 'reseeding') ? 'flex' : 'none';
        if (panelManual) panelManual.style.display = isManual ? 'flex' : 'none';

        // Masquer les contrôles de mission autonome (Parcelle & Démarrer la Mission) en Conduite Manuelle,
        // et les réafficher automatiquement dès qu'une des 3 missions autonomes est sélectionnée.
        document.querySelectorAll('.autonomous-only-control, .autonomous-parcel-card, .btn-launch-mission-cta').forEach(el => {
            el.style.display = isManual ? 'none' : '';
        });

        // 3. Réinitialiser la vue en ÉTAT A (Préparation) pour la mission active
        if (typeof window.setMissionUIState === 'function') {
            window.setMissionUIState('IDLE');
        }
    }

    activateMissionInDashboard(missionName) {
        this.selectMissionAndOpenWorkspace(missionName);
    }

    bindGlobalEvents() {
        // Raccourci Échap pour revenir au choix de mission si on est dans l'espace de travail (hors mission en cours)
        window.addEventListener('keydown', (e) => {
            if (e.key === 'Escape' && this.currentStep === 4) {
                if (window.isMissionRunning) {
                    if (typeof window.showMissionLockAlert === 'function') {
                        window.showMissionLockAlert();
                    }
                    return;
                }
                this.goToStep(3);
            }
        });
    }

    setHardwareStatus(connected) {
        this.hwConnected = Boolean(connected);
        const statusBox = document.getElementById('wizHwStatusBox');
        const statusText = document.getElementById('wizHwStatusText');
        const continueBtn = document.getElementById('btnWizContinueToStep3') || document.getElementById('btnWizUcNextCamera');
        if (statusBox && statusText) {
            statusBox.className = 'hw-status-box ' + (this.hwConnected ? 'connected' : 'disconnected');
            statusText.innerText = this.hwConnected
                ? "✅ LIAISON ACTIVE AVEC L'ESP32-S3 (Firmware v2.1)"
                : "🛑 NON CONNECTÉ — En attente du signal ESP32-S3";
        }
        if (continueBtn) {
            continueBtn.disabled = !this.hwConnected;
        }
    }
}

