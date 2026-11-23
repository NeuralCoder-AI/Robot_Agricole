/**
 * static/js/ui/wizard_navigator.js — Gestionnaire de navigation fluide par Sous-Fenêtres (Wizard Épuré).
 * Étape 1 : Choix du Mode d'Exécution (Simulation Uniquement vs Simulation Couplée).
 * Étape 2 : Connexion Matérielle ESP32-S3 (si mode Couplé sélectionné).
 * Étape 3 : Choix de la Mission (Désherbage, Semence Initiale, Révision de la Semence, Mode Manuel).
 * Étape 4 : Espace de Travail Épuré (Scène 3D + Caméra IA + Panneau unique de la mission choisie).
 */

export class WizardNavigator {
    constructor(options = {}) {
        this.currentStep = 1; // 1, 2, 3 ou 4 (4 = Workspace)
        this.executionMode = 'SIMULATION_ONLY'; // 'SIMULATION_ONLY' ou 'COUPLED'
        this.selectedMission = 'weeding'; // 'weeding', 'seeding', 'reseeding', 'manual'
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
        this.selectMissionAndOpenWorkspace(this.selectedMission || 'seeding');
    }

    createWizardDOM() {
        if (document.getElementById('wizardOverlay')) return;

        const overlay = document.createElement('div');
        overlay.id = 'wizardOverlay';
        overlay.className = 'wizard-overlay';
        overlay.innerHTML = `
            <div class="wizard-box">
                <div class="wizard-header">
                    <div class="wizard-title-group">
                        <h2 id="wizardStepTitle">💻 Choix du Mode d'Exécution</h2>
                        <p id="wizardStepSubtitle">Configurez votre environnement avant de lancer le robot agricole</p>
                    </div>
                    <div class="wizard-steps-indicator">
                        <div class="wizard-step-dot" id="wizardDot1" title="Étape 1 : Mode"></div>
                        <div class="wizard-step-dot" id="wizardDot2" title="Étape 2 : Connexion"></div>
                        <div class="wizard-step-dot" id="wizardDot3" title="Étape 3 : Mission"></div>
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
        if ((this.robotUCMode || window.isRobotUCMode) && (step === 1 || step === 2)) {
            step = 3;
        }
        this.currentStep = step;
        const body = document.getElementById('wizardBody');
        const footer = document.getElementById('wizardFooter');
        const title = document.getElementById('wizardStepTitle');
        const subtitle = document.getElementById('wizardStepSubtitle');

        // Mettre à jour les indicateurs
        for (let i = 1; i <= 3; i++) {
            const dot = document.getElementById(`wizardDot${i}`);
            if (dot) {
                dot.className = 'wizard-step-dot' + (i === step ? ' active' : (i < step ? ' done' : ''));
            }
        }

        if (step === 1) {
            this.overlayEl.style.display = 'flex';
            title.innerHTML = "⚙️ Étape 1 — Choix du Mode d'Exécution";
            subtitle.innerText = "Sélectionnez votre mode : Simulation 3D, Hybride Couplé ou 100 % Réel Zéro Simulation (Caméra 170°)";

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
            subtitle.innerText = "Connectez votre robot ESP32-S3 maintenant ou passez au Cockpit pour configurer le Robot et la Caméra 170°";

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
            this.overlayEl.style.display = 'flex';
            title.innerHTML = "🌾 Étape 3 — Choix du Type de Mission";
            subtitle.innerText = "Sélectionnez l'opération agricole à exécuter sur la parcelle d'arachide";

            body.innerHTML = `
                <div class="wizard-grid-4">
                    <!-- Mission 1 : Désherbage -->
                    <div class="wizard-card-option ${this.selectedMission === 'weeding' ? 'active' : ''}" data-mission="weeding">
                        <div>
                            <div class="wizard-card-icon">🌿</div>
                            <div class="wizard-card-title">
                                <span>Désherbage Intelligent</span>
                                <span class="wizard-card-tag" style="color:#10b981; border-color:#10b981; background:rgba(16,185,129,0.12);">Fraise M6 + Dents M5</span>
                            </div>
                            <div class="wizard-card-desc">
                                Rangs d'arachide vivants espacés de 46 cm avec présence d'adventices. La fraise M6 et les dents M5 labourent les 2 couloirs inter-rangs sans toucher aux plants.
                            </div>
                        </div>
                        <div style="color: #10b981; font-weight: 700; font-size: 13px;">Ouvrir l'espace désherbage ➔</div>
                    </div>

                    <!-- Mission 2 : Semence Initiale -->
                    <div class="wizard-card-option ${this.selectedMission === 'seeding' ? 'active' : ''}" data-mission="seeding">
                        <div>
                            <div class="wizard-card-icon">🌱</div>
                            <div class="wizard-card-title">
                                <span>Semence Initiale</span>
                                <span class="wizard-card-tag" style="color:#00e5ff; border-color:#00e5ff; background:rgba(0,229,255,0.12);">Champ 100% Vierge</span>
                            </div>
                            <div class="wizard-card-desc">
                                Parcelle de terre 100 % vide. Les 2 tuyaux de semence arrière M8 (-23 cm et +23 cm) déposent 2 rangées simultanées écartées de 46 cm avec pas latéral de 92 cm.
                            </div>
                        </div>
                        <div style="color: #00e5ff; font-weight: 700; font-size: 13px;">Ouvrir l'espace semence ➔</div>
                    </div>

                    <!-- Mission 3 : Révision de la Semence -->
                    <div class="wizard-card-option ${this.selectedMission === 'reseeding' ? 'active' : ''}" data-mission="reseeding">
                        <div>
                            <div class="wizard-card-icon">🔍</div>
                            <div class="wizard-card-title">
                                <span>Révision de la Semence</span>
                                <span class="wizard-card-tag" style="color:#eab308; border-color:#eab308; background:rgba(234,179,8,0.12);">Resemence Ciblée</span>
                            </div>
                            <div class="wizard-card-desc">
                                Scan IA par caméra : détection des trous de germination (> 35 cm) et des plants morts. Resemence chirurgicale synchronisée avec les tuyaux arrière M8.
                            </div>
                        </div>
                        <div style="color: #eab308; font-weight: 700; font-size: 13px;">Ouvrir l'espace révision ➔</div>
                    </div>

                    <!-- Mission 4 : Mode Manuel -->
                    <div class="wizard-card-option ${this.selectedMission === 'manual' ? 'active' : ''}" data-mission="manual">
                        <div>
                            <div class="wizard-card-icon">🕹️</div>
                            <div class="wizard-card-title">
                                <span>Mode Manuel & Réglages</span>
                                <span class="wizard-card-tag" style="color:#94a3b8; border-color:#94a3b8; background:rgba(148,163,184,0.12);">Contrôle Total</span>
                            </div>
                            <div class="wizard-card-desc">
                                Pilotage direct des moteurs M1 à M8, déploiement des panneaux solaires M7, vidange de la trémie M8, calibration et sauvegarde NVS.
                            </div>
                        </div>
                        <div style="color: #94a3b8; font-weight: 700; font-size: 13px;">Ouvrir le cockpit manuel ➔</div>
                    </div>
                </div>
            `;

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
        // 1. Mise à jour du titre de la vue active dans le Header
        const titleEl = document.getElementById('hdrActiveViewTitle');
        if (titleEl) {
            const titles = {
                'weeding': '🌿 Désherbage Intelligent',
                'seeding': '🌱 Semence Initiale',
                'reseeding': '🔍 Révision de la Semence',
                'manual': '🕹️ Mode Manuel & Diagnostics'
            };
            titleEl.textContent = titles[missionName] || '🌾 Mission Agricole';
        }

        // 2. Gestion de l'affichage exclusif des panneaux latéraux
        const panelWeeding = document.getElementById('panelWeedingMission');
        const panelSeeding = document.getElementById('panelSeedingMission');
        const panelReseeding = document.getElementById('panelReseedingMission');
        const panelManual = document.getElementById('panelManualMission');

        if (panelWeeding) panelWeeding.style.display = (missionName === 'weeding') ? 'flex' : 'none';
        if (panelSeeding) panelSeeding.style.display = (missionName === 'seeding') ? 'flex' : 'none';
        if (panelReseeding) panelReseeding.style.display = (missionName === 'reseeding') ? 'flex' : 'none';
        if (panelManual) panelManual.style.display = (missionName === 'manual') ? 'flex' : 'none';

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
        const continueBtn = document.getElementById('btnWizContinueToStep3');
        if (statusBox && statusText) {
            statusBox.className = 'hw-status-box ' + (this.hwConnected ? 'connected' : 'disconnected');
            statusText.innerText = this.hwConnected
                ? "✅ LIAISON ACTIVE AVEC L'ESP32-S3 (Firmware v2)"
                : "🛑 NON CONNECTÉ — En attente du signal ESP32-S3";
        }
        if (continueBtn) {
            continueBtn.disabled = !this.hwConnected;
        }
    }
}

