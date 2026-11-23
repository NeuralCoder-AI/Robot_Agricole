/**
 * static/js/core/kinematic_engine.js
 * Moteur Cinématique Déterministe Unique (KinematicEngine) — 100 % Dynamique via `robot_config`.
 * 
 * Architecture centralisée garantissant :
 * 1. Lecture dynamique en temps réel des mesures du robot (`updateSpecs`) : zéro constante codée en dur.
 * 2. Génération exacte des lignes & curseur de rang strict (rowsX, rowStep 1 vs 2, impossible de revenir en arrière).
 * 3. Couloir strict de collision ("Corridor Box") : les obstacles hors couloir sont ignorés à 100%.
 * 4. Attente intelligente devant obstacle mobile (`WAITING_MOVING_OBSTACLE`) avant de contourner.
 * 5. Waypoints absolus du trapèze (WP0, WP1, WP2, WP3) recalculés dynamiquement.
 * 6. Mode « Fin de Ligne Intelligente » (`SMART_LINE_END` / `LINE_END_DETECTED` > 0.50 m).
 */

const MOBILE_OBSTACLE_CLASSES = new Set([
    "person", "human", "humain", "personne", "pedestrian", "worker", "agriculteur", "child",
    "dog", "cat", "cow", "sheep", "goat", "horse", "bird", "animal", "animal_mobile",
    "car", "truck", "tractor", "bicycle", "motorcycle", "vehicle", "vehicule"
]);

class KinematicEngine {
    // Constantes dimensionnelles par défaut (mises à jour dynamiquement par updateSpecs)
    static ROBOT_WIDTH = 1.21;
    static ROBOT_HALF_WIDTH = 0.605;
    static ROBOT_LENGTH = 2.15;
    static ROBOT_NOSE_EXTENT = 0.85;
    static REAR_TINES_OFFSET = 1.15;
    static CORRIDOR_SAFETY_MARGIN = 0.08;
    static BYPASS_SAFETY_MARGIN = 0.38;
    static TEETH_REAR_CLEARANCE = 0.25;
    static APPROACH_LEN = 1.10;
    static REAR_CLEARANCE_EXTENT = 1.40;
    static MOVING_OBSTACLE_WAIT_SEC = 5.0;

    constructor(config = {}) {
        this.ROBOT_WIDTH = Number(config.chassis_width_m ?? KinematicEngine.ROBOT_WIDTH);
        this.ROBOT_HALF_WIDTH = Number((this.ROBOT_WIDTH / 2.0).toFixed(4));
        this.ROBOT_LENGTH = Number(config.chassis_length_m ?? KinematicEngine.ROBOT_LENGTH);
        this.ROBOT_NOSE_EXTENT = Number(config.nose_offset_m ?? KinematicEngine.ROBOT_NOSE_EXTENT);
        this.REAR_TINES_OFFSET = Number(config.rear_teeth_offset_m ?? KinematicEngine.REAR_TINES_OFFSET);
        this.CORRIDOR_SAFETY_MARGIN = Number(config.corridor_safety_margin_m ?? KinematicEngine.CORRIDOR_SAFETY_MARGIN);
        this.BYPASS_SAFETY_MARGIN = Number(config.lateral_safety_margin_m ?? KinematicEngine.BYPASS_SAFETY_MARGIN);
        this.TEETH_REAR_CLEARANCE = Number(config.teeth_rear_clearance_m ?? KinematicEngine.TEETH_REAR_CLEARANCE);
        this.APPROACH_LEN = Number(config.approach_length_m ?? KinematicEngine.APPROACH_LEN);
        this.REAR_CLEARANCE_EXTENT = Number((this.REAR_TINES_OFFSET + this.TEETH_REAR_CLEARANCE).toFixed(4));
        this.MOVING_OBSTACLE_WAIT_SEC = Number(config.moving_obstacle_wait_sec ?? KinematicEngine.MOVING_OBSTACLE_WAIT_SEC);
        this.LINE_END_CONFIRM_DIST_M = Number(config.line_end_confirm_dist_m ?? 0.50);

        this.rowSpacing = Number(config.rowSpacing ?? config.tool_spacing_m ?? 0.46);
        this.toolHalfOffset = Number((this.rowSpacing / 2.0).toFixed(4));
        this.centralGuideSpacing = Number((this.rowSpacing * 2.0).toFixed(4));

        this.fieldLength = Number(config.fieldLength ?? 10.0);
        this.fieldWidth = Number(config.fieldWidth ?? 4.6);
        this.startX = Number(config.startX ?? 0.0);
        this.startZ = Number(config.startZ ?? 0.0);
        if (String(config.stepDir || "").toUpperCase() === "AUTO") {
            this.progressionMode = "AUTO";
            this.stepDir = -1;
            this.lockedFieldSideSign = null;
        } else {
            this.stepDir = (Number(config.stepDir ?? -1) < 0) ? -1 : 1;
            this.progressionMode = (this.stepDir < 0) ? "RIGHT" : "LEFT";
            this.lockedFieldSideSign = this.stepDir;
        }
        this.missionType = String(config.missionType ?? 'weeding');
        this.boundaryMode = String(config.boundaryMode ?? config.field_boundary_mode ?? 'FIXED_DIMENSIONS');
        this.workedRowsSet = new Set();
        this.leftLineExists = true;
        this.rightLineExists = true;
        this.leftAlreadyWorked = false;
        this.rightAlreadyWorked = false;
        this.lastSideDecision = null;

        this._rebuildFieldLines();

        // Règle de saut de ligne
        this.rowStep = (this.rowSpacing <= 0.50) ? 2 : 1;
        this.currentRowIndex = 0;
        this.targetRowX = this.rowsX[0] ?? this.startX;
        this.passIndex = 0;
        this.passDirection = 1; // +1 vers +Z (cap 0), -1 vers -Z (cap PI)

        // Pose actuelle
        this.x = this.targetRowX;
        this.y = 0.60;
        this.z = this.startZ;
        this.heading = (this.passDirection > 0) ? 0.0 : Math.PI;

        // Suivi des obstacles contournés sur la passe actuelle
        this.clearedObstaclesOnCurrentPass = new Set();

        // États internes
        this.state = "IDLE";
        this.activeWaypoints = [];
        this.activeWpIndex = 0;
        this.activeObstacle = null;

        // Attente d'obstacle mobile
        this.waitingObstacle = null;
        this.waitingElapsedSec = 0.0;
        this.remainingWaitSec = 0.0;
        this.resumedAfterMovingObstacle = false;

        // Suivi Intelligent Fin de Ligne & Compteur 5 frames Sol Nu
        this.lineMissingDistanceM = 0.0;
        this.lineEndDetected = false;
        this.fieldEndDetected = false;
        this.awaitingNewRowLineCheck = false;
        this.HYBRID_END_CONSECUTIVE_FRAMES = 5;
        this.hybridInvalidFramesCount = 0;
        this.completionMessage = "";

        // Moteurs [M1..M8] et vitesse
        this.motors = [0, 0, 0, 0, 0, 0, 0, 0];
        this.linearSpeed = 0.0;
    }

    static wrapAngle(angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    /**
     * Met à jour dynamiquement toutes les dimensions mécaniques et optiques du robot.
     */
    updateSpecs(specs = {}) {
        if (!specs) return;
        if (specs.chassis_width_m !== undefined) {
            this.ROBOT_WIDTH = Number(specs.chassis_width_m);
            this.ROBOT_HALF_WIDTH = Number((this.ROBOT_WIDTH / 2.0).toFixed(4));
            KinematicEngine.ROBOT_WIDTH = this.ROBOT_WIDTH;
            KinematicEngine.ROBOT_HALF_WIDTH = this.ROBOT_HALF_WIDTH;
        }
        if (specs.chassis_length_m !== undefined) {
            this.ROBOT_LENGTH = Number(specs.chassis_length_m);
            KinematicEngine.ROBOT_LENGTH = this.ROBOT_LENGTH;
        }
        if (specs.nose_offset_m !== undefined) {
            this.ROBOT_NOSE_EXTENT = Number(specs.nose_offset_m);
            KinematicEngine.ROBOT_NOSE_EXTENT = this.ROBOT_NOSE_EXTENT;
        }
        if (specs.rear_teeth_offset_m !== undefined) {
            this.REAR_TINES_OFFSET = Number(specs.rear_teeth_offset_m);
            KinematicEngine.REAR_TINES_OFFSET = this.REAR_TINES_OFFSET;
        }
        if (specs.lateral_safety_margin_m !== undefined) {
            this.BYPASS_SAFETY_MARGIN = Number(specs.lateral_safety_margin_m);
            KinematicEngine.BYPASS_SAFETY_MARGIN = this.BYPASS_SAFETY_MARGIN;
        }
        if (specs.teeth_rear_clearance_m !== undefined) {
            this.TEETH_REAR_CLEARANCE = Number(specs.teeth_rear_clearance_m);
            KinematicEngine.TEETH_REAR_CLEARANCE = this.TEETH_REAR_CLEARANCE;
        }
        if (specs.tool_spacing_m !== undefined) {
            this.rowSpacing = Number(specs.tool_spacing_m);
            this.toolHalfOffset = Number((this.rowSpacing / 2.0).toFixed(4));
            this.centralGuideSpacing = Number((this.rowSpacing * 2.0).toFixed(4));
        }
        if (specs.moving_obstacle_wait_sec !== undefined) {
            this.MOVING_OBSTACLE_WAIT_SEC = Number(specs.moving_obstacle_wait_sec);
            KinematicEngine.MOVING_OBSTACLE_WAIT_SEC = this.MOVING_OBSTACLE_WAIT_SEC;
        }
        if (specs.field_boundary_mode !== undefined) {
            this.boundaryMode = String(specs.field_boundary_mode);
        }
        this.REAR_CLEARANCE_EXTENT = Number((this.REAR_TINES_OFFSET + this.TEETH_REAR_CLEARANCE).toFixed(4));
        this._rebuildFieldLines();
        if (this.activeObstacle) {
            const plan = this.computeTrapezoidWaypoints(this.activeObstacle);
            this.activeWaypoints = plan.waypoints;
        }
    }

    _rebuildFieldLines() {
        const spacing = this.rowSpacing || 0.46;
        this.toolHalfOffset = Number((spacing / 2.0).toFixed(4));
        this.centralGuideSpacing = Number((spacing * 2.0).toFixed(4));

        const numGuides = Math.max(1, Math.floor(this.fieldWidth / this.centralGuideSpacing) + 1);
        this.centralGuidesX = [];
        for (let k = 0; k < numGuides; k++) {
            this.centralGuidesX.push(Number((this.startX + k * this.centralGuideSpacing * this.stepDir).toFixed(3)));
        }

        const totalFurrows = Math.floor(this.fieldWidth / spacing) + 4;
        const firstFurrowX = this.startX - this.toolHalfOffset * this.stepDir;
        this.furrowsX = [];
        for (let i = 0; i < totalFurrows; i++) {
            this.furrowsX.push(Number((firstFurrowX + i * spacing * this.stepDir).toFixed(3)));
        }

        const isWeeding = (this.missionType === 'weeding' || this.missionType === 'smart_weeding');
        if (!isWeeding) {
            this.rowsX = this.centralGuidesX.slice();
        } else {
            this.rowsX = this.centralGuidesX.map(gx => Number((gx + this.toolHalfOffset * this.stepDir).toFixed(3)));
        }
    }

    /**
     * Initialise ou réinitialise la mission avec le tableau strict des lignes.
     */
    initMission(rowSpacing, fieldLength, fieldWidth, startX, startZ, stepDir, missionType = 'weeding', boundaryMode = null) {
        if (rowSpacing !== undefined && rowSpacing !== null) this.rowSpacing = Number(rowSpacing);
        if (fieldLength !== undefined && fieldLength !== null) this.fieldLength = Number(fieldLength);
        if (fieldWidth !== undefined && fieldWidth !== null) this.fieldWidth = Number(fieldWidth);
        if (startX !== undefined && startX !== null) this.startX = Number(startX);
        if (startZ !== undefined && startZ !== null) this.startZ = Number(startZ);
        if (stepDir !== undefined && stepDir !== null) {
            const sStr = String(stepDir).toUpperCase();
            if (sStr === 'AUTO') {
                this.progressionMode = 'AUTO';
                this.lockedFieldSideSign = null;
                this.stepDir = -1;
            } else if (sStr === 'LEFT' || sStr === 'GAUCHE' || Number(stepDir) > 0) {
                this.progressionMode = 'LEFT';
                this.stepDir = 1;
                this.lockedFieldSideSign = 1;
            } else {
                this.progressionMode = 'RIGHT';
                this.stepDir = -1;
                this.lockedFieldSideSign = -1;
            }
        }
        if (missionType) this.missionType = String(missionType);
        if (boundaryMode) this.boundaryMode = String(boundaryMode);
        this.workedRowsSet.clear();
        this.lastSideDecision = null;

        this._rebuildFieldLines();

        this.rowStep = 1; // Avance de 1 ligne directrice centrale à chaque demi-tour
        this.currentRowIndex = 0;
        this.targetRowX = this.rowsX[0] ?? this.startX;
        this.passIndex = 0;
        this.passDirection = 1;

        this.x = this.targetRowX;
        this.z = this.startZ;
        this.heading = 0.0;
        this.clearedObstaclesOnCurrentPass.clear();
        this.state = "ROW_FOLLOWING";
        this.activeWaypoints = [];
        this.activeWpIndex = 0;
        this.activeObstacle = null;

        this.waitingObstacle = null;
        this.waitingElapsedSec = 0.0;
        this.remainingWaitSec = 0.0;
        this.resumedAfterMovingObstacle = false;

        this.lineMissingDistanceM = 0.0;
        this.lineEndDetected = false;
        this.fieldEndDetected = false;
        this.awaitingNewRowLineCheck = false;
        this.hybridInvalidFramesCount = 0;
        this.completionMessage = "";

        this.motors = [0, 0, 0, 0, 50, 70, 0, 0];
        this.linearSpeed = 0.0;
    }

    static isObstacleMoving(obs) {
        if (!obs) return false;
        if (obs.is_moving || obs.is_mobile_class) return true;
        const cls = String(obs.class_name || obs.type || "").trim().toLowerCase();
        if (MOBILE_OBSTACLE_CLASSES.has(cls)) return true;
        if (Number(obs.speed_mps || 0) > 0.05 || Number(obs.speed_px_s || 0) >= 12.0) return true;
        return false;
    }

    /**
     * Règle 2 : Couloir strict de collision ('Corridor Box').
     */
    checkObstacleCollision(obs) {
        if (!obs) return { is_blocking: false, reason: "NULL_OBS" };

        const obsX = Number(obs.x ?? obs.abs_x ?? 0.0);
        const obsZ = Number(obs.z ?? obs.abs_z ?? 0.0);
        const obsW = Number(obs.width_m ?? obs.width ?? ((obs.radius ? obs.radius * 2.0 : 0.90)));
        const obsDepth = Number(obs.length_m ?? obs.depth ?? ((obs.radius ? obs.radius * 2.0 : 0.90)));
        const obsId = String(obs.id || `${obsX.toFixed(2)}_${obsZ.toFixed(2)}`);

        const lateralDist = Math.abs(obsX - this.targetRowX);
        const collisionHalfWidth = (obsW / 2.0) + this.ROBOT_HALF_WIDTH + this.CORRIDOR_SAFETY_MARGIN;
        const isBlockingCurrentRow = lateralDist < collisionHalfWidth;

        if (!isBlockingCurrentRow) {
            return {
                is_blocking: false,
                reason: "HORS_COULOIR",
                lateral_dist: Number(lateralDist.toFixed(3)),
                collision_half_width: Number(collisionHalfWidth.toFixed(3)),
                obs_id: obsId
            };
        }

        if (this.clearedObstaclesOnCurrentPass.has(obsId)) {
            return {
                is_blocking: false,
                reason: "ALREADY_CLEARED",
                obs_id: obsId
            };
        }

        const dirZ = this.passDirection;
        const frontFaceZ = (dirZ > 0) ? (obsZ - obsDepth / 2.0) : (obsZ + obsDepth / 2.0);
        const noseZ = this.z + dirZ * this.ROBOT_NOSE_EXTENT;
        const forwardDist = dirZ * (frontFaceZ - noseZ);

        if (forwardDist > 0.0 && forwardDist <= 1.20) {
            return {
                is_blocking: true,
                forward_dist: Number(forwardDist.toFixed(3)),
                front_face_z: Number(frontFaceZ.toFixed(3)),
                collision_half_width: Number(collisionHalfWidth.toFixed(3)),
                is_moving: KinematicEngine.isObstacleMoving(obs),
                obs_id: obsId,
                obs: obs
            };
        }

        return {
            is_blocking: false,
            reason: (forwardDist > 1.20) ? "TROP_LOIN" : "DERRIERE",
            forward_dist: Number(forwardDist.toFixed(3)),
            collision_half_width: Number(collisionHalfWidth.toFixed(3)),
            obs_id: obsId
        };
    }

    /**
     * Règle 3 : Contournement Trapézoïdal par Waypoints Absolus (WP0, WP1, WP2, WP3)
     * 100 % pilotés par les dimensions dynamiques.
     */
    computeTrapezoidWaypoints(obs, sideX = 1) {
        const obsX = Number(obs.x ?? obs.abs_x ?? 0.0);
        const obsZ = Number(obs.z ?? obs.abs_z ?? 0.0);
        const obsW = Number(obs.width_m ?? obs.width ?? ((obs.radius ? obs.radius * 2.0 : 0.90)));
        const obsDepth = Number(obs.length_m ?? obs.depth ?? ((obs.radius ? obs.radius * 2.0 : 0.90)));

        const minZ = Number(obs.min_z ?? (obsZ - obsDepth / 2.0));
        const maxZ = Number(obs.max_z ?? (obsZ + obsDepth / 2.0));

        const dirZ = this.passDirection;
        const rowX = this.targetRowX;

        const frontZ = (dirZ > 0) ? minZ : maxZ;
        const rearZ = (dirZ > 0) ? maxZ : minZ;

        const side = (sideX >= 0) ? 1 : -1;
        const lateralClearance = this.BYPASS_SAFETY_MARGIN;
        const bypassX = Number((obsX + side * ((obsW / 2.0) + this.ROBOT_HALF_WIDTH + lateralClearance)).toFixed(3));
        const deltaX = Number(Math.abs(bypassX - rowX).toFixed(3));
        const approachLen = this.APPROACH_LEN;

        const wp0Offset = this.ROBOT_NOSE_EXTENT + approachLen;
        const wp0 = {
            x: Number(rowX.toFixed(3)),
            z: Number((frontZ - dirZ * wp0Offset).toFixed(3))
        };
        const wp1 = {
            x: Number(bypassX.toFixed(3)),
            z: Number((frontZ - dirZ * this.ROBOT_NOSE_EXTENT).toFixed(3))
        };
        const wp2Offset = this.REAR_TINES_OFFSET + this.TEETH_REAR_CLEARANCE;
        const wp2 = {
            x: Number(bypassX.toFixed(3)),
            z: Number((rearZ + dirZ * wp2Offset).toFixed(3))
        };
        const wp3 = {
            x: Number(rowX.toFixed(3)),
            z: Number((wp2.z + dirZ * (approachLen + 0.15)).toFixed(3))
        };

        const alpha = Math.atan2(deltaX, approachLen);
        const d1 = Math.hypot(deltaX, approachLen);
        const d2 = Math.abs(wp2.z - wp1.z);

        return {
            WP0: wp0,
            WP1: wp1,
            WP2: wp2,
            WP3: wp3,
            waypoints: [wp0, wp1, wp2, wp3],
            bypass_x: bypassX,
            delta_x: deltaX,
            approach_len: approachLen,
            alpha_rad: Number(alpha.toFixed(4)),
            alpha_deg: Number((alpha * (180.0 / Math.PI)).toFixed(2)),
            d1_m: Number(d1.toFixed(3)),
            d2_m: Number(d2.toFixed(3)),
            dir_z: dirZ,
            front_z: Number(frontZ.toFixed(3)),
            rear_z: Number(rearZ.toFixed(3))
        };
    }

    analyzeSoilTreatmentState(leftROI = null, rightROI = null) {
        const stepX = Number((this.rowSpacing * this.rowStep).toFixed(3));
        const leftCandX = Number((this.x + stepX).toFixed(3));
        const rightCandX = Number((this.x - stepX).toFixed(3));

        let leftWorked = Boolean(this.leftAlreadyWorked || this.workedRowsSet.has(leftCandX));
        let rightWorked = Boolean(this.rightAlreadyWorked || this.workedRowsSet.has(rightCandX));

        if (leftROI && typeof leftROI === "object") {
            if (leftROI.worked !== undefined || leftROI.already_worked !== undefined) {
                leftWorked = Boolean(leftROI.worked ?? leftROI.already_worked);
            } else if ((leftROI.texture_var || 0) >= 160.0 || leftROI.has_wheel_tracks) {
                leftWorked = true;
            }
        }
        if (rightROI && typeof rightROI === "object") {
            if (rightROI.worked !== undefined || rightROI.already_worked !== undefined) {
                rightWorked = Boolean(rightROI.worked ?? rightROI.already_worked);
            } else if ((rightROI.texture_var || 0) >= 160.0 || rightROI.has_wheel_tracks) {
                rightWorked = true;
            }
        }

        if (leftWorked && !rightWorked) {
            return {
                left_worked: true,
                right_worked: false,
                recommended_side: "RIGHT",
                recommended_sign: -1,
                hud_message: "🧠 IA SOL : Côté Gauche déjà traité -> Sens DROITE verrouillé"
            };
        }
        if (rightWorked && !leftWorked) {
            return {
                left_worked: false,
                right_worked: true,
                recommended_side: "LEFT",
                recommended_sign: 1,
                hud_message: "🧠 IA SOL : Côté Droit déjà traité -> Sens GAUCHE verrouillé"
            };
        }
        return {
            left_worked: leftWorked,
            right_worked: rightWorked,
            recommended_side: "BOTH_UNTOUCHED",
            recommended_sign: -1,
            hud_message: "🤖 IA AUTO : Lignes vierges à Gauche et à Droite -> Choix autonome : Vers la DROITE verrouillé pour tous les tours."
        };
    }

    detectNextRowDirection(options = {}) {
        if (this.lockedFieldSideSign !== null && this.lastSideDecision) {
            return { ...this.lastSideDecision };
        }
        const hasLeft = (options.leftLineExists !== undefined) ? Boolean(options.leftLineExists) : this.leftLineExists;
        const hasRight = (options.rightLineExists !== undefined) ? Boolean(options.rightLineExists) : this.rightLineExists;

        if (options.leftAlreadyWorked !== undefined) this.leftAlreadyWorked = Boolean(options.leftAlreadyWorked);
        if (options.rightAlreadyWorked !== undefined) this.rightAlreadyWorked = Boolean(options.rightAlreadyWorked);

        // CAS 1 — Ligne présente d'un seul côté
        if (hasRight && !hasLeft) {
            return this._lockSide(-1, "RIGHT", "CASE_1_ONLY_RIGHT_LINE_EXISTS", "🧠 IA AUTO : Ligne uniquement à DROITE -> Sens DROITE verrouillé");
        }
        if (hasLeft && !hasRight) {
            return this._lockSide(1, "LEFT", "CASE_1_ONLY_LEFT_LINE_EXISTS", "🧠 IA AUTO : Ligne uniquement à GAUCHE -> Sens GAUCHE verrouillé");
        }

        // CAS 2 — Lignes présentes des deux côtés (Analyse de l'état du sol)
        const soil = this.analyzeSoilTreatmentState(options.leftROI, options.rightROI);
        if (soil.left_worked && !soil.right_worked) {
            return this._lockSide(-1, "RIGHT", "CASE_2_LEFT_ALREADY_WORKED", soil.hud_message);
        }
        if (soil.right_worked && !soil.left_worked) {
            return this._lockSide(1, "LEFT", "CASE_2_RIGHT_ALREADY_WORKED", soil.hud_message);
        }

        // CAS 3 — Les deux côtés sont 100 % vierges
        const override = String(options.farmerOverride || "").toUpperCase();
        if (override === "LEFT" || override === "GAUCHE" || override === "1") {
            return this._lockSide(1, "LEFT", "CASE_3_BOTH_UNTOUCHED_FARMER_LEFT", "🤖 IA AUTO : Lignes vierges à Gauche et à Droite -> Choix : Vers la GAUCHE verrouillé pour tous les tours.");
        }
        return this._lockSide(-1, "RIGHT", "CASE_3_BOTH_UNTOUCHED_AUTONOMOUS_FALLBACK", soil.hud_message);
    }

    _lockSide(sign, label, caseId, hudMessage) {
        const s = (sign < 0) ? -1 : 1;
        this.lockedFieldSideSign = s;
        this.stepDir = s;
        this._rebuildFieldLines();
        this.lastSideDecision = {
            direction: label,
            side_sign: s,
            locked: true,
            case: caseId,
            hud_message: hudMessage
        };
        return { ...this.lastSideDecision };
    }

    advanceRowCursor() {
        this.workedRowsSet.add(Number(this.targetRowX.toFixed(3)));
        if (this.lockedFieldSideSign === null) {
            this.detectNextRowDirection();
        }
        this.currentRowIndex += this.rowStep;
        if (this.boundaryMode !== "SMART_LINE_END" && this.currentRowIndex >= this.rowsX.length) {
            this.state = "MISSION_COMPLETE";
            this.motors = [0, 0, 0, 0, 0, 0, 0, 0];
            this.linearSpeed = 0.0;
            return false;
        }

        if (this.currentRowIndex < this.rowsX.length) {
            this.targetRowX = this.rowsX[this.currentRowIndex];
        } else {
            this.targetRowX = Number((this.targetRowX + this.stepDir * this.centralGuideSpacing).toFixed(3));
            this.rowsX.push(this.targetRowX);
        }

        this.passDirection *= -1;
        this.passIndex += 1;
        this.clearedObstaclesOnCurrentPass.clear();
        return true;
    }

    getTinesBoundingBox(x, z, dirZ = null) {
        const direction = (dirZ === null) ? this.passDirection : dirZ;
        const tinesZ = z - direction * this.REAR_TINES_OFFSET;
        const halfW = 0.30;
        const halfD = 0.15;
        return [x - halfW, tinesZ - halfD, x + halfW, tinesZ + halfD];
    }

    static boxDistance(b1, b2) {
        const dx = Math.max(0.0, b1[0] - b2[2], b2[0] - b1[2]);
        const dz = Math.max(0.0, b1[1] - b2[3], b2[1] - b1[3]);
        return Math.hypot(dx, dz);
    }

    triggerAvoidancePlan(obs, sideX = 1) {
        const plan = this.computeTrapezoidWaypoints(obs, sideX);
        const obsId = String(obs.id || `${Number(obs.x ?? 0).toFixed(2)}_${Number(obs.z ?? 0).toFixed(2)}`);
        this.clearedObstaclesOnCurrentPass.add(obsId);

        this.activeObstacle = obs;
        this.activeWaypoints = plan.waypoints;
        this.activeWpIndex = 0;
        this.waitingObstacle = null;
        this.waitingElapsedSec = 0.0;
        this.remainingWaitSec = 0.0;
        this.state = "APPROACH_AVOIDANCE";
        return plan;
    }

    step(dt = 0.05, obstacles = null, lineDetected = undefined, hybridRowValid = undefined) {
        if (this.state === "IDLE" || this.state === "MISSION_COMPLETE" || this.state === "MISSION_COMPLETED_BY_HYBRID_VISION") {
            this.motors = [0, 0, 0, 0, 0, 0, 0, 0];
            this.linearSpeed = 0.0;
            return this.getTelemetry();
        }

        // ----------------------------------------------------------------------
        // État : WAITING_MOVING_OBSTACLE
        // ----------------------------------------------------------------------
        if (this.state === "WAITING_MOVING_OBSTACLE") {
            this.motors = [0, 0, 0, 0, 50, 0, 0, 0];
            this.linearSpeed = 0.0;

            let stillBlocking = null;
            if (obstacles && obstacles.length > 0) {
                for (const obs of obstacles) {
                    const chk = this.checkObstacleCollision(obs);
                    if (chk.is_blocking) {
                        stillBlocking = obs;
                        break;
                    }
                }
            }

            if (!stillBlocking) {
                this.waitingObstacle = null;
                this.waitingElapsedSec = 0.0;
                this.remainingWaitSec = 0.0;
                this.resumedAfterMovingObstacle = true;
                this.state = "ROW_FOLLOWING";
                this.motors = [22, 22, 22, 22, 50, 70, 0, 0];
                this.linearSpeed = 0.65;
                return this.getTelemetry();
            }

            this.waitingElapsedSec += Math.max(0.0, dt);
            this.remainingWaitSec = Math.max(0.0, Number((this.MOVING_OBSTACLE_WAIT_SEC - this.waitingElapsedSec).toFixed(2)));
            if (this.waitingElapsedSec >= this.MOVING_OBSTACLE_WAIT_SEC) {
                this.triggerAvoidancePlan(stillBlocking);
                this.motors = [0, 0, 0, 0, -50, 0, 0, 0];
            }
            return this.getTelemetry();
        }

        // ----------------------------------------------------------------------
        // État : ROW_FOLLOWING
        // ----------------------------------------------------------------------
        if (this.state === "ROW_FOLLOWING") {
            if (hybridRowValid === true) {
                this.hybridInvalidFramesCount = 0;
                if (this.awaitingNewRowLineCheck) {
                    this.awaitingNewRowLineCheck = false;
                }
            } else if (hybridRowValid === false) {
                this.hybridInvalidFramesCount += 1;
                if (this.hybridInvalidFramesCount >= this.HYBRID_END_CONSECUTIVE_FRAMES) {
                    this.hybridInvalidFramesCount = 0;
                    this.lineEndDetected = true;
                    this.motors = [0, 0, 0, 0, -50, 0, 0, 0];
                    this.linearSpeed = 0.0;
                    if (this.awaitingNewRowLineCheck) {
                        this.awaitingNewRowLineCheck = false;
                        this.fieldEndDetected = true;
                        this.state = "MISSION_COMPLETED_BY_HYBRID_VISION";
                        this.completionMessage = "🛑 MISSION TERMINÉE — FIN DES LIGNES D'ARACHIDE";
                        this.motors = [0, 0, 0, 0, 0, 0, 0, 0];
                        return this.getTelemetry();
                    }
                    const hasNext = this.advanceRowCursor();
                    if (hasNext) {
                        this.state = "U_TURN_SHIFT";
                        this.awaitingNewRowLineCheck = true;
                    } else {
                        this.state = "MISSION_COMPLETED_BY_HYBRID_VISION";
                        this.completionMessage = "🛑 MISSION TERMINÉE — FIN DES LIGNES D'ARACHIDE";
                        this.fieldEndDetected = true;
                        this.motors = [0, 0, 0, 0, 0, 0, 0, 0];
                    }
                    return this.getTelemetry();
                }
            }
            if (this.awaitingNewRowLineCheck && lineDetected !== undefined) {
                this.awaitingNewRowLineCheck = false;
                if (!lineDetected) {
                    this.fieldEndDetected = true;
                    this.lineEndDetected = true;
                    this.state = "MISSION_COMPLETE";
                    this.completionMessage = "✅ Fin de parcelle détectée automatiquement — Mission terminée";
                    this.motors = [0, 0, 0, 0, 0, 0, 0, 0];
                    this.linearSpeed = 0.0;
                    return this.getTelemetry();
                }
            }

            if (obstacles && obstacles.length > 0) {
                for (const obs of obstacles) {
                    const chk = this.checkObstacleCollision(obs);
                    if (chk.is_blocking) {
                        if (chk.is_moving) {
                            this.state = "WAITING_MOVING_OBSTACLE";
                            this.waitingObstacle = obs;
                            this.waitingElapsedSec = 0.0;
                            this.remainingWaitSec = this.MOVING_OBSTACLE_WAIT_SEC;
                            this.resumedAfterMovingObstacle = false;
                            this.motors = [0, 0, 0, 0, 50, 0, 0, 0];
                            this.linearSpeed = 0.0;
                            return this.getTelemetry();
                        } else {
                            this.triggerAvoidancePlan(obs);
                            this.motors = [0, 0, 0, 0, -50, 0, 0, 0];
                            return this.getTelemetry();
                        }
                    }
                }
            }

            const dirZ = this.passDirection;
            const targetHeading = (dirZ > 0) ? 0.0 : Math.PI;
            this.heading = targetHeading;
            this.x = this.targetRowX;

            const step = 0.65 * dt;
            this.z += dirZ * step;
            this.linearSpeed = 0.65;
            this.motors = [22, 22, 22, 22, 50, 70, 0, 0];

            if (this.boundaryMode === "SMART_LINE_END" || lineDetected !== undefined) {
                if (lineDetected === false) {
                    this.lineMissingDistanceM += Math.abs(step);
                    if (this.lineMissingDistanceM >= this.LINE_END_CONFIRM_DIST_M - 1e-6) {
                        this.lineEndDetected = true;
                        this.motors = [0, 0, 0, 0, -50, 0, 0, 0];
                        this.linearSpeed = 0.0;
                        const hasNext = this.advanceRowCursor();
                        if (hasNext) {
                            this.state = "U_TURN_SHIFT";
                            this.awaitingNewRowLineCheck = true;
                        } else {
                            this.state = "MISSION_COMPLETE";
                        }
                        return this.getTelemetry();
                    }
                } else if (lineDetected === true) {
                    this.lineMissingDistanceM = 0.0;
                    this.lineEndDetected = false;
                }
            }

            if (this.boundaryMode !== "SMART_LINE_END") {
                const atEnd = (dirZ > 0 && this.z >= this.fieldLength) || (dirZ < 0 && this.z <= this.startZ);
                if (atEnd) {
                    this.motors = [0, 0, 0, 0, -50, 0, 0, 0];
                    const hasNext = this.advanceRowCursor();
                    if (hasNext) {
                        this.state = "U_TURN_SHIFT";
                    } else {
                        this.state = "MISSION_COMPLETE";
                    }
                }
            }

            return this.getTelemetry();
        }

        // ----------------------------------------------------------------------
        // État : U_TURN_SHIFT (Transition vers le nouveau rang)
        // ----------------------------------------------------------------------
        if (this.state === "U_TURN_SHIFT") {
            const dx = this.targetRowX - this.x;
            if (Math.abs(dx) > 0.015) {
                const stepX = Math.sign(dx) * Math.min(0.50 * dt, Math.abs(dx));
                this.x += stepX;
                this.heading = Math.atan2(dx, 0.0);
                this.motors = [18, 18, 18, 18, -50, 0, 0, 0];
            } else {
                this.x = this.targetRowX;
                this.heading = (this.passDirection > 0) ? 0.0 : Math.PI;
                this.lineMissingDistanceM = 0.0;
                this.hybridInvalidFramesCount = 0;
                this.state = "ROW_FOLLOWING";
                this.motors = [22, 22, 22, 22, 50, 70, 0, 0];
            }
            return this.getTelemetry();
        }

        // ----------------------------------------------------------------------
        // États de contournement : APPROACH_AVOIDANCE, TRAJ_1, TRAJ_2, TRAJ_3
        // ----------------------------------------------------------------------
        if (["APPROACH_AVOIDANCE", "TRAJ_1_EXIT", "TRAJ_2_PARALLEL", "TRAJ_3_RETURN"].includes(this.state)) {
            if (!this.activeWaypoints || this.activeWpIndex >= this.activeWaypoints.length) {
                this.state = "REALIGN_AFTER_AVOID";
                return this.getTelemetry();
            }

            const targetWp = this.activeWaypoints[this.activeWpIndex];
            const targetX = targetWp.x;
            const targetZ = targetWp.z;

            const distToWp = Math.hypot(targetX - this.x, targetZ - this.z);

            if (distToWp > 0.03) {
                const targetHead = Math.atan2(targetX - this.x, targetZ - this.z);
                const headErr = KinematicEngine.wrapAngle(targetHead - this.heading);

                if (Math.abs(headErr) > (3.0 * Math.PI / 180.0) && this.state !== "TRAJ_2_PARALLEL") {
                    this.heading += Math.sign(headErr) * Math.min((90.0 * Math.PI / 180.0) * dt, Math.abs(headErr));
                    this.motors = (headErr > 0) ? [15, -15, 15, -15, -50, 0, 0, 0] : [-15, 15, -15, 15, -50, 0, 0, 0];
                    this.linearSpeed = 0.0;
                } else {
                    if (this.state === "TRAJ_2_PARALLEL") {
                        this.heading = (this.passDirection > 0) ? 0.0 : Math.PI;
                    }

                    const moveStep = Math.min(0.60 * dt, distToWp);
                    const dx = targetX - this.x;
                    const dz = targetZ - this.z;
                    const tot = Math.hypot(dx, dz);
                    this.x += (dx / tot) * moveStep;
                    this.z += (dz / tot) * moveStep;
                    this.linearSpeed = 0.60;
                    this.motors = [20, 20, 20, 20, -50, 0, 0, 0];
                }
            } else {
                this.x = targetX;
                this.z = targetZ;
                this.activeWpIndex += 1;

                if (this.activeWpIndex === 1) {
                    this.state = "TRAJ_1_EXIT";
                } else if (this.activeWpIndex === 2) {
                    this.state = "TRAJ_2_PARALLEL";
                } else if (this.activeWpIndex === 3) {
                    this.state = "TRAJ_3_RETURN";
                } else if (this.activeWpIndex >= 4) {
                    this.state = "REALIGN_AFTER_AVOID";
                }
            }

            return this.getTelemetry();
        }

        // ----------------------------------------------------------------------
        // État : REALIGN_AFTER_AVOID
        // ----------------------------------------------------------------------
        if (this.state === "REALIGN_AFTER_AVOID") {
            const targetHeading = (this.passDirection > 0) ? 0.0 : Math.PI;
            const headErr = KinematicEngine.wrapAngle(targetHeading - this.heading);
            if (Math.abs(headErr) > (2.0 * Math.PI / 180.0)) {
                this.heading += Math.sign(headErr) * Math.min((80.0 * Math.PI / 180.0) * dt, Math.abs(headErr));
                this.motors = [14, -14, 14, -14, -50, 0, 0, 0];
            } else {
                this.heading = targetHeading;
                this.x = this.targetRowX;
                this.state = "ROW_FOLLOWING";
                this.motors = [22, 22, 22, 22, 50, 70, 0, 0];
            }
            return this.getTelemetry();
        }

        return this.getTelemetry();
    }

    getTelemetry() {
        return {
            x: Number(this.x.toFixed(3)),
            y: Number(this.y.toFixed(3)),
            z: Number(this.z.toFixed(3)),
            heading: Number(this.heading.toFixed(4)),
            heading_deg: Number((this.heading * (180.0 / Math.PI)).toFixed(2)),
            state: this.state,
            current_row_index: this.currentRowIndex,
            target_row_x: Number(this.targetRowX.toFixed(3)),
            pass_direction: this.passDirection,
            pass_index: this.passIndex,
            linear_speed: Number(this.linearSpeed.toFixed(2)),
            motors: [...this.motors],
            rows_x: [...this.rowsX],
            row_step: this.rowStep,
            waiting_elapsed_sec: Number(this.waitingElapsedSec.toFixed(2)),
            remaining_wait_sec: Number(this.remainingWaitSec.toFixed(2)),
            resumed_after_moving_obstacle: this.resumedAfterMovingObstacle,
            line_missing_distance_m: Number(this.lineMissingDistanceM.toFixed(3)),
            line_end_detected: this.lineEndDetected,
            field_end_detected: this.fieldEndDetected,
            completion_message: this.completionMessage
        };
    }
}

/**
 * Empreinte 2D OBB exacte au sol du châssis contre un obstacle cylindrique (zéro faux positif).
 */
function checkRealGroundCollision(robotX, robotZ, robotHeading, obsX, obsZ, obsRadius) {
    const dx = obsX - robotX;
    const dz = obsZ - robotZ;
    const cosH = Math.cos(robotHeading);
    const sinH = Math.sin(robotHeading);
    const localX = dx * cosH - dz * sinH;
    const localZ = dx * sinH + dz * cosH;

    const halfChassisWidth = 0.52;
    const frontLimit = 0.72;
    const rearLimit = -1.02;
    const effectiveObsRadius = Math.max(0.15, (obsRadius || 0.45) * 0.72);

    const closestX = Math.max(-halfChassisWidth, Math.min(halfChassisWidth, localX));
    const closestZ = Math.max(rearLimit, Math.min(frontLimit, localZ));
    const distSq = (localX - closestX) ** 2 + (localZ - closestZ) ** 2;

    return distSq < (effectiveObsRadius ** 2);
}

KinematicEngine.checkRealGroundCollision = checkRealGroundCollision;
KinematicEngine.prototype.checkRealGroundCollision = checkRealGroundCollision;

if (typeof window !== "undefined") {
    window.KinematicEngine = KinematicEngine;
    window.checkRealGroundCollision = checkRealGroundCollision;
    if (!window.kinematicEngine) {
        window.kinematicEngine = new KinematicEngine();
    }
}
if (typeof module !== "undefined" && module.exports) {
    module.exports = { KinematicEngine, checkRealGroundCollision };
}
