/**
 * verify_box3_simulation.js
 * Test déterministe de simulation 500 frames vérifiant que la machine à états Box3
 * d'index.html évite rigoureusement l'obstacle sans AUCUNE collision :
 * robotBox.intersectsBox(obsBox) === false à chaque frame, à l'aller (+Z) et au retour (-Z).
 */

class Vector3 {
    constructor(x = 0, y = 0, z = 0) {
        this.x = x;
        this.y = y;
        this.z = z;
    }
}

class Box3 {
    constructor(min = null, max = null) {
        this.min = min || new Vector3(+Infinity, +Infinity, +Infinity);
        this.max = max || new Vector3(-Infinity, -Infinity, -Infinity);
    }

    setFromCenterAndSize(center, size) {
        this.min.x = center.x - size.x / 2;
        this.min.y = center.y - size.y / 2;
        this.min.z = center.z - size.z / 2;
        this.max.x = center.x + size.x / 2;
        this.max.y = center.y + size.y / 2;
        this.max.z = center.z + size.z / 2;
        return this;
    }

    intersectsBox(box) {
        if (this.corners) {
            return Box3.satIntersect(this.corners, box);
        }
        return !(
            box.max.x < this.min.x || box.min.x > this.max.x ||
            box.max.y < this.min.y || box.min.y > this.max.y ||
            box.max.z < this.min.z || box.min.z > this.max.z
        );
    }

    static satIntersect(poly, box) {
        const boxCorners = [
            { x: box.min.x, z: box.min.z },
            { x: box.max.x, z: box.min.z },
            { x: box.max.x, z: box.max.z },
            { x: box.min.x, z: box.max.z }
        ];
        function getAxes(pts) {
            const axes = [];
            for (let i = 0; i < pts.length; i++) {
                const p1 = pts[i];
                const p2 = pts[(i + 1) % pts.length];
                const edge = { x: p2.x - p1.x, z: p2.z - p1.z };
                const normal = { x: -edge.z, z: edge.x };
                const len = Math.hypot(normal.x, normal.z);
                axes.push({ x: normal.x / len, z: normal.z / len });
            }
            return axes;
        }
        const axes = [...getAxes(poly), { x: 1, z: 0 }, { x: 0, z: 1 }];
        for (const ax of axes) {
            let minP = Infinity, maxP = -Infinity;
            for (const p of poly) {
                const dot = p.x * ax.x + p.z * ax.z;
                minP = Math.min(minP, dot);
                maxP = Math.max(maxP, dot);
            }
            let minB = Infinity, maxB = -Infinity;
            for (const b of boxCorners) {
                const dot = b.x * ax.x + b.z * ax.z;
                minB = Math.min(minB, dot);
                maxB = Math.max(maxB, dot);
            }
            if (maxP < minB || maxB < minP) return false;
        }
        return true;
    }

    setFromObject(obj) {
        if (obj.isObstacle) {
            this.corners = null;
            return this.setFromCenterAndSize(
                new Vector3(obj.position.x, 0, obj.position.z),
                new Vector3(obj.width, 1.0, obj.depth)
            );
        } else {
            // Empreinte orientée exacte du châssis au sol (largeur 1.21 m, nez 0.85 m, dents 1.15 m)
            const halfW = 0.605;
            const noseOffset = 0.85;
            const rearOffset = 1.15;
            const yaw = obj.rotation || 0.0;
            const cosY = Math.cos(yaw);
            const sinY = Math.sin(yaw);

            const corners = [
                { x: obj.position.x - cosY * halfW + sinY * noseOffset, z: obj.position.z + sinY * halfW + cosY * noseOffset },
                { x: obj.position.x + cosY * halfW + sinY * noseOffset, z: obj.position.z - sinY * halfW + cosY * noseOffset },
                { x: obj.position.x + cosY * halfW - sinY * rearOffset, z: obj.position.z - sinY * halfW - cosY * rearOffset },
                { x: obj.position.x - cosY * halfW - sinY * rearOffset, z: obj.position.z + sinY * halfW - cosY * rearOffset }
            ];
            this.corners = corners;

            let minX = Infinity, maxX = -Infinity, minZ = Infinity, maxZ = -Infinity;
            for (const p of corners) {
                if (p.x < minX) minX = p.x;
                if (p.x > maxX) maxX = p.x;
                if (p.z < minZ) minZ = p.z;
                if (p.z > maxZ) maxZ = p.z;
            }
            this.min = new Vector3(minX, 0, minZ);
            this.max = new Vector3(maxX, 1.0, maxZ);
            return this;
        }
    }
}

const THREE = { Vector3, Box3 };

function runSimulationScenario(directionName, dirZ, startX, startZ, obsX, obsZ, obsW, obsD) {
    console.log(`\n========================================================================`);
    console.log(`  TEST SCÉNARIO : ${directionName} (dirZ = ${dirZ})`);
    console.log(`  Obstacle : [x: ${obsX.toFixed(2)}, z: ${obsZ.toFixed(2)}, taille: ${obsW.toFixed(2)}m x ${obsD.toFixed(2)}m]`);
    console.log(`========================================================================`);

    const robotGroup = {
        position: new Vector3(startX, 0.6, startZ),
        isObstacle: false
    };

    let robotX = startX;
    let robotZ = startZ;
    let robotYaw = (dirZ > 0) ? 0.0 : Math.PI;
    let tinesDepth = 1.0;
    let latestMotors = [22, 22, 22, 22, 50, 70, 0, 0];
    let currentSpeedKmh = 2.3;

    const robotState = { missionActive: true };

    const obstacleObj = {
        id: `rock_${directionName}`,
        position: new Vector3(obsX, 0, obsZ),
        width: obsW,
        depth: obsD,
        isObstacle: true
    };
    const allObstacles = [obstacleObj];

    const nav3D = {
        state: "FOLLOWING_ROW",
        currentRowIndex: 0,
        currentRowX: startX,
        rowsX: [startX],
        rowStep: 2,
        dirZ: dirZ,
        fieldLengthZ: 14.0,
        startZ: 0.0,
        activeObs: null,
        activeObsBox: null,
        wp0_z: 0.0,
        wp1: { x: 0.0, z: 0.0 },
        wp2_z: 0.0,
        wp3: { x: 0.0, z: 0.0 },
        bypassX: 0.0,
        frontZ: 0.0,
        rearZ: 0.0,
        speedMps: 0.65,
        clearedObstacleIds: new Set()
    };

    const obsBox = new THREE.Box3().setFromObject(obstacleObj);

    let collisionsCount = 0;
    let minDistanceRecorded = 999.0;
    const dt = 0.05; // 20 Hz
    const traceFrames = [];

    for (let frame = 1; frame <= 500; frame++) {
        // --- MISE À JOUR EXACTE DE LA MACHINE NAV3D (identique à index.html) ---
        // Dimensions mécaniques fixes au sol (sans gonflement de Box3 en rotation)
        const halfRobotWidth = 0.605;
        const noseOffset = 0.85;
        const rearTeethOffset = 1.15;
        const lateralClearance = 0.22;
        const teethClearance = 0.20;
        const corridorMargin = 0.08;

        if (nav3D.state === "FOLLOWING_ROW") {
            let blockingObs = null;
            let blockingObsBox = null;
            let minFrontDist = 999.0;

            for (let i = 0; i < allObstacles.length; i++) {
                const obs = allObstacles[i];
                const obsId = String(obs.id || `obs_${i}`);
                if (nav3D.clearedObstacleIds.has(obsId)) continue;

                const oBox = new THREE.Box3().setFromObject(obs);
                const corridorMinX = oBox.min.x - halfRobotWidth - corridorMargin;
                const corridorMaxX = oBox.max.x + halfRobotWidth + corridorMargin;
                const isBlockingCorridor = (nav3D.currentRowX >= corridorMinX && nav3D.currentRowX <= corridorMaxX);

                if (!isBlockingCorridor) continue;

                const frontZ = (dirZ > 0) ? oBox.min.z : oBox.max.z;
                const distCenterToFront = dirZ * (frontZ - robotZ);

                if (distCenterToFront > -0.2 && distCenterToFront <= 1.35) {
                    if (distCenterToFront < minFrontDist) {
                        minFrontDist = distCenterToFront;
                        blockingObs = obs;
                        blockingObsBox = oBox;
                    }
                }
            }

            if (blockingObs && blockingObsBox) {
                const frontZ = (dirZ > 0) ? blockingObsBox.min.z : blockingObsBox.max.z;
                const rearZ = (dirZ > 0) ? blockingObsBox.max.z : blockingObsBox.min.z;
                nav3D.activeObs = blockingObs;
                nav3D.activeObsBox = blockingObsBox;
                nav3D.frontZ = frontZ;
                nav3D.rearZ = rearZ;
                nav3D.wp0_z = frontZ - dirZ * 1.95;

                const obsCenterX = (blockingObsBox.min.x + blockingObsBox.max.x) / 2.0;
                const side = (obsCenterX >= nav3D.currentRowX) ? -1 : 1;
                const lateralClearance = 0.38;
                const bypassX = (side > 0)
                    ? (blockingObsBox.max.x + halfRobotWidth + lateralClearance)
                    : (blockingObsBox.min.x - halfRobotWidth - lateralClearance);
                nav3D.bypassX = bypassX;

                nav3D.wp1 = { x: bypassX, z: frontZ - dirZ * 0.85 };
                const WP2_z = rearZ + dirZ * 1.38;
                nav3D.wp2_z = WP2_z;
                nav3D.wp3 = { x: nav3D.currentRowX, z: WP2_z + dirZ * 1.25 };

                nav3D.state = "STOP_AND_LIFT";
            } else {
                robotX = nav3D.currentRowX;
                robotYaw = (dirZ > 0) ? 0.0 : Math.PI;
                robotZ += dirZ * (nav3D.speedMps * dt);
            }
        } else if (nav3D.state === "STOP_AND_LIFT") {
            tinesDepth = 0.0;
            robotX = nav3D.currentRowX;
            robotYaw = (dirZ > 0) ? 0.0 : Math.PI;
            robotZ -= dirZ * (0.40 * dt);

            const backupDone = (dirZ > 0 && robotZ <= nav3D.wp0_z) || (dirZ < 0 && robotZ >= nav3D.wp0_z);
            if (backupDone) {
                robotZ = nav3D.wp0_z;
                nav3D.state = "SEG1_OUT";
            }
        } else if (nav3D.state === "SEG1_OUT") {
            tinesDepth = 0.0;
            const targetX = nav3D.bypassX;
            const targetZ = nav3D.wp1.z;
            const dx = targetX - robotX;
            const dz = targetZ - robotZ;
            robotYaw = Math.atan2(dx, dz);

            const dist = Math.hypot(dx, dz);
            const step = Math.min(0.55 * dt, dist);
            if (dist > 0.02) {
                robotX += (dx / dist) * step;
                robotZ += (dz / dist) * step;
            }
            if (Math.abs(robotX - targetX) < 0.03 || (dirZ > 0 && robotZ >= targetZ) || (dirZ < 0 && robotZ <= targetZ)) {
                robotX = targetX;
                robotZ = targetZ;
                robotYaw = (dirZ > 0) ? 0.0 : Math.PI;
                nav3D.state = "SEG2_STRAIGHT";
            }
        } else if (nav3D.state === "SEG2_STRAIGHT") {
            tinesDepth = 0.0;
            robotX = nav3D.bypassX;
            robotYaw = (dirZ > 0) ? 0.0 : Math.PI;
            robotZ += dirZ * (0.60 * dt);

            const passedWp2 = (dirZ > 0 && robotZ >= nav3D.wp2_z) || (dirZ < 0 && robotZ <= nav3D.wp2_z);
            if (passedWp2) {
                robotZ = nav3D.wp2_z;
                nav3D.state = "SEG3_RETURN";
            }
        } else if (nav3D.state === "SEG3_RETURN") {
            tinesDepth = 0.0;
            const targetX = nav3D.currentRowX;
            const targetZ = nav3D.wp3.z;
            const dx = targetX - robotX;
            const dz = targetZ - robotZ;
            robotYaw = Math.atan2(dx, dz);

            const dist = Math.hypot(dx, dz);
            const step = Math.min(0.55 * dt, dist);
            if (dist > 0.02) {
                robotX += (dx / dist) * step;
                robotZ += (dz / dist) * step;
            }
            if (Math.abs(robotX - targetX) < 0.03 || (dirZ > 0 && robotZ >= targetZ) || (dirZ < 0 && robotZ <= targetZ)) {
                robotX = targetX;
                robotZ = targetZ;
                robotYaw = (dirZ > 0) ? 0.0 : Math.PI;
                nav3D.state = "REALIGN_AND_DROP";
            }
        } else if (nav3D.state === "REALIGN_AND_DROP") {
            robotX = nav3D.currentRowX;
            robotYaw = (dirZ > 0) ? 0.0 : Math.PI;
            if (nav3D.activeObs) {
                nav3D.clearedObstacleIds.add(String(nav3D.activeObs.id));
                nav3D.activeObs = null;
            }
            tinesDepth = 1.0;
            nav3D.state = "FOLLOWING_ROW";
        }

        // Mise à jour de la position 3D du groupe robot
        robotGroup.position.x = robotX;
        robotGroup.position.z = robotZ;
        robotGroup.rotation = robotYaw;
        robotGroup.position.z = robotZ;

        // VÉRIFICATION OBLIGATOIRE DE NON-INTERSECTION BOX3 À CHAQUE FRAME
        const updatedRobotBox = new THREE.Box3().setFromObject(robotGroup);
        const isHit = updatedRobotBox.intersectsBox(obsBox);

        if (isHit) {
            collisionsCount++;
            console.error(`💥 COLLISION DÉTECTÉE frame ${frame} : robotBox=[${updatedRobotBox.min.x.toFixed(2)},${updatedRobotBox.min.z.toFixed(2)} à ${updatedRobotBox.max.x.toFixed(2)},${updatedRobotBox.max.z.toFixed(2)}] vs obsBox=[${obsBox.min.x.toFixed(2)},${obsBox.min.z.toFixed(2)} à ${obsBox.max.x.toFixed(2)},${obsBox.max.z.toFixed(2)}]`);
        }

        // Distance minimale réelle entre l'empreinte au sol et l'obstacle
        let distBox;
        if (updatedRobotBox.corners) {
            let minD = Infinity;
            for (const p of updatedRobotBox.corners) {
                const dx = Math.max(0, obsBox.min.x - p.x, p.x - obsBox.max.x);
                const dz = Math.max(0, obsBox.min.z - p.z, p.z - obsBox.max.z);
                minD = Math.min(minD, Math.hypot(dx, dz));
            }
            distBox = minD;
        } else {
            const dxBox = Math.max(0, updatedRobotBox.min.x - obsBox.max.x, obsBox.min.x - updatedRobotBox.max.x);
            const dzBox = Math.max(0, updatedRobotBox.min.z - obsBox.max.z, obsBox.min.z - updatedRobotBox.max.z);
            distBox = Math.hypot(dxBox, dzBox);
        }
        if (distBox < minDistanceRecorded) {
            minDistanceRecorded = distBox;
        }

        if (frame % 25 === 0 || nav3D.state !== (traceFrames[traceFrames.length - 1]?.state)) {
            traceFrames.push({
                frame,
                t: (frame * dt).toFixed(2),
                state: nav3D.state,
                x: robotX.toFixed(3),
                z: robotZ.toFixed(3),
                yawDeg: ((robotYaw * 180) / Math.PI).toFixed(1),
                distBox: distBox.toFixed(3),
                isHit
            });
        }
    }

    // Affiche un extrait de la trace
    console.log(`Trace d'échantillonnage de la trajectoire (Frame, Temps, État, X, Z, Cap, Marge Sécurité) :`);
    console.table(traceFrames.slice(0, 20));

    console.log(`\n--- RÉSULTATS DU TEST ---`);
    console.log(`Frames simulées         : 500`);
    console.log(`Collisions détectées    : ${collisionsCount} (robotBox.intersectsBox(obsBox))`);
    console.log(`Marge minimale mesurée  : ${minDistanceRecorded.toFixed(3)} m (Marge compacte >= 0.18m)`);

    if (collisionsCount === 0 && minDistanceRecorded >= 0.18) {
        console.log(`✅ TEST RÉUSSI : robotBox.intersectsBox(obsBox) est STRICTEMENT FALSE à chaque frame !\n`);
        return true;
    } else {
        console.error(`❌ ÉCHEC DU TEST : Collision détectée ou marge insuffisante !\n`);
        return false;
    }
}

// 1. Test Aller (+Z)
const testAller = runSimulationScenario("ALLER (+Z)", 1, 0.0, 0.0, 0.0, 5.2, 0.50, 0.50);

// 2. Test Retour (-Z)
const testRetour = runSimulationScenario("RETOUR (-Z)", -1, 0.92, 10.0, 0.92, 4.8, 0.50, 0.50);

if (!testAller || !testRetour) {
    process.exit(1);
} else {
    console.log("🎉 TOUS LES TESTS DE SIMULATION 500 FRAMES (ALLER & RETOUR) ONT PASSÉ À 100% SANS AUCUNE COLLISION !");
    process.exit(0);
}
