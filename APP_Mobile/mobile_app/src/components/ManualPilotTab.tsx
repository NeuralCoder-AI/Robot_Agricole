import React, { useEffect, useMemo, useRef, useState } from 'react';
import {
  PanResponder,
  ScrollView,
  StyleSheet,
  Text,
  TouchableOpacity,
  Vibration,
  View,
} from 'react-native';
import { AgronomicMetrics, DoneEventRecord, RobotOperatingMode } from '../types/robot';
import { computeDifferentialDrive } from '../utils/protocol';

interface ManualPilotTabProps {
  armed: number;
  mode: RobotOperatingMode;
  motors: number[];
  agronomics: AgronomicMetrics;
  lastDoneEvents: DoneEventRecord[];
  onDrive: (a: number, d: number, ms?: number) => void;
  onTank: (l: number, r: number, ms?: number) => void;
  onStopAll: () => void;
  onStopMotor: (motor: number) => void;
  onMotorSpeed: (motor: number, speed: number, ms?: number) => void;
  onPulse: (motor: number, speed: number, ms: number) => void;
  onSendPing: () => void;
}

const JOYSTICK_RADIUS = 85;
const KNOB_RADIUS = 30;
const MAX_TRAVEL = JOYSTICK_RADIUS - KNOB_RADIUS / 2;

export const ManualPilotTab: React.FC<ManualPilotTabProps> = ({
  mode,
  motors,
  agronomics,
  lastDoneEvents,
  onDrive,
  onTank,
  onStopAll,
  onStopMotor,
  onMotorSpeed,
  onPulse,
  onSendPing,
}) => {
  const [speedLimit, setSpeedLimit] = useState<number>(60);
  const [weederSpeed, setWeederSpeed] = useState<number>(70);
  const [joyPos, setJoyPos] = useState<{ x: number; y: number }>({ x: 0, y: 0 });
  const [driveVector, setDriveVector] = useState<{ a: number; d: number }>({ a: 0, d: 0 });
  const [isHoldingContinuous, setIsHoldingContinuous] = useState<boolean>(false);

  const lastDriveSendRef = useRef<number>(0);
  const speedLimitRef = useRef<number>(speedLimit);
  speedLimitRef.current = speedLimit;

  const isLocked = mode === 'AUTO';
  const isLockedRef = useRef<boolean>(isLocked);
  isLockedRef.current = isLocked;

  const onDriveRef = useRef(onDrive);
  onDriveRef.current = onDrive;

  const onStopAllRef = useRef(onStopAll);
  onStopAllRef.current = onStopAll;

  // Entretien automatique du watchdog UART (UWDT = 1000 ms) :
  // Envoie PING*10 toutes les 300 ms tant qu'une commande continue est active
  useEffect(() => {
    const anyContinuousActive =
      isHoldingContinuous ||
      Math.abs(motors[5] || 0) > 0 ||
      Math.abs(motors[7] || 0) > 0;

    if (!anyContinuousActive) return;

    const timer = setInterval(() => {
      onSendPing();
    }, 300);

    return () => clearInterval(timer);
  }, [isHoldingContinuous, motors, onSendPing]);

  // Joystick tactile fluide basé sur PanResponder natif React Native
  const panResponder = useMemo(
    () =>
      PanResponder.create({
        onStartShouldSetPanResponder: () => !isLockedRef.current,
        onMoveShouldSetPanResponder: () => !isLockedRef.current,
        onPanResponderGrant: () => {
          if (isLockedRef.current) return;
          setIsHoldingContinuous(true);
          try {
            Vibration.vibrate(15);
          } catch {
            // ignore if vibration unsupported
          }
        },
        onPanResponderMove: (_evt, gestureState) => {
          if (isLockedRef.current) return;
          const { dx, dy } = gestureState;
          const dist = Math.hypot(dx, dy);
          const clampedDist = Math.min(dist, MAX_TRAVEL);
          const angle = Math.atan2(dy, dx);

          const nx = clampedDist * Math.cos(angle);
          const ny = clampedDist * Math.sin(angle);
          setJoyPos({ x: nx, y: ny });

          const currentLimit = speedLimitRef.current;
          const a = Math.round((-ny / MAX_TRAVEL) * currentLimit);
          const d = Math.round((nx / MAX_TRAVEL) * currentLimit);
          setDriveVector({ a, d });
          setIsHoldingContinuous(true);

          const now = Date.now();
          if (now - lastDriveSendRef.current >= 85) {
            lastDriveSendRef.current = now;
            onDriveRef.current(a, d);
          }
        },
        onPanResponderRelease: () => {
          setJoyPos({ x: 0, y: 0 });
          setDriveVector({ a: 0, d: 0 });
          setIsHoldingContinuous(false);
          if (!isLockedRef.current) {
            onStopAllRef.current();
          }
        },
        onPanResponderTerminate: () => {
          setJoyPos({ x: 0, y: 0 });
          setDriveVector({ a: 0, d: 0 });
          setIsHoldingContinuous(false);
          if (!isLockedRef.current) {
            onStopAllRef.current();
          }
        },
      }),
    []
  );

  const handleReleaseButton = () => {
    setJoyPos({ x: 0, y: 0 });
    setDriveVector({ a: 0, d: 0 });
    setIsHoldingContinuous(false);
    if (!isLocked) {
      onStopAll();
    }
  };

  const diffPreview = computeDifferentialDrive(driveVector.a, driveVector.d);
  const lastM7Done = lastDoneEvents.find((e) => e.motor === 7);

  return (
    <ScrollView
      style={styles.container}
      contentContainerStyle={styles.content}
      scrollEnabled={!isHoldingContinuous}
    >
      {isLocked && (
        <View style={styles.lockedBanner}>
          <Text style={styles.lockedBannerText}>
            🔒 JOYSTICK VERROUILLÉ : Mode AUTOMATIQUE actif. Passez en MANUEL pour reprendre la main.
          </Text>
        </View>
      )}

      {/* SECTION 1 : LOCOMOTION DIFFÉRENTIELLE (M1 à M4) */}
      <View style={styles.card}>
        <View style={styles.cardHeader}>
          <Text style={styles.cardTitle}>1. LOCOMOTION 4×4 (M1 À M4)</Text>
          <Text style={styles.formulaBadge}>
            G(M1,M3)={diffPreview.left}% | D(M2,M4)={diffPreview.right}%
          </Text>
        </View>

        {/* Curseur de limitation de vitesse globale */}
        <View style={styles.speedLimitBox}>
          <Text style={styles.speedLimitLabel}>
            LIMITEUR DE VITESSE GLOBALE : <Text style={styles.cyanVal}>{speedLimit}%</Text>
          </Text>
          <View style={styles.presetRow}>
            {[25, 40, 60, 80, 100].map((val) => (
              <TouchableOpacity
                key={val}
                style={[
                  styles.presetBtn,
                  speedLimit === val && styles.presetBtnActive,
                ]}
                onPress={() => setSpeedLimit(val)}
              >
                <Text
                  style={[
                    styles.presetBtnText,
                    speedLimit === val && styles.presetBtnTextActive,
                  ]}
                >
                  {val}%
                </Text>
              </TouchableOpacity>
            ))}
          </View>
        </View>

        {/* Zone Tactile : Joystick Virtuel (PanResponder) + D-Pad Proportionnel */}
        <View style={styles.controlsGrid}>
          {/* Joystick Virtuel Tactile PanResponder */}
          <View style={styles.joystickColumn}>
            <View
              style={[styles.joystickPad, isLocked && { opacity: 0.4 }]}
              {...panResponder.panHandlers}
            >
              <View style={styles.crosshairH} />
              <View style={styles.crosshairV} />
              <View
                style={[
                  styles.joystickKnob,
                  {
                    transform: [
                      { translateX: joyPos.x },
                      { translateY: joyPos.y },
                    ],
                  },
                ]}
              >
                <Text style={styles.knobLabel}>JOY</Text>
              </View>
            </View>
            <Text style={styles.joyReadout}>
              DRIVE:{driveVector.a},{driveVector.d} (STOP auto au relâchement)
            </Text>
          </View>

          {/* D-Pad Proportionnel & Arrêt */}
          <View style={styles.dpadColumn}>
            <TouchableOpacity
              style={styles.dpadBtn}
              disabled={isLocked}
              onPressIn={() => {
                setIsHoldingContinuous(true);
                setDriveVector({ a: speedLimit, d: 0 });
                onDrive(speedLimit, 0);
              }}
              onPressOut={handleReleaseButton}
            >
              <Text style={styles.dpadBtnText}>▲ AVANCER</Text>
            </TouchableOpacity>

            <View style={styles.dpadMiddleRow}>
              <TouchableOpacity
                style={styles.dpadBtnSmall}
                disabled={isLocked}
                onPressIn={() => {
                  const a = Math.round(speedLimit * 0.6);
                  const d = -Math.round(speedLimit * 0.5);
                  setIsHoldingContinuous(true);
                  setDriveVector({ a, d });
                  onDrive(a, d);
                }}
                onPressOut={handleReleaseButton}
              >
                <Text style={styles.dpadBtnText}>◀ G</Text>
              </TouchableOpacity>

              <TouchableOpacity
                style={styles.dpadStopBtn}
                onPress={onStopAll}
              >
                <Text style={styles.dpadStopBtnText}>STOP</Text>
              </TouchableOpacity>

              <TouchableOpacity
                style={styles.dpadBtnSmall}
                disabled={isLocked}
                onPressIn={() => {
                  const a = Math.round(speedLimit * 0.6);
                  const d = Math.round(speedLimit * 0.5);
                  setIsHoldingContinuous(true);
                  setDriveVector({ a, d });
                  onDrive(a, d);
                }}
                onPressOut={handleReleaseButton}
              >
                <Text style={styles.dpadBtnText}>D ▶</Text>
              </TouchableOpacity>
            </View>

            <TouchableOpacity
              style={styles.dpadBtn}
              disabled={isLocked}
              onPressIn={() => {
                setIsHoldingContinuous(true);
                setDriveVector({ a: -speedLimit, d: 0 });
                onDrive(-speedLimit, 0);
              }}
              onPressOut={handleReleaseButton}
            >
              <Text style={styles.dpadBtnText}>▼ RECULER</Text>
            </TouchableOpacity>
          </View>
        </View>

        {/* Boutons de Rotation sur Place (Pivot Tank) */}
        <View style={styles.tankRow}>
          <TouchableOpacity
            style={styles.tankBtn}
            disabled={isLocked}
            onPressIn={() => {
              setIsHoldingContinuous(true);
              onTank(-50, 50);
            }}
            onPressOut={handleReleaseButton}
          >
            <Text style={styles.tankBtnTitle}>↺ PIVOT GAUCHE</Text>
            <Text style={styles.tankBtnSub}>TANK:-50,50</Text>
          </TouchableOpacity>

          <TouchableOpacity
            style={styles.tankBtn}
            disabled={isLocked}
            onPressIn={() => {
              setIsHoldingContinuous(true);
              onTank(50, -50);
            }}
            onPressOut={handleReleaseButton}
          >
            <Text style={styles.tankBtnTitle}>↻ PIVOT DROITE</Text>
            <Text style={styles.tankBtnSub}>TANK:50,-50</Text>
          </TouchableOpacity>
        </View>
      </View>

      {/* SECTION 2 : PANNEAU DES 4 OUTILS AGRICOLES (M5 À M8) */}
      <View style={styles.card}>
        <View style={styles.cardHeader}>
          <Text style={styles.cardTitle}>2. OUTILS AGRICOLES (M5 À M8)</Text>
          <Text style={styles.watchdogTag}>UWDT 1000ms (PING*10 300ms)</Text>
        </View>

        {/* OUTIL M5 : DENTS DE TERRAGE (GPIO 8/9) */}
        <View style={styles.toolBlock}>
          <View style={styles.toolHeaderRow}>
            <Text style={styles.toolTitle}>⛏️ M5 — Dents de Terrage (GPIO 8/9)</Text>
            <Text style={styles.toolVal}>
              Profondeur: {agronomics.tines_depth}% | PWM: {motors[4] || 0}%
            </Text>
          </View>
          <View style={styles.gaugeTrack}>
            <View
              style={[
                styles.gaugeFill,
                { width: `${agronomics.tines_depth}%`, backgroundColor: '#ff9100' },
              ]}
            />
          </View>
          <View style={styles.toolBtnRow}>
            <TouchableOpacity
              style={[styles.toolBtn, styles.btnPrimary]}
              onPress={() => onPulse(5, 50, 4000)}
            >
              <Text style={styles.toolBtnText}>▼ Descendre (4s)</Text>
              <Text style={styles.toolCmdSub}>PULSE:5:50:4000</Text>
            </TouchableOpacity>
            <TouchableOpacity
              style={[styles.toolBtn, styles.btnSecondary]}
              onPress={() => onPulse(5, -50, 4000)}
            >
              <Text style={styles.toolBtnText}>▲ Relever (4s)</Text>
              <Text style={styles.toolCmdSub}>PULSE:5:-50:4000</Text>
            </TouchableOpacity>
            <TouchableOpacity
              style={[styles.toolBtn, styles.btnStopTool]}
              onPress={() => onStopMotor(5)}
            >
              <Text style={styles.toolBtnText}>⏹ STOP:5</Text>
            </TouchableOpacity>
          </View>
        </View>

        {/* OUTIL M6 : FRAISE DE DÉSHERBAGE (GPIO 10/11) */}
        <View style={styles.toolBlock}>
          <View style={styles.toolHeaderRow}>
            <Text style={styles.toolTitle}>⚙️ M6 — Fraise de Désherbage (GPIO 10/11)</Text>
            <Text style={styles.toolVal}>Consigne: {weederSpeed}% | Actuel: {motors[5] || 0}%</Text>
          </View>
          <View style={styles.presetRow}>
            {[0, 30, 50, 70, 90, 100].map((spd) => (
              <TouchableOpacity
                key={spd}
                style={[
                  styles.presetBtn,
                  weederSpeed === spd && styles.presetBtnActive,
                ]}
                onPress={() => {
                  setWeederSpeed(spd);
                  if (spd === 0) onStopMotor(6);
                  else onMotorSpeed(6, spd);
                }}
              >
                <Text
                  style={[
                    styles.presetBtnText,
                    weederSpeed === spd && styles.presetBtnTextActive,
                  ]}
                >
                  {spd}%
                </Text>
              </TouchableOpacity>
            ))}
          </View>
          <View style={styles.toolBtnRow}>
            <TouchableOpacity
              style={[styles.toolBtn, styles.btnGreen]}
              onPress={() => onMotorSpeed(6, weederSpeed || 70)}
            >
              <Text style={styles.toolBtnText}>▶ Marche ({weederSpeed || 70}%)</Text>
              <Text style={styles.toolCmdSub}>M6:{weederSpeed || 70}</Text>
            </TouchableOpacity>
            <TouchableOpacity
              style={[styles.toolBtn, styles.btnWarn]}
              onPress={() => onMotorSpeed(6, -50)}
            >
              <Text style={styles.toolBtnText}>↺ Débourrage</Text>
              <Text style={styles.toolCmdSub}>M6:-50</Text>
            </TouchableOpacity>
            <TouchableOpacity
              style={[styles.toolBtn, styles.btnStopTool]}
              onPress={() => onStopMotor(6)}
            >
              <Text style={styles.toolBtnText}>⏹ STOP:6</Text>
            </TouchableOpacity>
          </View>
        </View>

        {/* OUTIL M7 : PANNEAUX SOLAIRES (GPIO 12/13) */}
        <View style={styles.toolBlock}>
          <View style={styles.toolHeaderRow}>
            <Text style={styles.toolTitle}>☀️ M7 — Panneaux Solaires (GPIO 12/13)</Text>
            <Text style={styles.toolVal}>
              {lastM7Done ? '✅ #D 7 Reçu' : `Ouverture: ${agronomics.solar_deploy}%`}
            </Text>
          </View>
          <View style={styles.toolBtnRow}>
            <TouchableOpacity
              style={[styles.toolBtn, styles.btnPrimary]}
              onPress={() => onPulse(7, 60, 30000)}
            >
              <Text style={styles.toolBtnText}>☀️ Déployer (30s)</Text>
              <Text style={styles.toolCmdSub}>PULSE:7:60:30000</Text>
            </TouchableOpacity>
            <TouchableOpacity
              style={[styles.toolBtn, styles.btnSecondary]}
              onPress={() => onPulse(7, -60, 30000)}
            >
              <Text style={styles.toolBtnText}>📁 Ranger (30s)</Text>
              <Text style={styles.toolCmdSub}>PULSE:7:-60:30000</Text>
            </TouchableOpacity>
            <TouchableOpacity
              style={[styles.toolBtn, styles.btnStopTool]}
              onPress={() => onStopMotor(7)}
            >
              <Text style={styles.toolBtnText}>⏹ STOP:7</Text>
            </TouchableOpacity>
          </View>
        </View>

        {/* OUTIL M8 : SEMOIR DE PRÉCISION (GPIO 14/21) */}
        <View style={styles.toolBlock}>
          <View style={styles.toolHeaderRow}>
            <Text style={styles.toolTitle}>🌱 M8 — Semoir (GPIO 14/21)</Text>
            <Text style={styles.toolVal}>
              Graines: {agronomics.seeds_sown} | PWM: {motors[7] || 0}%
            </Text>
          </View>
          <View style={styles.toolBtnRow}>
            <TouchableOpacity
              style={[styles.toolBtn, styles.btnGreen]}
              onPress={() => onMotorSpeed(8, 60)}
            >
              <Text style={styles.toolBtnText}>🌱 Semis Continu</Text>
              <Text style={styles.toolCmdSub}>M8:60</Text>
            </TouchableOpacity>
            <TouchableOpacity
              style={[styles.toolBtn, styles.btnPrimary]}
              onPress={() => onPulse(8, 60, 2000)}
            >
              <Text style={styles.toolBtnText}>⚡ Rafale 2s</Text>
              <Text style={styles.toolCmdSub}>PULSE:8:60:2000</Text>
            </TouchableOpacity>
            <TouchableOpacity
              style={[styles.toolBtn, styles.btnWarn]}
              onPress={() => onPulse(8, -40, 5000)}
            >
              <Text style={styles.toolBtnText}>🔄 Vidange 5s</Text>
              <Text style={styles.toolCmdSub}>PULSE:8:-40:5000</Text>
            </TouchableOpacity>
            <TouchableOpacity
              style={[styles.toolBtn, styles.btnStopTool]}
              onPress={() => onStopMotor(8)}
            >
              <Text style={styles.toolBtnText}>⏹ STOP:8</Text>
            </TouchableOpacity>
          </View>
        </View>
      </View>
    </ScrollView>
  );
};

const styles = StyleSheet.create({
  container: { flex: 1, backgroundColor: '#070b12' },
  content: { padding: 12, gap: 12, paddingBottom: 32 },
  lockedBanner: {
    backgroundColor: 'rgba(255, 145, 0, 0.2)',
    borderWidth: 1,
    borderColor: '#ff9100',
    borderRadius: 8,
    padding: 10,
  },
  lockedBannerText: {
    color: '#ffd180',
    fontWeight: '800',
    fontSize: 12,
    textAlign: 'center',
  },
  card: {
    backgroundColor: '#101829',
    borderRadius: 12,
    borderWidth: 1,
    borderColor: '#1f3052',
    padding: 12,
    gap: 12,
  },
  cardHeader: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    borderBottomWidth: 1,
    borderBottomColor: '#1e2e4d',
    paddingBottom: 8,
  },
  cardTitle: {
    color: '#00e5ff',
    fontSize: 13,
    fontWeight: '900',
    letterSpacing: 0.6,
  },
  formulaBadge: {
    color: '#00e676',
    fontSize: 11,
    fontWeight: '800',
  },
  watchdogTag: {
    color: '#ffd600',
    fontSize: 10,
    fontWeight: '800',
  },
  speedLimitBox: {
    gap: 6,
  },
  speedLimitLabel: {
    color: '#b0bec5',
    fontSize: 12,
    fontWeight: '800',
  },
  cyanVal: {
    color: '#00e5ff',
    fontWeight: '900',
  },
  presetRow: {
    flexDirection: 'row',
    gap: 6,
  },
  presetBtn: {
    flex: 1,
    backgroundColor: '#18243c',
    paddingVertical: 7,
    borderRadius: 6,
    borderWidth: 1,
    borderColor: '#283b60',
    alignItems: 'center',
  },
  presetBtnActive: {
    backgroundColor: '#00e5ff',
    borderColor: '#84ffff',
  },
  presetBtnText: {
    color: '#cfd8dc',
    fontSize: 12,
    fontWeight: '800',
  },
  presetBtnTextActive: {
    color: '#031926',
  },
  controlsGrid: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    gap: 12,
  },
  joystickColumn: {
    alignItems: 'center',
    gap: 6,
  },
  joystickPad: {
    width: JOYSTICK_RADIUS * 2,
    height: JOYSTICK_RADIUS * 2,
    borderRadius: JOYSTICK_RADIUS,
    backgroundColor: '#090f1c',
    borderWidth: 2,
    borderColor: '#00e5ff',
    alignItems: 'center',
    justifyContent: 'center',
    position: 'relative',
  },
  crosshairH: {
    position: 'absolute',
    width: '85%',
    height: 1,
    backgroundColor: '#1c2e4a',
  },
  crosshairV: {
    position: 'absolute',
    height: '85%',
    width: 1,
    backgroundColor: '#1c2e4a',
  },
  joystickKnob: {
    width: KNOB_RADIUS * 2,
    height: KNOB_RADIUS * 2,
    borderRadius: KNOB_RADIUS,
    backgroundColor: '#00e5ff',
    borderWidth: 3,
    borderColor: '#ffffff',
    alignItems: 'center',
    justifyContent: 'center',
  },
  knobLabel: {
    color: '#002233',
    fontSize: 11,
    fontWeight: '900',
  },
  joyReadout: {
    color: '#90a4ae',
    fontSize: 10,
    fontWeight: '700',
  },
  dpadColumn: {
    flex: 1,
    gap: 8,
  },
  dpadBtn: {
    backgroundColor: '#1a2742',
    borderWidth: 1,
    borderColor: '#2d4470',
    borderRadius: 8,
    paddingVertical: 12,
    alignItems: 'center',
  },
  dpadMiddleRow: {
    flexDirection: 'row',
    gap: 6,
  },
  dpadBtnSmall: {
    flex: 1,
    backgroundColor: '#1a2742',
    borderWidth: 1,
    borderColor: '#2d4470',
    borderRadius: 8,
    paddingVertical: 12,
    alignItems: 'center',
  },
  dpadStopBtn: {
    flex: 1.1,
    backgroundColor: '#ff5252',
    borderRadius: 8,
    paddingVertical: 12,
    alignItems: 'center',
  },
  dpadBtnText: {
    color: '#ffffff',
    fontSize: 12,
    fontWeight: '900',
  },
  dpadStopBtnText: {
    color: '#ffffff',
    fontSize: 12,
    fontWeight: '900',
  },
  tankRow: {
    flexDirection: 'row',
    gap: 10,
  },
  tankBtn: {
    flex: 1,
    backgroundColor: '#17233b',
    borderWidth: 1,
    borderColor: '#00e5ff',
    borderRadius: 8,
    paddingVertical: 10,
    alignItems: 'center',
  },
  tankBtnTitle: {
    color: '#00e5ff',
    fontSize: 12,
    fontWeight: '900',
  },
  tankBtnSub: {
    color: '#90a4ae',
    fontSize: 10,
    fontWeight: '700',
  },
  toolBlock: {
    backgroundColor: '#0a101d',
    borderRadius: 10,
    padding: 10,
    borderWidth: 1,
    borderColor: '#1a2947',
    gap: 8,
  },
  toolHeaderRow: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
  },
  toolTitle: {
    color: '#f0f4f8',
    fontSize: 13,
    fontWeight: '800',
  },
  toolVal: {
    color: '#00e676',
    fontSize: 11,
    fontWeight: '800',
  },
  gaugeTrack: {
    height: 8,
    backgroundColor: '#152036',
    borderRadius: 4,
    overflow: 'hidden',
  },
  gaugeFill: {
    height: '100%',
  },
  toolBtnRow: {
    flexDirection: 'row',
    flexWrap: 'wrap',
    gap: 6,
  },
  toolBtn: {
    flexGrow: 1,
    paddingVertical: 8,
    paddingHorizontal: 8,
    borderRadius: 8,
    alignItems: 'center',
    borderWidth: 1,
  },
  btnPrimary: {
    backgroundColor: '#162847',
    borderColor: '#00e5ff',
  },
  btnSecondary: {
    backgroundColor: '#1c263b',
    borderColor: '#546e7a',
  },
  btnGreen: {
    backgroundColor: '#0c3320',
    borderColor: '#00e676',
  },
  btnWarn: {
    backgroundColor: '#3b250c',
    borderColor: '#ff9100',
  },
  btnStopTool: {
    backgroundColor: '#3b1219',
    borderColor: '#ff5252',
  },
  toolBtnText: {
    color: '#ffffff',
    fontSize: 11,
    fontWeight: '900',
  },
  toolCmdSub: {
    color: '#90a4ae',
    fontSize: 9,
    fontWeight: '700',
  },
});
