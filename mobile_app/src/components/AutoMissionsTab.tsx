import React from 'react';
import {
  ScrollView,
  StyleSheet,
  Text,
  TouchableOpacity,
  View,
} from 'react-native';
import { AgronomicMetrics, DoneEventRecord, MissionState } from '../types/robot';

interface AutoMissionsTabProps {
  missionState: MissionState;
  agronomics: AgronomicMetrics;
  doneEvents: DoneEventRecord[];
  onStartMission: (name: string) => void;
  onPauseMission: () => void;
  onResumeMission: () => void;
  onAbortMission: () => void;
}

export const AutoMissionsTab: React.FC<AutoMissionsTabProps> = ({
  missionState,
  agronomics,
  doneEvents,
  onStartMission,
  onPauseMission,
  onResumeMission,
  onAbortMission,
}) => {
  const isRunning = Boolean(missionState.name);

  return (
    <ScrollView style={styles.container} contentContainerStyle={styles.content}>
      {/* Bannière de verrouillage du Joystick Manuel */}
      <View style={styles.safetyLockBanner}>
        <Text style={styles.safetyLockIcon}>🛡️</Text>
        <View style={{ flex: 1 }}>
          <Text style={styles.safetyLockTitle}>
            VERROUILLAGE ANTI-CONFLIT DU JOYSTICK MANUEL ACTIF
          </Text>
          <Text style={styles.safetyLockDesc}>
            Les commandes manuelles de locomotion sont bloquées pendant l'orchestration autonome.
            Basculer en mode MANUEL annule immédiatement la mission et envoie STOP.
          </Text>
        </View>
      </View>

      {/* PANNEAU DE PROGRESSION DE LA MISSION EN COURS */}
      <View style={styles.progressCard}>
        <View style={styles.progressHeader}>
          <Text style={styles.progressTitle}>
            {isRunning
              ? `🚀 MISSION ACTIVE : ${missionState.name?.toUpperCase()}`
              : '⏸ AUCUNE MISSION EN COURS'}
          </Text>
          <Text style={styles.progressPct}>{missionState.progress}%</Text>
        </View>

        <View style={styles.progressBarBg}>
          <View
            style={[
              styles.progressBarFill,
              {
                width: `${missionState.progress}%`,
                backgroundColor: missionState.paused ? '#ffd600' : '#00e676',
              },
            ]}
          />
        </View>

        <Text style={styles.stepLabel}>
          Étape active : <Text style={styles.stepLabelHighlight}>{missionState.step_label}</Text>
        </Text>

        {/* Boutons Pause / Reprendre / Arrêt de mission */}
        <View style={styles.controlButtonsRow}>
          {!missionState.paused ? (
            <TouchableOpacity
              style={[styles.ctrlBtn, styles.pauseBtn, !isRunning && styles.disabledBtn]}
              disabled={!isRunning}
              onPress={onPauseMission}
            >
              <Text style={styles.ctrlBtnText}>⏸ METTRE EN PAUSE</Text>
            </TouchableOpacity>
          ) : (
            <TouchableOpacity
              style={[styles.ctrlBtn, styles.resumeBtn]}
              onPress={onResumeMission}
            >
              <Text style={styles.ctrlBtnText}>▶ REPRENDRE LA MISSION</Text>
            </TouchableOpacity>
          )}

          <TouchableOpacity
            style={[styles.ctrlBtn, styles.abortBtn, !isRunning && styles.disabledBtn]}
            disabled={!isRunning}
            onPress={onAbortMission}
          >
            <Text style={styles.ctrlBtnText}>⏹ ARRÊTER MISSION (STOP)</Text>
          </TouchableOpacity>
        </View>
      </View>

      {/* LISTE DES 4 MISSIONS OFFICIELLES */}
      <View style={styles.missionsCard}>
        <Text style={styles.sectionTitle}>CATALOGUE DES MISSIONS AUTONOMES (IA & RTK)</Text>

        {/* MISSION 1 : DÉSHERBAGE INTELLIGENT COMPLET */}
        <View style={styles.missionItem}>
          <View style={styles.missionHeader}>
            <Text style={styles.missionName}>
              🌿 Mission 1 — Désherbage Intelligent Complet (M5 + M6 + DRIVE)
            </Text>
            <Text style={styles.missionTag}>SÉQUENCE OFFICIELLE</Text>
          </View>
          <Text style={styles.missionSeq}>
            ARM → PULSE:5:50:4000 → Attente #D 5 → M6:70 → Avancée DRIVE:20,0:15000 (maintien PING 300ms)
            → Arrêt impératif STOP:6 avant relevage → PULSE:5:-50:4000 → Attente #D 5
          </Text>
          <View style={styles.missionActionRow}>
            <TouchableOpacity
              style={styles.launchBtn}
              onPress={() => onStartMission('weeding')}
            >
              <Text style={styles.launchBtnText}>▶ LANCER SÉQUENCE OFFICIELLE</Text>
            </TouchableOpacity>
            <TouchableOpacity
              style={[styles.launchBtn, styles.launchBtnAlt]}
              onPress={() => onStartMission('smart_weeding')}
            >
              <Text style={styles.launchBtnText}>⛰️ AVEC COMPENSATION PENTE</Text>
            </TouchableOpacity>
          </View>
        </View>

        {/* MISSION 2 : SEMIS AUTONOME SUR SILLON */}
        <View style={styles.missionItem}>
          <View style={styles.missionHeader}>
            <Text style={styles.missionName}>
              🌱 Mission 2 — Semis Autonome sur Sillon (M8 + DRIVE)
            </Text>
            <Text style={styles.missionTag}>PRÉCISION 3 RANGS</Text>
          </View>
          <Text style={styles.missionSeq}>
            ARM → M8:60 → DRIVE:30,0:20000 → Attente #D 1 à #D 4 → STOP:8
          </Text>
          <TouchableOpacity
            style={styles.launchBtn}
            onPress={() => onStartMission('seeding')}
          >
            <Text style={styles.launchBtnText}>▶ LANCER SEMIS AUTONOME</Text>
          </TouchableOpacity>
        </View>

        {/* MISSION 3 : DÉPLOIEMENT / RANGEMENT PANNEAUX SOLAIRES */}
        <View style={styles.missionItem}>
          <View style={styles.missionHeader}>
            <Text style={styles.missionName}>
              ☀️ Mission 3 — Panneaux Solaires Photovoltaïques (M7)
            </Text>
            <Text style={styles.missionTag}>TMAX:7:35 SÉCURISÉ</Text>
          </View>
          <Text style={styles.missionSeq}>
            ARM → TMAX:7:35 → PULSE:7:60:30000 (Déployer) ou PULSE:7:-60:30000 (Ranger) → Attente #D 7
          </Text>
          <View style={styles.missionActionRow}>
            <TouchableOpacity
              style={styles.launchBtn}
              onPress={() => onStartMission('solar_deploy')}
            >
              <Text style={styles.launchBtnText}>☀️ DÉPLOYER (30 s)</Text>
            </TouchableOpacity>
            <TouchableOpacity
              style={[styles.launchBtn, styles.launchBtnSecondary]}
              onPress={() => onStartMission('solar_retract')}
            >
              <Text style={styles.launchBtnText}>📁 RANGER (30 s)</Text>
            </TouchableOpacity>
          </View>
        </View>

        {/* MISSION 4 : PARCOURS AUTONOME MULTI-RANGS (VISION / RTK PAR PETITS PAS) */}
        <View style={styles.missionItem}>
          <View style={styles.missionHeader}>
            <Text style={styles.missionName}>
              🛰️ Mission 4 — Parcours Multi-Rangs (Vision / RTK par petits pas)
            </Text>
            <Text style={styles.missionTag}>300–1000 ms</Text>
          </View>
          <Text style={styles.missionSeq}>
            Corrections dynamiques sur pentes par petits pas DRIVE:a,d:ms (300 à 1000 ms),
            évitement d'obstacles LiDAR ({agronomics.distance}m) et désherbage/semis simultané.
          </Text>
          <View style={styles.missionActionRow}>
            <TouchableOpacity
              style={styles.launchBtn}
              onPress={() => onStartMission('multi_row')}
            >
              <Text style={styles.launchBtnText}>🛰️ LANCER MULTI-RANGS COMPLET</Text>
            </TouchableOpacity>
            <TouchableOpacity
              style={[styles.launchBtn, styles.launchBtnAlt]}
              onPress={() => onStartMission('step_nav')}
            >
              <Text style={styles.launchBtnText}>🧭 PAS RTK SEULS</Text>
            </TouchableOpacity>
          </View>
        </View>
      </View>

      {/* JOURNAL DES ÉVÉNEMENTS ASYNCHRONES #D n REÇUS */}
      <View style={styles.doneLogCard}>
        <View style={styles.progressHeader}>
          <Text style={styles.sectionTitle}>JOURNAL DES FINS D'ACTION (#D n)</Text>
          <Text style={styles.doneCountBadge}>{doneEvents.length} événements</Text>
        </View>
        {doneEvents.length === 0 ? (
          <Text style={styles.emptyLogText}>
            Aucun événement #D n reçu pour le moment. Les fins d'actions temporisées apparaîtront ici en direct.
          </Text>
        ) : (
          doneEvents.slice(0, 10).map((ev, idx) => (
            <View key={`${ev.ts}-${idx}`} style={styles.doneRow}>
              <Text style={styles.doneBadge}>{ev.raw}</Text>
              <Text style={styles.doneDesc}>
                Moteur M{ev.motor} — Action temporisée terminée
              </Text>
              <Text style={styles.doneTime}>
                {new Date(ev.ts).toLocaleTimeString()}
              </Text>
            </View>
          ))
        )}
      </View>
    </ScrollView>
  );
};

const styles = StyleSheet.create({
  container: { flex: 1, backgroundColor: '#070b12' },
  content: { padding: 12, gap: 12, paddingBottom: 32 },
  safetyLockBanner: {
    flexDirection: 'row',
    alignItems: 'center',
    backgroundColor: '#132238',
    borderWidth: 1,
    borderColor: '#00e5ff',
    borderRadius: 10,
    padding: 10,
    gap: 10,
  },
  safetyLockIcon: { fontSize: 22 },
  safetyLockTitle: {
    color: '#00e5ff',
    fontSize: 12,
    fontWeight: '900',
  },
  safetyLockDesc: {
    color: '#b0bec5',
    fontSize: 11,
    marginTop: 2,
  },
  progressCard: {
    backgroundColor: '#101829',
    borderRadius: 12,
    borderWidth: 1,
    borderColor: '#233659',
    padding: 12,
    gap: 10,
  },
  progressHeader: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
  },
  progressTitle: {
    color: '#ffffff',
    fontSize: 13,
    fontWeight: '900',
  },
  progressPct: {
    color: '#00e676',
    fontSize: 16,
    fontWeight: '900',
  },
  progressBarBg: {
    height: 12,
    backgroundColor: '#080d17',
    borderRadius: 6,
    overflow: 'hidden',
    borderWidth: 1,
    borderColor: '#1f3052',
  },
  progressBarFill: {
    height: '100%',
  },
  stepLabel: {
    color: '#90a4ae',
    fontSize: 12,
    fontWeight: '700',
  },
  stepLabelHighlight: {
    color: '#ffd600',
    fontWeight: '900',
  },
  controlButtonsRow: {
    flexDirection: 'row',
    gap: 8,
  },
  ctrlBtn: {
    flex: 1,
    paddingVertical: 11,
    borderRadius: 8,
    alignItems: 'center',
    borderWidth: 1,
  },
  pauseBtn: {
    backgroundColor: '#332607',
    borderColor: '#ffd600',
  },
  resumeBtn: {
    backgroundColor: '#09361f',
    borderColor: '#00e676',
  },
  abortBtn: {
    backgroundColor: '#3b111b',
    borderColor: '#ff1744',
  },
  disabledBtn: {
    opacity: 0.4,
  },
  ctrlBtnText: {
    color: '#ffffff',
    fontSize: 11,
    fontWeight: '900',
  },
  missionsCard: {
    backgroundColor: '#101829',
    borderRadius: 12,
    borderWidth: 1,
    borderColor: '#1f3052',
    padding: 12,
    gap: 12,
  },
  sectionTitle: {
    color: '#00e5ff',
    fontSize: 13,
    fontWeight: '900',
    letterSpacing: 0.6,
  },
  missionItem: {
    backgroundColor: '#090f1c',
    borderRadius: 10,
    borderWidth: 1,
    borderColor: '#1d2d4e',
    padding: 10,
    gap: 8,
  },
  missionHeader: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    gap: 6,
  },
  missionName: {
    flex: 1,
    color: '#f0f4f8',
    fontSize: 13,
    fontWeight: '900',
  },
  missionTag: {
    backgroundColor: '#172845',
    color: '#00e5ff',
    fontSize: 9,
    fontWeight: '900',
    paddingHorizontal: 6,
    paddingVertical: 3,
    borderRadius: 4,
  },
  missionSeq: {
    color: '#90a4ae',
    fontSize: 11,
    lineHeight: 16,
    fontFamily: 'monospace',
  },
  missionActionRow: {
    flexDirection: 'row',
    gap: 8,
  },
  launchBtn: {
    flex: 1,
    backgroundColor: '#00c853',
    paddingVertical: 10,
    borderRadius: 8,
    alignItems: 'center',
  },
  launchBtnAlt: {
    backgroundColor: '#00b0ff',
  },
  launchBtnSecondary: {
    backgroundColor: '#37474f',
  },
  launchBtnText: {
    color: '#041b0c',
    fontSize: 11,
    fontWeight: '900',
  },
  doneLogCard: {
    backgroundColor: '#101829',
    borderRadius: 12,
    borderWidth: 1,
    borderColor: '#1f3052',
    padding: 12,
    gap: 8,
  },
  doneCountBadge: {
    color: '#ffd600',
    fontSize: 11,
    fontWeight: '800',
  },
  emptyLogText: {
    color: '#607d8b',
    fontSize: 12,
    fontStyle: 'italic',
  },
  doneRow: {
    flexDirection: 'row',
    alignItems: 'center',
    backgroundColor: '#090f1b',
    paddingVertical: 6,
    paddingHorizontal: 10,
    borderRadius: 6,
    borderLeftWidth: 3,
    borderLeftColor: '#ffd600',
    gap: 8,
  },
  doneBadge: {
    color: '#ffd600',
    fontWeight: '900',
    fontFamily: 'monospace',
    fontSize: 12,
  },
  doneDesc: {
    flex: 1,
    color: '#cfd8dc',
    fontSize: 11,
  },
  doneTime: {
    color: '#78909c',
    fontSize: 10,
  },
});
