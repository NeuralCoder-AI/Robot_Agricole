/**
 * mobile_app/src/components/WeedingTab.tsx
 * Écran métier Désherbage Arachide (Inter-rang 46 cm) :
 * Dimensions du champ, descente/relevage dents M5, commande fraise M6 et lancement mission.
 */

import React from 'react';
import { StyleSheet, View, Text, TouchableOpacity, TextInput } from 'react-native';
import { ROBOT_CONFIG } from '../config/constants';
import { AgronomicMetrics, MissionState } from '../types/robot';

interface WeedingTabProps {
  fieldLength: string;
  setFieldLength: (val: string) => void;
  fieldWidth: string;
  setFieldWidth: (val: string) => void;
  calculatedCorridors: number;
  agronomics: AgronomicMetrics;
  missionState: MissionState;
  onSendRaw: (cmd: string) => void;
  onStartWeeding: () => void;
  onStopMission: () => void;
}

export const WeedingTab: React.FC<WeedingTabProps> = ({
  fieldLength,
  setFieldLength,
  fieldWidth,
  setFieldWidth,
  calculatedCorridors,
  agronomics,
  missionState,
  onSendRaw,
  onStartWeeding,
  onStopMission,
}) => {
  const isMissionActive = Boolean(missionState.name && missionState.progress < 100);

  return (
    <View style={styles.container}>
      {/* Configuration du Champ */}
      <View style={styles.card}>
        <Text style={styles.sectionTitle}>🌿 PARCELLE ARACHIDE (INTER-RANG 46 CM)</Text>
        <View style={styles.row}>
          <View style={styles.inputGroup}>
            <Text style={styles.inputLabel}>Longueur (m)</Text>
            <TextInput
              style={styles.input}
              value={fieldLength}
              onChangeText={setFieldLength}
              keyboardType="numeric"
              placeholder="10.0"
              placeholderTextColor={ROBOT_CONFIG.COLORS.textMuted}
            />
          </View>
          <View style={styles.inputGroup}>
            <Text style={styles.inputLabel}>Largeur (m)</Text>
            <TextInput
              style={styles.input}
              value={fieldWidth}
              onChangeText={setFieldWidth}
              keyboardType="numeric"
              placeholder="4.6"
              placeholderTextColor={ROBOT_CONFIG.COLORS.textMuted}
            />
          </View>
          <View style={styles.inputGroup}>
            <Text style={styles.inputLabel}>Couloirs calculés</Text>
            <Text style={styles.calcValue}>{calculatedCorridors} rangs</Text>
          </View>
        </View>

        <TouchableOpacity
          style={[
            styles.mainActionButton,
            { backgroundColor: isMissionActive ? ROBOT_CONFIG.COLORS.accentAmber : ROBOT_CONFIG.COLORS.accentGreen },
          ]}
          onPress={isMissionActive ? onStopMission : onStartWeeding}
        >
          <Text style={styles.mainActionButtonText}>
            {isMissionActive
              ? `🛑 STOPPER MISSION (${missionState.progress}%)`
              : '🤖 DÉSHERBER TOUT LE CHAMP'}
          </Text>
        </TouchableOpacity>
        {missionState.step_label ? (
          <Text style={styles.statusLabel}>{missionState.step_label}</Text>
        ) : null}
      </View>

      {/* Commandes Manuelles Outils M5 / M6 */}
      <View style={styles.card}>
        <Text style={styles.sectionTitle}>🛠️ OUTILS DE DÉSHERBAGE (M5 / M6)</Text>
        <View style={styles.toolRow}>
          <TouchableOpacity
            style={styles.toolBtn}
            onPress={() => onSendRaw('PULSE:5:50:4000')}
          >
            <Text style={styles.toolBtnText}>⬇️ DESCENDRE DENTS (M5)</Text>
          </TouchableOpacity>
          <TouchableOpacity
            style={styles.toolBtn}
            onPress={() => onSendRaw('PULSE:5:-50:4000')}
          >
            <Text style={styles.toolBtnText}>⬆️ RELEVER DENTS (M5)</Text>
          </TouchableOpacity>
        </View>

        <View style={styles.toolRow}>
          <TouchableOpacity
            style={[styles.toolBtn, { backgroundColor: '#15803d' }]}
            onPress={() => onSendRaw('M6:70')}
          >
            <Text style={styles.toolBtnText}>🌀 FRAISE 23cm ON (M6:70)</Text>
          </TouchableOpacity>
          <TouchableOpacity
            style={[styles.toolBtn, { backgroundColor: '#b91c1c' }]}
            onPress={() => onSendRaw('STOP:6')}
          >
            <Text style={styles.toolBtnText}>⏹️ ARRÊT FRAISE (STOP:6)</Text>
          </TouchableOpacity>
        </View>
      </View>

      {/* Statistiques Agronomiques */}
      <View style={styles.card}>
        <Text style={styles.sectionTitle}>📊 STATISTIQUES EN TEMPS RÉEL</Text>
        <View style={styles.statGrid}>
          <Text style={styles.statItem}>
            Adventices : {agronomics.weeds_destroyed} / {agronomics.weeds_total}
          </Text>
          <Text style={styles.statItem}>
            Distance : {agronomics.distance_traveled_m?.toFixed(1) || '0.0'} m
          </Text>
          <Text style={styles.statItem}>
            Couloir actif : {agronomics.current_corridor || 1} / {calculatedCorridors}
          </Text>
          <Text style={styles.statItem}>
            Obstacle : {agronomics.obstacle ? '⚠️ PRÉSENT' : '✅ LIBRE'}
          </Text>
        </View>
      </View>
    </View>
  );
};

const styles = StyleSheet.create({
  container: {
    padding: 12,
    gap: 12,
  },
  card: {
    backgroundColor: ROBOT_CONFIG.COLORS.panelBg,
    borderRadius: 8,
    padding: 12,
    borderWidth: 1,
    borderColor: ROBOT_CONFIG.COLORS.border,
  },
  sectionTitle: {
    color: ROBOT_CONFIG.COLORS.accentCyan,
    fontSize: 12,
    fontWeight: '700',
    marginBottom: 8,
  },
  row: {
    flexDirection: 'row',
    gap: 8,
    alignItems: 'center',
    marginBottom: 10,
  },
  inputGroup: {
    flex: 1,
  },
  inputLabel: {
    color: ROBOT_CONFIG.COLORS.textMuted,
    fontSize: 10,
    marginBottom: 4,
  },
  input: {
    backgroundColor: ROBOT_CONFIG.COLORS.cardBg,
    color: ROBOT_CONFIG.COLORS.textMain,
    borderWidth: 1,
    borderColor: ROBOT_CONFIG.COLORS.border,
    borderRadius: 6,
    paddingHorizontal: 8,
    paddingVertical: 6,
    fontSize: 13,
    fontFamily: 'monospace',
  },
  calcValue: {
    color: ROBOT_CONFIG.COLORS.accentGreen,
    fontSize: 14,
    fontWeight: 'bold',
    paddingVertical: 6,
  },
  mainActionButton: {
    paddingVertical: 10,
    borderRadius: 6,
    alignItems: 'center',
  },
  mainActionButtonText: {
    color: '#ffffff',
    fontSize: 13,
    fontWeight: 'bold',
  },
  statusLabel: {
    color: ROBOT_CONFIG.COLORS.textMuted,
    fontSize: 11,
    marginTop: 6,
    textAlign: 'center',
  },
  toolRow: {
    flexDirection: 'row',
    gap: 8,
    marginTop: 6,
  },
  toolBtn: {
    flex: 1,
    backgroundColor: ROBOT_CONFIG.COLORS.cardBg,
    borderWidth: 1,
    borderColor: ROBOT_CONFIG.COLORS.border,
    paddingVertical: 8,
    borderRadius: 6,
    alignItems: 'center',
  },
  toolBtnText: {
    color: ROBOT_CONFIG.COLORS.textMain,
    fontSize: 11,
    fontWeight: '600',
  },
  statGrid: {
    flexDirection: 'row',
    flexWrap: 'wrap',
    gap: 8,
  },
  statItem: {
    color: ROBOT_CONFIG.COLORS.textMain,
    fontSize: 11,
    width: '48%',
  },
});

