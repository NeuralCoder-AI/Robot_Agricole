/**
 * mobile_app/src/components/SeedingTab.tsx
 * Écran métier Semence Arachide (2 becs espacés de 46 cm) :
 * Espacement des graines (15..30 cm), option dents abaissées, impulsion semoir M8 et mission.
 */

import React from 'react';
import { StyleSheet, View, Text, TouchableOpacity } from 'react-native';
import { ROBOT_CONFIG } from '../config/constants';
import { AgronomicMetrics, MissionState } from '../types/robot';

interface SeedingTabProps {
  seedSpacingCm: number;
  setSeedSpacingCm: (val: number) => void;
  seedUseTines: boolean;
  setSeedUseTines: (val: boolean) => void;
  agronomics: AgronomicMetrics;
  missionState: MissionState;
  onSendRaw: (cmd: string) => void;
  onStartSeeding: () => void;
  onStopMission: () => void;
}

export const SeedingTab: React.FC<SeedingTabProps> = ({
  seedSpacingCm,
  setSeedSpacingCm,
  seedUseTines,
  setSeedUseTines,
  agronomics,
  missionState,
  onSendRaw,
  onStartSeeding,
  onStopMission,
}) => {
  const isMissionActive = Boolean(missionState.name && missionState.progress < 100);
  const spacingOptions = [15, 20, 25, 30];

  return (
    <View style={styles.container}>
      {/* Réglage Espacement Arachide */}
      <View style={styles.card}>
        <Text style={styles.sectionTitle}>🌱 PARAMÈTRES DU SEMOIR ARACHIDE (2 BECS / 46 CM)</Text>
        <Text style={styles.subTitle}>Espacement linéaire entre graines d'arachide :</Text>
        <View style={styles.spacingBtnRow}>
          {spacingOptions.map((opt) => (
            <TouchableOpacity
              key={opt}
              style={[
                styles.spacingBtn,
                seedSpacingCm === opt && styles.spacingBtnActive,
              ]}
              onPress={() => setSeedSpacingCm(opt)}
            >
              <Text
                style={[
                  styles.spacingBtnText,
                  seedSpacingCm === opt && styles.spacingBtnTextActive,
                ]}
              >
                {opt} cm
              </Text>
            </TouchableOpacity>
          ))}
        </View>

        {/* Option Dents M5 */}
        <TouchableOpacity
          style={styles.toggleRow}
          onPress={() => setSeedUseTines(!seedUseTines)}
        >
          <View
            style={[
              styles.checkbox,
              seedUseTines && { backgroundColor: ROBOT_CONFIG.COLORS.accentGreen },
            ]}
          />
          <Text style={styles.toggleLabel}>
            Semer avec les 4 dents d'enfouissement abaissées (M5)
          </Text>
        </TouchableOpacity>

        {/* Action Semer tout le champ */}
        <TouchableOpacity
          style={[
            styles.mainActionButton,
            { backgroundColor: isMissionActive ? ROBOT_CONFIG.COLORS.accentAmber : ROBOT_CONFIG.COLORS.accentGreen },
          ]}
          onPress={isMissionActive ? onStopMission : onStartSeeding}
        >
          <Text style={styles.mainActionButtonText}>
            {isMissionActive
              ? `🛑 STOPPER SEMIS (${missionState.progress}%)`
              : `🌱 SEMER TOUTE LA PARCELLE (${seedSpacingCm} CM)`}
          </Text>
        </TouchableOpacity>
        {missionState.step_label ? (
          <Text style={styles.statusLabel}>{missionState.step_label}</Text>
        ) : null}
      </View>

      {/* Test Manuel Semoir M8 */}
      <View style={styles.card}>
        <Text style={styles.sectionTitle}>⚙️ COMMANDE MANUELLE SEMOIR (M8)</Text>
        <TouchableOpacity
          style={styles.pulseBtn}
          onPress={() => onSendRaw('PULSE:8:80:400')}
        >
          <Text style={styles.pulseBtnText}>🌰 TEST 1 GRAINE (PULSE:8:80:400)</Text>
        </TouchableOpacity>
      </View>

      {/* Compteur Semence */}
      <View style={styles.card}>
        <Text style={styles.sectionTitle}>📊 SUIVI DU SEMIS</Text>
        <View style={styles.statGrid}>
          <Text style={styles.statItem}>
            Graines semées : {agronomics.seeds_sown}
          </Text>
          <Text style={styles.statItem}>
            Distance : {agronomics.distance_traveled_m?.toFixed(1) || '0.0'} m
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
  subTitle: {
    color: ROBOT_CONFIG.COLORS.textMuted,
    fontSize: 11,
    marginBottom: 8,
  },
  spacingBtnRow: {
    flexDirection: 'row',
    gap: 8,
    marginBottom: 12,
  },
  spacingBtn: {
    flex: 1,
    backgroundColor: ROBOT_CONFIG.COLORS.cardBg,
    borderWidth: 1,
    borderColor: ROBOT_CONFIG.COLORS.border,
    paddingVertical: 8,
    borderRadius: 6,
    alignItems: 'center',
  },
  spacingBtnActive: {
    backgroundColor: '#0284c7',
    borderColor: ROBOT_CONFIG.COLORS.accentCyan,
  },
  spacingBtnText: {
    color: ROBOT_CONFIG.COLORS.textMain,
    fontSize: 12,
    fontWeight: '600',
  },
  spacingBtnTextActive: {
    color: '#ffffff',
    fontWeight: 'bold',
  },
  toggleRow: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 8,
    marginBottom: 12,
  },
  checkbox: {
    width: 18,
    height: 18,
    borderRadius: 4,
    borderWidth: 1,
    borderColor: ROBOT_CONFIG.COLORS.border,
    backgroundColor: ROBOT_CONFIG.COLORS.cardBg,
  },
  toggleLabel: {
    color: ROBOT_CONFIG.COLORS.textMain,
    fontSize: 11,
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
  pulseBtn: {
    backgroundColor: ROBOT_CONFIG.COLORS.cardBg,
    borderWidth: 1,
    borderColor: ROBOT_CONFIG.COLORS.border,
    paddingVertical: 10,
    borderRadius: 6,
    alignItems: 'center',
  },
  pulseBtnText: {
    color: ROBOT_CONFIG.COLORS.textMain,
    fontSize: 12,
    fontWeight: '600',
  },
  statGrid: {
    flexDirection: 'row',
    gap: 16,
  },
  statItem: {
    color: ROBOT_CONFIG.COLORS.textMain,
    fontSize: 12,
  },
});

