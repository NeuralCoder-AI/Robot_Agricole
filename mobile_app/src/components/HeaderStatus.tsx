/**
 * mobile_app/src/components/HeaderStatus.tsx
 * Barre supérieure d'état du Robot Agricole ESP32-S3 :
 * Statuts WebSocket, ESP32 physique, Boutons ARM / ESTOP, Mode Couplé et Paramètres Matériel.
 */

import React from 'react';
import { StyleSheet, View, Text, TouchableOpacity } from 'react-native';
import { ROBOT_CONFIG } from '../config/constants';

interface HeaderStatusProps {
  wsConnected: boolean;
  hwConnected: boolean;
  simulationLocked: boolean;
  executionMode: 'SIMULATION_ONLY' | 'COUPLED';
  isArmed: boolean;
  speedKmh: number;
  onToggleArm: () => void;
  onOpenHardwareModal: () => void;
}

export const HeaderStatus: React.FC<HeaderStatusProps> = ({
  wsConnected,
  hwConnected,
  simulationLocked,
  executionMode,
  isArmed,
  speedKmh,
  onToggleArm,
  onOpenHardwareModal,
}) => {
  return (
    <View style={styles.container}>
      <View style={styles.leftGroup}>
        <View style={styles.badgeGroup}>
          <View
            style={[
              styles.dot,
              { backgroundColor: wsConnected ? ROBOT_CONFIG.COLORS.accentGreen : ROBOT_CONFIG.COLORS.accentRed },
            ]}
          />
          <Text style={styles.badgeText}>{wsConnected ? 'SERVEUR EN LIGNE' : 'HORS LIGNE'}</Text>
        </View>

        <TouchableOpacity style={styles.hwBadge} onPress={onOpenHardwareModal}>
          <Text style={styles.hwBadgeLabel}>
            {executionMode === 'COUPLED'
              ? hwConnected
                ? '🔗 ESP32 CONNECTÉ'
                : '⚠️ ESP32 DÉCONNECTÉ'
              : '💻 SIMULATION SEULE'}
          </Text>
        </TouchableOpacity>

        {simulationLocked && (
          <View style={styles.lockedBadge}>
            <Text style={styles.lockedText}>🔒 SIMULATION VERROUILLÉE</Text>
          </View>
        )}
      </View>

      <View style={styles.rightGroup}>
        <Text style={styles.speedText}>{speedKmh.toFixed(1)} km/h</Text>
        <TouchableOpacity
          style={[
            styles.armButton,
            { backgroundColor: isArmed ? ROBOT_CONFIG.COLORS.accentRed : ROBOT_CONFIG.COLORS.accentGreen },
          ]}
          onPress={onToggleArm}
        >
          <Text style={styles.armButtonText}>{isArmed ? '🛑 ESTOP' : '⚡ ARMER'}</Text>
        </TouchableOpacity>
      </View>
    </View>
  );
};

const styles = StyleSheet.create({
  container: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    paddingHorizontal: 12,
    paddingVertical: 8,
    backgroundColor: ROBOT_CONFIG.COLORS.panelBg,
    borderBottomWidth: 1,
    borderBottomColor: ROBOT_CONFIG.COLORS.border,
  },
  leftGroup: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 8,
  },
  badgeGroup: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 5,
  },
  dot: {
    width: 9,
    height: 9,
    borderRadius: 5,
  },
  badgeText: {
    color: ROBOT_CONFIG.COLORS.textMuted,
    fontSize: 11,
    fontWeight: '700',
  },
  hwBadge: {
    backgroundColor: ROBOT_CONFIG.COLORS.cardBg,
    paddingHorizontal: 8,
    paddingVertical: 4,
    borderRadius: 6,
    borderWidth: 1,
    borderColor: ROBOT_CONFIG.COLORS.border,
  },
  hwBadgeLabel: {
    color: ROBOT_CONFIG.COLORS.accentCyan,
    fontSize: 11,
    fontWeight: '600',
  },
  lockedBadge: {
    backgroundColor: '#7f1d1d',
    paddingHorizontal: 6,
    paddingVertical: 3,
    borderRadius: 4,
  },
  lockedText: {
    color: '#fca5a5',
    fontSize: 10,
    fontWeight: '700',
  },
  rightGroup: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 12,
  },
  speedText: {
    color: ROBOT_CONFIG.COLORS.accentCyan,
    fontSize: 14,
    fontWeight: 'bold',
    fontFamily: 'monospace',
  },
  armButton: {
    paddingHorizontal: 14,
    paddingVertical: 6,
    borderRadius: 6,
  },
  armButtonText: {
    color: '#ffffff',
    fontSize: 12,
    fontWeight: 'bold',
  },
});

