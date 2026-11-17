/**
 * mobile_app/src/components/HardwareModal.tsx
 * Modal de configuration matérielle pour le couplage ESP32-S3 Firmware v2 :
 * Mode d'exécution, choix du transport (Wi-Fi, Socket, USB-UART) et connexion physique.
 */

import React, { useState } from 'react';
import { StyleSheet, View, Text, Modal, TouchableOpacity, TextInput } from 'react-native';
import { ROBOT_CONFIG } from '../config/constants';

interface HardwareModalProps {
  visible: boolean;
  onClose: () => void;
  executionMode: 'SIMULATION_ONLY' | 'COUPLED';
  setExecutionMode: (mode: 'SIMULATION_ONLY' | 'COUPLED') => void;
  hwConnected: boolean;
  hwTransport: 'WIFI_HTTP' | 'SOCKET_TCP_WS' | 'USB_UART';
  setHwTransport: (t: 'WIFI_HTTP' | 'SOCKET_TCP_WS' | 'USB_UART') => void;
  hwTarget: string;
  setHwTarget: (target: string) => void;
  onConnectHardware: () => void;
  onDisconnectHardware: () => void;
}

export const HardwareModal: React.FC<HardwareModalProps> = ({
  visible,
  onClose,
  executionMode,
  setExecutionMode,
  hwConnected,
  hwTransport,
  setHwTransport,
  hwTarget,
  setHwTarget,
  onConnectHardware,
  onDisconnectHardware,
}) => {
  const [targetInput, setTargetInput] = useState<string>(hwTarget);

  return (
    <Modal visible={visible} animationType="slide" transparent>
      <View style={styles.backdrop}>
        <View style={styles.modalCard}>
          <Text style={styles.title}>🔌 CONFIGURATION MATÉRIELLE ESP32-S3</Text>

          {/* Choix Mode d'Exécution */}
          <Text style={styles.sectionLabel}>MODE D'EXÉCUTION :</Text>
          <View style={styles.btnRow}>
            <TouchableOpacity
              style={[
                styles.modeBtn,
                executionMode === 'SIMULATION_ONLY' && styles.modeBtnActive,
              ]}
              onPress={() => setExecutionMode('SIMULATION_ONLY')}
            >
              <Text
                style={[
                  styles.modeBtnText,
                  executionMode === 'SIMULATION_ONLY' && styles.modeBtnTextActive,
                ]}
              >
                💻 SIMULATION SEULE
              </Text>
            </TouchableOpacity>

            <TouchableOpacity
              style={[
                styles.modeBtn,
                executionMode === 'COUPLED' && styles.modeBtnActive,
              ]}
              onPress={() => setExecutionMode('COUPLED')}
            >
              <Text
                style={[
                  styles.modeBtnText,
                  executionMode === 'COUPLED' && styles.modeBtnTextActive,
                ]}
              >
                🔗 COUPLAGE ROBOT RÉEL
              </Text>
            </TouchableOpacity>
          </View>

          {/* Choix Transport si mode COUPLED */}
          {executionMode === 'COUPLED' && (
            <>
              <Text style={styles.sectionLabel}>LIAISON MATÉRIELLE :</Text>
              <View style={styles.btnRow}>
                {(['WIFI_HTTP', 'SOCKET_TCP_WS', 'USB_UART'] as const).map((t) => (
                  <TouchableOpacity
                    key={t}
                    style={[
                      styles.transportBtn,
                      hwTransport === t && styles.transportBtnActive,
                    ]}
                    onPress={() => setHwTransport(t)}
                  >
                    <Text
                      style={[
                        styles.transportBtnText,
                        hwTransport === t && styles.transportBtnTextActive,
                      ]}
                    >
                      {t === 'WIFI_HTTP' ? '📶 WI-FI' : t === 'SOCKET_TCP_WS' ? '🌐 SOCKET' : '🔌 USB'}
                    </Text>
                  </TouchableOpacity>
                ))}
              </View>

              <Text style={styles.sectionLabel}>ADRESSE IP / PORT :</Text>
              <TextInput
                style={styles.input}
                value={targetInput}
                onChangeText={(text) => {
                  setTargetInput(text);
                  setHwTarget(text);
                }}
                placeholder={ROBOT_CONFIG.DEFAULT_ESP32_IP}
                placeholderTextColor={ROBOT_CONFIG.COLORS.textMuted}
              />

              <TouchableOpacity
                style={[
                  styles.actionBtn,
                  { backgroundColor: hwConnected ? ROBOT_CONFIG.COLORS.accentRed : ROBOT_CONFIG.COLORS.accentGreen },
                ]}
                onPress={hwConnected ? onDisconnectHardware : onConnectHardware}
              >
                <Text style={styles.actionBtnText}>
                  {hwConnected ? '❌ DÉCONNECTER LE ROBOT RÉEL' : '⚡ CONNECTER L\'ESP32-S3'}
                </Text>
              </TouchableOpacity>
            </>
          )}

          <TouchableOpacity style={styles.closeBtn} onPress={onClose}>
            <Text style={styles.closeBtnText}>FERMER</Text>
          </TouchableOpacity>
        </View>
      </View>
    </Modal>
  );
};

const styles = StyleSheet.create({
  backdrop: {
    flex: 1,
    backgroundColor: 'rgba(0, 0, 0, 0.75)',
    justifyContent: 'center',
    alignItems: 'center',
    padding: 16,
  },
  modalCard: {
    width: '100%',
    maxWidth: 420,
    backgroundColor: ROBOT_CONFIG.COLORS.panelBg,
    borderRadius: 12,
    borderWidth: 1,
    borderColor: ROBOT_CONFIG.COLORS.border,
    padding: 16,
    gap: 12,
  },
  title: {
    color: ROBOT_CONFIG.COLORS.accentCyan,
    fontSize: 14,
    fontWeight: 'bold',
    textAlign: 'center',
  },
  sectionLabel: {
    color: ROBOT_CONFIG.COLORS.textMuted,
    fontSize: 11,
    fontWeight: '700',
  },
  btnRow: {
    flexDirection: 'row',
    gap: 8,
  },
  modeBtn: {
    flex: 1,
    backgroundColor: ROBOT_CONFIG.COLORS.cardBg,
    borderWidth: 1,
    borderColor: ROBOT_CONFIG.COLORS.border,
    paddingVertical: 8,
    borderRadius: 6,
    alignItems: 'center',
  },
  modeBtnActive: {
    backgroundColor: '#0369a1',
    borderColor: ROBOT_CONFIG.COLORS.accentCyan,
  },
  modeBtnText: {
    color: ROBOT_CONFIG.COLORS.textMain,
    fontSize: 11,
    fontWeight: '600',
  },
  modeBtnTextActive: {
    color: '#ffffff',
    fontWeight: 'bold',
  },
  transportBtn: {
    flex: 1,
    backgroundColor: ROBOT_CONFIG.COLORS.cardBg,
    borderWidth: 1,
    borderColor: ROBOT_CONFIG.COLORS.border,
    paddingVertical: 6,
    borderRadius: 6,
    alignItems: 'center',
  },
  transportBtnActive: {
    backgroundColor: '#0284c7',
    borderColor: ROBOT_CONFIG.COLORS.accentCyan,
  },
  transportBtnText: {
    color: ROBOT_CONFIG.COLORS.textMain,
    fontSize: 11,
  },
  transportBtnTextActive: {
    color: '#ffffff',
    fontWeight: 'bold',
  },
  input: {
    backgroundColor: ROBOT_CONFIG.COLORS.cardBg,
    color: ROBOT_CONFIG.COLORS.textMain,
    borderWidth: 1,
    borderColor: ROBOT_CONFIG.COLORS.border,
    borderRadius: 6,
    paddingHorizontal: 10,
    paddingVertical: 8,
    fontSize: 13,
    fontFamily: 'monospace',
  },
  actionBtn: {
    paddingVertical: 10,
    borderRadius: 6,
    alignItems: 'center',
  },
  actionBtnText: {
    color: '#ffffff',
    fontSize: 12,
    fontWeight: 'bold',
  },
  closeBtn: {
    backgroundColor: ROBOT_CONFIG.COLORS.cardBg,
    paddingVertical: 8,
    borderRadius: 6,
    alignItems: 'center',
    borderWidth: 1,
    borderColor: ROBOT_CONFIG.COLORS.border,
  },
  closeBtnText: {
    color: ROBOT_CONFIG.COLORS.textMuted,
    fontSize: 12,
    fontWeight: '600',
  },
});

