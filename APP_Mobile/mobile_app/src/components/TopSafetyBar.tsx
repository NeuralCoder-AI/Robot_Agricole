import React, { useEffect, useState } from 'react';
import {
  StyleSheet,
  Text,
  TextInput,
  TouchableOpacity,
  View,
} from 'react-native';
import { RobotOperatingMode } from '../types/robot';

interface TopSafetyBarProps {
  wsUrl: string;
  detectedIp: string;
  isAutoIp: boolean;
  onUpdateWsUrl: (newUrl: string, manual?: boolean) => void;
  onResetAutoIp: () => void;
  connected: boolean;
  latencyMs: number | null;
  armed: number;
  mode: RobotOperatingMode;
  onArm: () => void;
  onEstop: () => void;
  onToggleMode: (targetMode: RobotOperatingMode) => void;
}

export const TopSafetyBar: React.FC<TopSafetyBarProps> = ({
  wsUrl,
  detectedIp,
  isAutoIp,
  onUpdateWsUrl,
  onResetAutoIp,
  connected,
  latencyMs,
  armed,
  mode,
  onArm,
  onEstop,
  onToggleMode,
}) => {
  const [editingIp, setEditingIp] = useState(false);
  const [draftUrl, setDraftUrl] = useState(wsUrl);

  useEffect(() => {
    if (!editingIp) {
      setDraftUrl(wsUrl);
    }
  }, [wsUrl, editingIp]);

  const handleSaveIp = () => {
    setEditingIp(false);
    const trimmed = draftUrl.trim();
    if (trimmed) {
      const normalized =
        trimmed.startsWith('ws://') || trimmed.startsWith('wss://')
          ? trimmed
          : `ws://${trimmed.replace(/^https?:\/\//, '')}${
              trimmed.includes(':') ? '' : ':8000'
            }${trimmed.endsWith('/ws') ? '' : '/ws'}`;
      onUpdateWsUrl(normalized, true);
    }
  };

  return (
    <View style={styles.container}>
      {/* Ligne 1 : État Réseau WebSocket, Latence PING/PONG & Badge IP Auto */}
      <View style={styles.networkRow}>
        <View style={styles.statusGroup}>
          <View
            style={[
              styles.statusDot,
              { backgroundColor: connected ? '#00e676' : '#ff1744' },
            ]}
          />
          <Text
            style={[
              styles.statusText,
              { color: connected ? '#00e676' : '#ff5252' },
            ]}
          >
            {connected ? 'EN LIGNE' : 'HORS LIGNE'}
          </Text>
          <View style={styles.latencyBadge}>
            <Text style={styles.latencyText}>
              PING: {latencyMs !== null && connected ? `${latencyMs} ms` : '-- ms'}
            </Text>
          </View>
        </View>

        <TouchableOpacity
          style={[
            styles.ipConfigBtn,
            isAutoIp ? styles.ipConfigBtnAuto : styles.ipConfigBtnManual,
          ]}
          onPress={() => {
            setDraftUrl(wsUrl);
            setEditingIp(!editingIp);
          }}
        >
          <Text style={styles.ipConfigBtnText} numberOfLines={1}>
            {isAutoIp ? `IP Auto : ${detectedIp}` : `IP Manuelle : ${detectedIp}`}
          </Text>
        </TouchableOpacity>
      </View>

      {editingIp && (
        <View style={styles.ipInputRow}>
          <TextInput
            style={styles.ipInput}
            value={draftUrl}
            onChangeText={setDraftUrl}
            placeholder="ws://<IP_PC>:8000/ws"
            placeholderTextColor="#607d8b"
            autoCapitalize="none"
            autoCorrect={false}
          />
          <TouchableOpacity style={styles.ipApplyBtn} onPress={handleSaveIp}>
            <Text style={styles.ipApplyBtnText}>FORCER IP</Text>
          </TouchableOpacity>
          <TouchableOpacity
            style={styles.ipAutoResetBtn}
            onPress={() => {
              setEditingIp(false);
              onResetAutoIp();
            }}
          >
            <Text style={styles.ipAutoResetBtnText}>AUTO</Text>
          </TouchableOpacity>
        </View>
      )}

      {/* Ligne 2 : Bouton ARM (GPIO 38) + Bouton Prioritaire ESTOP Rouge Vif */}
      <View style={styles.safetyActionsRow}>
        <TouchableOpacity
          style={[
            styles.armButton,
            armed === 1 ? styles.armButtonActive : styles.armButtonDisarmed,
          ]}
          onPress={onArm}
          activeOpacity={0.8}
        >
          <Text style={styles.armButtonIcon}>{armed === 1 ? '⚡' : '🔒'}</Text>
          <View>
            <Text
              style={[
                styles.armButtonTitle,
                { color: armed === 1 ? '#041e0c' : '#cfd8dc' },
              ]}
            >
              {armed === 1 ? 'ARMÉ (ACTIF)' : 'DÉSARMÉ'}
            </Text>
            <Text
              style={[
                styles.armButtonSub,
                { color: armed === 1 ? '#073b17' : '#90a4ae' },
              ]}
            >
              GPIO 38 BTS7960 {armed === 1 ? 'ON' : 'OFF'}
            </Text>
          </View>
        </TouchableOpacity>

        <TouchableOpacity
          style={styles.estopButton}
          onPress={onEstop}
          activeOpacity={0.75}
        >
          <Text style={styles.estopTitle}>🛑 ESTOP URGENCE</Text>
          <Text style={styles.estopSub}>COUPURE IMMÉDIATE 8 PWM</Text>
        </TouchableOpacity>
      </View>

      {/* Ligne 3 : Commutateur rapide de mode MANUEL vs AUTOMATIQUE (IA) */}
      <View style={styles.modeSwitchRow}>
        <TouchableOpacity
          style={[
            styles.modeTab,
            mode === 'MANUAL' && styles.modeTabManualActive,
          ]}
          onPress={() => onToggleMode('MANUAL')}
        >
          <Text
            style={[
              styles.modeTabText,
              mode === 'MANUAL' && styles.modeTabTextActive,
            ]}
          >
            🕹️ MODE MANUEL (OPÉRATEUR)
          </Text>
        </TouchableOpacity>

        <TouchableOpacity
          style={[
            styles.modeTab,
            mode === 'AUTO' && styles.modeTabAutoActive,
          ]}
          onPress={() => onToggleMode('AUTO')}
        >
          <Text
            style={[
              styles.modeTabText,
              mode === 'AUTO' && styles.modeTabTextActive,
            ]}
          >
            🤖 AUTOMATIQUE (IA / RTK)
          </Text>
        </TouchableOpacity>
      </View>
    </View>
  );
};

const styles = StyleSheet.create({
  container: {
    backgroundColor: '#0d1422',
    borderBottomWidth: 2,
    borderBottomColor: '#1e2f4d',
    paddingHorizontal: 12,
    paddingTop: 8,
    paddingBottom: 10,
    gap: 8,
  },
  networkRow: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
  },
  statusGroup: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 8,
  },
  statusDot: {
    width: 10,
    height: 10,
    borderRadius: 5,
  },
  statusText: {
    fontSize: 12,
    fontWeight: '900',
    letterSpacing: 0.8,
  },
  latencyBadge: {
    backgroundColor: '#152036',
    paddingHorizontal: 8,
    paddingVertical: 3,
    borderRadius: 6,
    borderWidth: 1,
    borderColor: '#25395c',
  },
  latencyText: {
    color: '#00e5ff',
    fontSize: 11,
    fontWeight: '800',
  },
  ipConfigBtn: {
    backgroundColor: '#162238',
    paddingHorizontal: 10,
    paddingVertical: 5,
    borderRadius: 6,
    borderWidth: 1,
    maxWidth: 205,
  },
  ipConfigBtnAuto: {
    borderColor: '#00e676',
    backgroundColor: '#0d261c',
  },
  ipConfigBtnManual: {
    borderColor: '#ff9100',
    backgroundColor: '#291d0e',
  },
  ipConfigBtnText: {
    color: '#e0f2f1',
    fontSize: 11,
    fontWeight: '800',
  },
  ipInputRow: {
    flexDirection: 'row',
    gap: 6,
    marginTop: 2,
  },
  ipInput: {
    flex: 1,
    backgroundColor: '#070b12',
    color: '#ffffff',
    borderWidth: 1,
    borderColor: '#00e5ff',
    borderRadius: 6,
    paddingHorizontal: 10,
    paddingVertical: 6,
    fontSize: 13,
  },
  ipApplyBtn: {
    backgroundColor: '#00e5ff',
    justifyContent: 'center',
    paddingHorizontal: 12,
    borderRadius: 6,
  },
  ipApplyBtnText: {
    color: '#001f29',
    fontWeight: '900',
    fontSize: 11,
  },
  ipAutoResetBtn: {
    backgroundColor: '#00e676',
    justifyContent: 'center',
    paddingHorizontal: 10,
    borderRadius: 6,
  },
  ipAutoResetBtnText: {
    color: '#041e0c',
    fontWeight: '900',
    fontSize: 11,
  },
  safetyActionsRow: {
    flexDirection: 'row',
    gap: 10,
  },
  armButton: {
    flex: 1,
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'center',
    paddingVertical: 12,
    paddingHorizontal: 10,
    borderRadius: 10,
    gap: 8,
    borderWidth: 2,
  },
  armButtonActive: {
    backgroundColor: '#00e676',
    borderColor: '#69f0ae',
  },
  armButtonDisarmed: {
    backgroundColor: '#263238',
    borderColor: '#455a64',
  },
  armButtonIcon: {
    fontSize: 20,
  },
  armButtonTitle: {
    fontSize: 14,
    fontWeight: '900',
    letterSpacing: 0.5,
  },
  armButtonSub: {
    fontSize: 10,
    fontWeight: '700',
  },
  estopButton: {
    flex: 1.25,
    backgroundColor: '#ff1744',
    borderColor: '#ff8a80',
    borderWidth: 2,
    borderRadius: 10,
    paddingVertical: 12,
    paddingHorizontal: 10,
    alignItems: 'center',
    justifyContent: 'center',
  },
  estopTitle: {
    color: '#ffffff',
    fontSize: 15,
    fontWeight: '900',
    letterSpacing: 0.8,
  },
  estopSub: {
    color: '#ffebee',
    fontSize: 10,
    fontWeight: '800',
  },
  modeSwitchRow: {
    flexDirection: 'row',
    backgroundColor: '#080d16',
    borderRadius: 8,
    padding: 3,
    borderWidth: 1,
    borderColor: '#1c2b45',
  },
  modeTab: {
    flex: 1,
    paddingVertical: 8,
    alignItems: 'center',
    borderRadius: 6,
  },
  modeTabManualActive: {
    backgroundColor: '#00b0ff',
  },
  modeTabAutoActive: {
    backgroundColor: '#ff9100',
  },
  modeTabText: {
    color: '#78909c',
    fontSize: 11,
    fontWeight: '900',
    letterSpacing: 0.5,
  },
  modeTabTextActive: {
    color: '#050a12',
  },
});
