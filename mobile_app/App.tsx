/**
 * mobile_app/App.tsx - Conteneur mobile Arachide (< 115 lignes)
 */
import React, { useState } from 'react';
import { StyleSheet, View, SafeAreaView, StatusBar, TouchableOpacity, Text } from 'react-native';
import { ROBOT_CONFIG } from './src/config/constants';
import { useRobotConnection } from './src/hooks/useRobotConnection';
import { HeaderStatus } from './src/components/HeaderStatus';
import { WeedingTab } from './src/components/WeedingTab';
import { SeedingTab } from './src/components/SeedingTab';
import { ConsoleView } from './src/components/ConsoleView';
import { HardwareModal } from './src/components/HardwareModal';

export default function App() {
  const [tab, setTab] = useState<'WEEDING' | 'SEEDING' | 'RESEEDING'>('WEEDING');
  const [direction, setDirection] = useState<1 | -1>(1);
  const [fLen, setFLen] = useState('10.0');
  const [fWid, setFWid] = useState('4.6');
  const [spacing, setSpacing] = useState(25);
  const [tines, setTines] = useState(true);
  const [hwModal, setHwModal] = useState(false);
  const conn = useRobotConnection();
  const corridors = Math.max(1, Math.round((parseFloat(fWid) || 4.6) / 0.46));

  const runMission = (m: string) => conn.sendJson({
    type: 'mission_start', mission: m, field_length_m: parseFloat(fLen) || 10.0,
    field_width_m: parseFloat(fWid) || 4.6, spacing_cm: spacing, use_tines: tines,
    progression_direction: direction, direction: direction,
  });

  return (
    <SafeAreaView style={styles.c}>
      <StatusBar barStyle="light-content" backgroundColor={ROBOT_CONFIG.COLORS.bgDark} />
      <HeaderStatus
        wsConnected={conn.wsConnected} hwConnected={conn.hwConnected}
        simulationLocked={conn.simulationLocked} executionMode={conn.executionMode}
        isArmed={conn.isArmed} speedKmh={conn.agronomics.speed_kmh}
        onToggleArm={() => conn.sendRawCommand(conn.isArmed ? 'ESTOP' : 'ARM')}
        onOpenHardwareModal={() => setHwModal(true)}
      />
      <View style={styles.nav}>
        {(['WEEDING', 'SEEDING', 'RESEEDING'] as const).map((t) => (
          <TouchableOpacity key={t} style={[styles.btn, tab === t && styles.active]} onPress={() => setTab(t)}>
            <Text style={[styles.txt, tab === t && styles.activeTxt]}>
              {t === 'WEEDING' ? '🌿 DÉSHERBAGE' : t === 'SEEDING' ? '🌱 SEMENCE' : '🔍 RÉVISION'}
            </Text>
          </TouchableOpacity>
        ))}
      </View>
      <View style={styles.body}>
        {tab === 'WEEDING' ? (
          <WeedingTab
            fieldLength={fLen} setFieldLength={setFLen} fieldWidth={fWid} setFieldWidth={setFWid}
            calculatedCorridors={corridors} agronomics={conn.agronomics} missionState={conn.missionState}
            onSendRaw={conn.sendRawCommand} onStartWeeding={() => runMission('weeding')}
            onStopMission={() => conn.sendJson({ type: 'mission_stop' })}
          />
        ) : tab === 'SEEDING' ? (
          <SeedingTab
            seedSpacingCm={spacing} setSeedSpacingCm={setSpacing} seedUseTines={tines} setSeedUseTines={setTines}
            agronomics={conn.agronomics} missionState={conn.missionState} onSendRaw={conn.sendRawCommand}
            onStartSeeding={() => runMission('seeding')} onStopMission={() => conn.sendJson({ type: 'mission_stop' })}
          />
        ) : (
          <View style={{ padding: 12 }}>
            <View style={{ flexDirection: 'row', gap: 8, marginBottom: 12 }}>
              <TouchableOpacity
                style={[styles.dirBtn, direction === -1 && styles.dirActive]}
                onPress={() => setDirection(-1)}>
                <Text style={[styles.dirTxt, direction === -1 && styles.dirActiveTxt]}>⬅️ GAUCHE</Text>
              </TouchableOpacity>
              <TouchableOpacity
                style={[styles.dirBtn, direction === 1 && styles.dirActive]}
                onPress={() => setDirection(1)}>
                <Text style={[styles.dirTxt, direction === 1 && styles.dirActiveTxt]}>➡️ DROITE</Text>
              </TouchableOpacity>
            </View>
            <View style={{ flexDirection: 'row', flexWrap: 'wrap', gap: 8, marginBottom: 12 }}>
              <View style={styles.metricCard}>
                <Text style={styles.metricLbl}>Trous détectés</Text>
                <Text style={styles.metricVal}>{conn.agronomics?.gaps_detected_count ?? 0}</Text>
              </View>
              <View style={styles.metricCard}>
                <Text style={styles.metricLbl}>Plants morts</Text>
                <Text style={[styles.metricVal, { color: '#fb7185' }]}>{conn.agronomics?.dead_plants_count ?? 0}</Text>
              </View>
              <View style={styles.metricCard}>
                <Text style={styles.metricLbl}>Resemés</Text>
                <Text style={[styles.metricVal, { color: '#34d399' }]}>{conn.agronomics?.reseeded_seeds_count ?? 0}</Text>
              </View>
              <View style={styles.metricCard}>
                <Text style={styles.metricLbl}>Rangs révisés</Text>
                <Text style={[styles.metricVal, { color: '#38bdf8' }]}>{conn.agronomics?.reviewed_rows_count ?? 0}</Text>
              </View>
            </View>
            <TouchableOpacity style={styles.actionBtn} onPress={() => runMission('reseeding')}>
              <Text style={styles.actionTxt}>🚀 LANCER LA RÉVISION</Text>
            </TouchableOpacity>
          </View>
        )}
      </View>
      <ConsoleView logs={conn.consoleLogs} onClear={() => conn.setConsoleLogs([])} />
      <HardwareModal
        visible={hwModal} onClose={() => setHwModal(false)}
        executionMode={conn.executionMode} setExecutionMode={conn.setExecutionMode}
        hwConnected={conn.hwConnected} hwTransport={conn.hwTransport} setHwTransport={conn.setHwTransport}
        hwTarget={conn.hwTarget} setHwTarget={conn.setHwTarget}
        onConnectHardware={() => conn.sendJson({ type: 'hardware_connect', mode: conn.executionMode, transport: conn.hwTransport, target: conn.hwTarget })}
        onDisconnectHardware={() => conn.sendJson({ type: 'hardware_disconnect' })}
      />
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  c: { flex: 1, backgroundColor: ROBOT_CONFIG.COLORS.bgDark },
  nav: { flexDirection: 'row', backgroundColor: ROBOT_CONFIG.COLORS.panelBg, borderBottomWidth: 1, borderBottomColor: ROBOT_CONFIG.COLORS.border },
  btn: { flex: 1, paddingVertical: 10, alignItems: 'center' },
  active: { borderBottomWidth: 2, borderBottomColor: ROBOT_CONFIG.COLORS.accentCyan },
  txt: { color: ROBOT_CONFIG.COLORS.textMuted, fontSize: 11, fontWeight: '700' },
  activeTxt: { color: ROBOT_CONFIG.COLORS.accentCyan },
  body: { flex: 1 },
  dirBtn: { flex: 1, padding: 10, borderRadius: 8, backgroundColor: '#0f172a', alignItems: 'center', borderWidth: 1, borderColor: '#334155' },
  dirActive: { borderColor: '#00e5ff', backgroundColor: 'rgba(0, 229, 255, 0.15)' },
  dirTxt: { color: '#94a3b8', fontWeight: '700', fontSize: 12 },
  dirActiveTxt: { color: '#00e5ff' },
  metricCard: { width: '48%', backgroundColor: '#09101d', padding: 8, borderRadius: 6, borderWidth: 1, borderColor: '#1e293b' },
  metricLbl: { fontSize: 10, color: '#94a3b8' },
  metricVal: { fontSize: 16, fontWeight: '800', color: '#facc15' },
  actionBtn: { backgroundColor: '#d97706', padding: 12, borderRadius: 8, alignItems: 'center', marginTop: 8 },
  actionTxt: { color: '#fff', fontWeight: '800', fontSize: 13 },
});
