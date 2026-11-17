import React, { useState } from 'react';
import {
  ScrollView,
  StyleSheet,
  Switch,
  Text,
  TextInput,
  TouchableOpacity,
  View,
} from 'react-native';
import {
  AgronomicMetrics,
  GlobalNvsConfig,
  MotorNvsConfig,
  TelemetryData,
  UartLogEntry,
} from '../types/robot';
import {
  calculateXorChecksum,
  formatCommandWithChecksum,
  MOTOR_DESCRIPTIONS,
} from '../utils/protocol';

interface TelemetryConfigTabProps {
  telemetry: TelemetryData;
  agronomics: AgronomicMetrics;
  motorConfigs: Record<number, MotorNvsConfig>;
  globalConfig: GlobalNvsConfig;
  uartLogs: UartLogEntry[];
  onSendRawCommand: (cmd: string, withChecksum: boolean) => void;
  onReadMotorConfig: (motor: number) => void;
  onReadGlobalConfig: () => void;
}

export const TelemetryConfigTab: React.FC<TelemetryConfigTabProps> = ({
  telemetry,
  agronomics,
  motorConfigs,
  globalConfig,
  uartLogs,
  onSendRawCommand,
  onReadMotorConfig,
  onReadGlobalConfig,
}) => {
  const [selectedMotor, setSelectedMotor] = useState<number>(1);
  const currentCfg = motorConfigs[selectedMotor] || {
    motor: selectedMotor,
    inv: 0,
    min: 0,
    max: 100,
    ramp: 100,
    tmax: 0,
  };

  const [invDraft, setInvDraft] = useState<number>(currentCfg.inv);
  const [minDraft, setMinDraft] = useState<string>(String(currentCfg.min));
  const [maxDraft, setMaxDraft] = useState<string>(String(currentCfg.max));
  const [rampDraft, setRampDraft] = useState<string>(String(currentCfg.ramp));
  const [tmaxDraft, setTmaxDraft] = useState<string>(String(currentCfg.tmax));
  const [wdtDraft, setWdtDraft] = useState<string>(String(globalConfig.wdt));
  const [uwdtDraft, setUwdtDraft] = useState<string>(String(globalConfig.uwdt));

  const [consoleInput, setConsoleInput] = useState<string>('');
  const [useXorChecksum, setUseXorChecksum] = useState<boolean>(true);
  const [filterTelemFrames, setFilterTelemFrames] = useState<boolean>(true);

  const selectMotorAndFetch = (m: number) => {
    setSelectedMotor(m);
    const existing = motorConfigs[m];
    if (existing) {
      setInvDraft(existing.inv);
      setMinDraft(String(existing.min));
      setMaxDraft(String(existing.max));
      setRampDraft(String(existing.ramp));
      setTmaxDraft(String(existing.tmax));
    }
    onReadMotorConfig(m);
  };

  const applyMotorParameters = () => {
    onSendRawCommand(`INV:${selectedMotor}:${invDraft}`, useXorChecksum);
    onSendRawCommand(`MIN:${selectedMotor}:${minDraft}`, useXorChecksum);
    onSendRawCommand(`MAX:${selectedMotor}:${maxDraft}`, useXorChecksum);
    onSendRawCommand(`RAMP:${selectedMotor}:${rampDraft}`, useXorChecksum);
    onSendRawCommand(`TMAX:${selectedMotor}:${tmaxDraft}`, useXorChecksum);
    setTimeout(() => onReadMotorConfig(selectedMotor), 250);
    setTimeout(() => onReadGlobalConfig(), 350);
  };

  const applyWatchdogs = () => {
    onSendRawCommand(`WDT:${wdtDraft}`, useXorChecksum);
    onSendRawCommand(`UWDT:${uwdtDraft}`, useXorChecksum);
    setTimeout(() => onReadGlobalConfig(), 250);
  };

  const handleSaveNvs = () => {
    onSendRawCommand('SAVE', useXorChecksum);
    setTimeout(() => onReadGlobalConfig(), 250);
  };

  const handleResetDefaults = () => {
    onSendRawCommand('DEFAULTS', useXorChecksum);
    setTimeout(() => {
      onReadMotorConfig(selectedMotor);
      onReadGlobalConfig();
    }, 300);
  };

  const handleSendCustom = () => {
    if (!consoleInput.trim()) return;
    onSendRawCommand(consoleInput.trim(), useXorChecksum);
    setConsoleInput('');
  };

  const previewWithCrc = consoleInput.trim()
    ? formatCommandWithChecksum(consoleInput.trim(), true)
    : 'Ex: M1:50*43';

  return (
    <ScrollView style={styles.container} contentContainerStyle={styles.content}>
      {/* 1. MÉTRIQUES AGRONOMIQUES & INCLINAISON 3D TEMPS RÉEL */}
      <View style={styles.card}>
        <View style={styles.cardHeader}>
          <Text style={styles.cardTitle}>1. CAPTEURS 3D & MÉTRIQUES AGRONOMIQUES</Text>
          <Text style={styles.uptimeText}>Uptime: {(telemetry.timestamp / 1000).toFixed(1)}s</Text>
        </View>
        <View style={styles.metricsGrid}>
          <View style={styles.metricBox}>
            <Text style={styles.metricLabel}>VITESSE</Text>
            <Text style={styles.metricValue}>{agronomics.speed_kmh} km/h</Text>
          </View>
          <View style={styles.metricBox}>
            <Text style={styles.metricLabel}>ROULIS / TANGAGE</Text>
            <Text style={styles.metricValue}>
              {agronomics.roll}° / {agronomics.pitch}°
            </Text>
          </View>
          <View style={styles.metricBox}>
            <Text style={styles.metricLabel}>ADVENTICES DÉTRUITES</Text>
            <Text style={[styles.metricValue, { color: '#00e676' }]}>
              {agronomics.weeds_destroyed} / {agronomics.weeds_total}
            </Text>
          </View>
          <View style={styles.metricBox}>
            <Text style={styles.metricLabel}>GRAINES SEMÉES</Text>
            <Text style={[styles.metricValue, { color: '#ffd600' }]}>
              {agronomics.seeds_sown}
            </Text>
          </View>
        </View>
      </View>

      {/* 2. TÉLÉMÉTRIE #T DES 8 MOTEURS BTS7960 (-100% à +100%) */}
      <View style={styles.card}>
        <View style={styles.cardHeader}>
          <Text style={styles.cardTitle}>2. TÉLÉMÉTRIE #T — 8 MOTEURS (-100% À +100%)</Text>
          <Text style={styles.telemFrameBadge}>
            #T {telemetry.armed} {(telemetry.motors || []).join(',')}
          </Text>
        </View>

        {[1, 2, 3, 4, 5, 6, 7, 8].map((mIdx) => {
          const val = telemetry.motors[mIdx - 1] || 0;
          const absPct = Math.min(100, Math.abs(val));
          const info = MOTOR_DESCRIPTIONS[mIdx];
          return (
            <View key={mIdx} style={styles.motorRow}>
              <View style={styles.motorLabelCol}>
                <Text style={styles.motorName}>{info.name}</Text>
                <Text style={styles.motorGpio}>{info.gpio}</Text>
              </View>
              <View style={styles.biBarTrack}>
                <View style={styles.biBarCenterLine} />
                {val < 0 && (
                  <View
                    style={[
                      styles.biBarNeg,
                      { width: `${absPct / 2}%`, right: '50%' },
                    ]}
                  />
                )}
                {val > 0 && (
                  <View
                    style={[
                      styles.biBarPos,
                      { width: `${absPct / 2}%`, left: '50%' },
                    ]}
                  />
                )}
              </View>
              <Text
                style={[
                  styles.motorValText,
                  val > 0 && { color: '#00e676' },
                  val < 0 && { color: '#ff9100' },
                ]}
              >
                {val > 0 ? `+${val}%` : `${val}%`}
              </Text>
            </View>
          );
        })}
      </View>

      {/* 3. CONFIGURATION NVS (ESP32 PREFERENCES) */}
      <View style={styles.card}>
        <View style={styles.cardHeader}>
          <Text style={styles.cardTitle}>3. RÉGLAGES NVS (CFG:n, GCFG, SAVE)</Text>
          <View
            style={[
              styles.savedBadge,
              { backgroundColor: globalConfig.saved === 1 ? '#0a3820' : '#3d2408' },
            ]}
          >
            <Text
              style={{
                color: globalConfig.saved === 1 ? '#00e676' : '#ff9100',
                fontSize: 10,
                fontWeight: '900',
              }}
            >
              {globalConfig.saved === 1 ? 'NVS: saved=1' : 'NVS: saved=0 (Modifié)'}
            </Text>
          </View>
        </View>

        {/* Sélecteur de moteur 1 à 8 */}
        <View style={styles.motorSelectorRow}>
          {[1, 2, 3, 4, 5, 6, 7, 8].map((m) => (
            <TouchableOpacity
              key={m}
              style={[
                styles.motorSelectBtn,
                selectedMotor === m && styles.motorSelectBtnActive,
              ]}
              onPress={() => selectMotorAndFetch(m)}
            >
              <Text
                style={[
                  styles.motorSelectText,
                  selectedMotor === m && styles.motorSelectTextActive,
                ]}
              >
                M{m}
              </Text>
            </TouchableOpacity>
          ))}
        </View>

        <View style={styles.cfgReadBar}>
          <Text style={styles.cfgSelectedLabel}>
            {MOTOR_DESCRIPTIONS[selectedMotor]?.name} ({MOTOR_DESCRIPTIONS[selectedMotor]?.role})
          </Text>
          <TouchableOpacity
            style={styles.readCfgBtn}
            onPress={() => onReadMotorConfig(selectedMotor)}
          >
            <Text style={styles.readCfgBtnText}>🔄 Lire CFG:{selectedMotor}</Text>
          </TouchableOpacity>
        </View>

        {/* Grille d'édition INV, MIN, MAX, RAMP, TMAX */}
        <View style={styles.cfgFormGrid}>
          <View style={styles.cfgField}>
            <Text style={styles.cfgFieldLabel}>INV:{selectedMotor}:b (0/1)</Text>
            <TouchableOpacity
              style={[
                styles.invToggleBtn,
                invDraft === 1 && styles.invToggleBtnActive,
              ]}
              onPress={() => setInvDraft(invDraft === 1 ? 0 : 1)}
            >
              <Text style={styles.invToggleText}>
                {invDraft === 1 ? '1 (Inversé)' : '0 (Normal)'}
              </Text>
            </TouchableOpacity>
          </View>

          <View style={styles.cfgField}>
            <Text style={styles.cfgFieldLabel}>MIN (0–90 %)</Text>
            <TextInput
              style={styles.cfgInput}
              keyboardType="numeric"
              value={minDraft}
              onChangeText={setMinDraft}
            />
          </View>

          <View style={styles.cfgField}>
            <Text style={styles.cfgFieldLabel}>MAX (MIN–100 %)</Text>
            <TextInput
              style={styles.cfgInput}
              keyboardType="numeric"
              value={maxDraft}
              onChangeText={setMaxDraft}
            />
          </View>

          <View style={styles.cfgField}>
            <Text style={styles.cfgFieldLabel}>RAMP (1–1000 %/s)</Text>
            <TextInput
              style={styles.cfgInput}
              keyboardType="numeric"
              value={rampDraft}
              onChangeText={setRampDraft}
            />
          </View>

          <View style={styles.cfgField}>
            <Text style={styles.cfgFieldLabel}>TMAX (0–86400 s)</Text>
            <TextInput
              style={styles.cfgInput}
              keyboardType="numeric"
              value={tmaxDraft}
              onChangeText={setTmaxDraft}
            />
          </View>

          <View style={styles.cfgField}>
            <Text style={styles.cfgFieldLabel}>Action M{selectedMotor}</Text>
            <TouchableOpacity style={styles.applyCfgBtn} onPress={applyMotorParameters}>
              <Text style={styles.applyCfgBtnText}>APPLIQUER M{selectedMotor}</Text>
            </TouchableOpacity>
          </View>
        </View>

        {/* Paramètres globaux WDT & UWDT */}
        <View style={styles.globalCfgRow}>
          <View style={{ flex: 1 }}>
            <Text style={styles.cfgFieldLabel}>WDT (ms)</Text>
            <TextInput
              style={styles.cfgInput}
              keyboardType="numeric"
              value={wdtDraft}
              onChangeText={setWdtDraft}
            />
          </View>
          <View style={{ flex: 1 }}>
            <Text style={styles.cfgFieldLabel}>UWDT (ms)</Text>
            <TextInput
              style={styles.cfgInput}
              keyboardType="numeric"
              value={uwdtDraft}
              onChangeText={setUwdtDraft}
            />
          </View>
          <TouchableOpacity style={styles.applyWdtBtn} onPress={applyWatchdogs}>
            <Text style={styles.applyCfgBtnText}>SET WDT/UWDT</Text>
          </TouchableOpacity>
        </View>

        {/* Actions NVS : GCFG, SAVE, DEFAULTS */}
        <View style={styles.nvsActionRow}>
          <TouchableOpacity style={styles.nvsBtnRead} onPress={onReadGlobalConfig}>
            <Text style={styles.nvsBtnText}>🔍 LIRE GCFG</Text>
          </TouchableOpacity>
          <TouchableOpacity style={styles.nvsBtnSave} onPress={handleSaveNvs}>
            <Text style={styles.nvsBtnSaveText}>💾 SAVE NVS</Text>
          </TouchableOpacity>
          <TouchableOpacity style={styles.nvsBtnDefaults} onPress={handleResetDefaults}>
            <Text style={styles.nvsBtnText}>♻️ DEFAULTS</Text>
          </TouchableOpacity>
        </View>
      </View>

      {/* 4. MINI-CONSOLE UART AVEC CHECKSUM XOR *HH */}
      <View style={styles.card}>
        <View style={styles.cardHeader}>
          <Text style={styles.cardTitle}>4. MINI-CONSOLE UART1 (XOR *HH)</Text>
          <View style={styles.switchRow}>
            <Text style={styles.switchLabel}>Checksum *HH</Text>
            <Switch value={useXorChecksum} onValueChange={setUseXorChecksum} />
            <Text style={styles.switchLabel}>Filtrer #T</Text>
            <Switch value={filterTelemFrames} onValueChange={setFilterTelemFrames} />
          </View>
        </View>

        <View style={styles.consoleInputRow}>
          <TextInput
            style={styles.consoleInput}
            value={consoleInput}
            onChangeText={setConsoleInput}
            placeholder="Ex: STATUS, CFG:5, GCFG, M1:50..."
            placeholderTextColor="#546e7a"
            autoCapitalize="characters"
          />
          <TouchableOpacity style={styles.consoleSendBtn} onPress={handleSendCustom}>
            <Text style={styles.consoleSendBtnText}>ENVOYER</Text>
          </TouchableOpacity>
        </View>
        <Text style={styles.crcPreviewText}>
          Trame calculée avec XOR *HH : <Text style={{ color: '#00e5ff' }}>{previewWithCrc}</Text>
        </Text>

        <View style={styles.terminalBox}>
          {uartLogs
            .filter((item) => !(filterTelemFrames && item.line.startsWith('#T ')))
            .slice(0, 40)
            .map((entry) => (
              <Text
                key={entry.id}
                style={[
                  styles.logLine,
                  entry.direction === 'TX' && styles.logTx,
                  entry.line.startsWith('OK') && styles.logOk,
                  entry.line.startsWith('ERR:') && styles.logErr,
                  entry.line.startsWith('#D') && styles.logDone,
                ]}
              >
                [{entry.timestamp}] {entry.direction === 'TX' ? '>>' : '<<'} {entry.line}
              </Text>
            ))}
        </View>
      </View>
    </ScrollView>
  );
};

const styles = StyleSheet.create({
  container: { flex: 1, backgroundColor: '#070b12' },
  content: { padding: 12, gap: 12, paddingBottom: 32 },
  card: {
    backgroundColor: '#101829',
    borderRadius: 12,
    borderWidth: 1,
    borderColor: '#1f3052',
    padding: 12,
    gap: 10,
  },
  cardHeader: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    borderBottomWidth: 1,
    borderBottomColor: '#1d2d4e',
    paddingBottom: 6,
  },
  cardTitle: {
    color: '#00e5ff',
    fontSize: 12,
    fontWeight: '900',
  },
  uptimeText: {
    color: '#90a4ae',
    fontSize: 11,
    fontFamily: 'monospace',
  },
  metricsGrid: {
    flexDirection: 'row',
    flexWrap: 'wrap',
    gap: 8,
  },
  metricBox: {
    flex: 1,
    minWidth: '45%',
    backgroundColor: '#090f1b',
    padding: 10,
    borderRadius: 8,
    borderWidth: 1,
    borderColor: '#1a2947',
  },
  metricLabel: {
    color: '#78909c',
    fontSize: 10,
    fontWeight: '800',
  },
  metricValue: {
    color: '#00e5ff',
    fontSize: 15,
    fontWeight: '900',
    marginTop: 2,
  },
  telemFrameBadge: {
    color: '#69f0ae',
    fontSize: 10,
    fontFamily: 'monospace',
  },
  motorRow: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 8,
    paddingVertical: 3,
  },
  motorLabelCol: {
    width: 115,
  },
  motorName: {
    color: '#e0e6ed',
    fontSize: 11,
    fontWeight: '800',
  },
  motorGpio: {
    color: '#607d8b',
    fontSize: 9,
  },
  biBarTrack: {
    flex: 1,
    height: 12,
    backgroundColor: '#080d17',
    borderRadius: 6,
    position: 'relative',
    overflow: 'hidden',
    borderWidth: 1,
    borderColor: '#1c2c4c',
  },
  biBarCenterLine: {
    position: 'absolute',
    left: '50%',
    top: 0,
    bottom: 0,
    width: 1,
    backgroundColor: '#546e7a',
  },
  biBarPos: {
    position: 'absolute',
    top: 0,
    bottom: 0,
    backgroundColor: '#00e676',
  },
  biBarNeg: {
    position: 'absolute',
    top: 0,
    bottom: 0,
    backgroundColor: '#ff9100',
  },
  motorValText: {
    width: 46,
    textAlign: 'right',
    color: '#90a4ae',
    fontSize: 11,
    fontWeight: '900',
    fontFamily: 'monospace',
  },
  savedBadge: {
    paddingHorizontal: 8,
    paddingVertical: 3,
    borderRadius: 6,
  },
  motorSelectorRow: {
    flexDirection: 'row',
    gap: 4,
  },
  motorSelectBtn: {
    flex: 1,
    backgroundColor: '#162238',
    paddingVertical: 8,
    borderRadius: 6,
    alignItems: 'center',
    borderWidth: 1,
    borderColor: '#25395c',
  },
  motorSelectBtnActive: {
    backgroundColor: '#00e5ff',
    borderColor: '#84ffff',
  },
  motorSelectText: {
    color: '#b0bec5',
    fontWeight: '900',
    fontSize: 12,
  },
  motorSelectTextActive: {
    color: '#021622',
  },
  cfgReadBar: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
  },
  cfgSelectedLabel: {
    color: '#cfd8dc',
    fontSize: 11,
    fontWeight: '700',
  },
  readCfgBtn: {
    backgroundColor: '#1a2b4c',
    paddingHorizontal: 10,
    paddingVertical: 5,
    borderRadius: 6,
    borderWidth: 1,
    borderColor: '#00e5ff',
  },
  readCfgBtnText: {
    color: '#00e5ff',
    fontSize: 11,
    fontWeight: '800',
  },
  cfgFormGrid: {
    flexDirection: 'row',
    flexWrap: 'wrap',
    gap: 8,
  },
  cfgField: {
    width: '31%',
    gap: 4,
  },
  cfgFieldLabel: {
    color: '#90a4ae',
    fontSize: 10,
    fontWeight: '700',
  },
  cfgInput: {
    backgroundColor: '#080d17',
    borderWidth: 1,
    borderColor: '#263b5e',
    borderRadius: 6,
    color: '#ffffff',
    paddingHorizontal: 8,
    paddingVertical: 6,
    fontSize: 12,
    fontFamily: 'monospace',
  },
  invToggleBtn: {
    backgroundColor: '#19263e',
    paddingVertical: 8,
    borderRadius: 6,
    alignItems: 'center',
    borderWidth: 1,
    borderColor: '#304878',
  },
  invToggleBtnActive: {
    backgroundColor: '#ff9100',
  },
  invToggleText: {
    color: '#ffffff',
    fontSize: 11,
    fontWeight: '800',
  },
  applyCfgBtn: {
    backgroundColor: '#00e5ff',
    paddingVertical: 8,
    borderRadius: 6,
    alignItems: 'center',
  },
  applyCfgBtnText: {
    color: '#021b29',
    fontSize: 10,
    fontWeight: '900',
  },
  globalCfgRow: {
    flexDirection: 'row',
    alignItems: 'flex-end',
    gap: 8,
  },
  applyWdtBtn: {
    backgroundColor: '#00b0ff',
    paddingVertical: 8,
    paddingHorizontal: 10,
    borderRadius: 6,
  },
  nvsActionRow: {
    flexDirection: 'row',
    gap: 8,
    marginTop: 4,
  },
  nvsBtnRead: {
    flex: 1,
    backgroundColor: '#1c2b47',
    paddingVertical: 10,
    borderRadius: 8,
    alignItems: 'center',
  },
  nvsBtnSave: {
    flex: 1.2,
    backgroundColor: '#00e676',
    paddingVertical: 10,
    borderRadius: 8,
    alignItems: 'center',
  },
  nvsBtnDefaults: {
    flex: 1,
    backgroundColor: '#37474f',
    paddingVertical: 10,
    borderRadius: 8,
    alignItems: 'center',
  },
  nvsBtnText: {
    color: '#ffffff',
    fontSize: 11,
    fontWeight: '900',
  },
  nvsBtnSaveText: {
    color: '#032410',
    fontSize: 11,
    fontWeight: '900',
  },
  switchRow: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 6,
  },
  switchLabel: {
    color: '#90a4ae',
    fontSize: 10,
  },
  consoleInputRow: {
    flexDirection: 'row',
    gap: 8,
  },
  consoleInput: {
    flex: 1,
    backgroundColor: '#080c14',
    borderWidth: 1,
    borderColor: '#283d63',
    borderRadius: 6,
    color: '#ffffff',
    paddingHorizontal: 10,
    paddingVertical: 7,
    fontSize: 12,
    fontFamily: 'monospace',
  },
  consoleSendBtn: {
    backgroundColor: '#00e5ff',
    justifyContent: 'center',
    paddingHorizontal: 14,
    borderRadius: 6,
  },
  consoleSendBtnText: {
    color: '#021b29',
    fontSize: 11,
    fontWeight: '900',
  },
  crcPreviewText: {
    color: '#78909c',
    fontSize: 10,
    fontFamily: 'monospace',
  },
  terminalBox: {
    backgroundColor: '#060a10',
    borderRadius: 8,
    borderWidth: 1,
    borderColor: '#19263c',
    padding: 8,
    maxHeight: 220,
    gap: 2,
  },
  logLine: {
    color: '#90a4ae',
    fontSize: 11,
    fontFamily: 'monospace',
  },
  logTx: { color: '#80d8ff' },
  logOk: { color: '#69f0ae' },
  logErr: { color: '#ff5252', fontWeight: 'bold' },
  logDone: { color: '#ffd740', fontWeight: 'bold' },
});
