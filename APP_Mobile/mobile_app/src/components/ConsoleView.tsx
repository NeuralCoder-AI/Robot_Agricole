/**
 * mobile_app/src/components/ConsoleView.tsx
 * Affichage propre de la console de communication ESP32-S3 :
 * Filtre 100 % du bruit télémétrique (#T, PING, PONG) et affiche uniquement les vrais événements.
 */

import React, { useRef, useEffect } from 'react';
import { StyleSheet, View, Text, ScrollView, TouchableOpacity } from 'react-native';
import { ROBOT_CONFIG } from '../config/constants';

interface ConsoleViewProps {
  logs: string[];
  onClear: () => void;
}

export const ConsoleView: React.FC<ConsoleViewProps> = ({ logs, onClear }) => {
  const scrollRef = useRef<ScrollView | null>(null);

  useEffect(() => {
    scrollRef.current?.scrollToEnd({ animated: true });
  }, [logs]);

  return (
    <View style={styles.container}>
      <View style={styles.header}>
        <Text style={styles.title}>📟 CONSOLE MATÉRIELLE ESP32-S3</Text>
        <TouchableOpacity style={styles.clearBtn} onPress={onClear}>
          <Text style={styles.clearBtnText}>EFFACER</Text>
        </TouchableOpacity>
      </View>

      <ScrollView ref={scrollRef} style={styles.logBox}>
        {logs.length === 0 ? (
          <Text style={styles.emptyText}>Aucun événement matériel (bruit #T/PING filtré)</Text>
        ) : (
          logs.map((line, idx) => {
            const isTx = line.startsWith('TX ->');
            const isRx = line.startsWith('RX <-');
            return (
              <Text
                key={idx}
                style={[
                  styles.logLine,
                  isTx && styles.txLine,
                  isRx && styles.rxLine,
                ]}
              >
                {line}
              </Text>
            );
          })
        )}
      </ScrollView>
    </View>
  );
};

const styles = StyleSheet.create({
  container: {
    backgroundColor: ROBOT_CONFIG.COLORS.panelBg,
    borderRadius: 8,
    borderWidth: 1,
    borderColor: ROBOT_CONFIG.COLORS.border,
    padding: 10,
    marginHorizontal: 12,
    marginBottom: 12,
    height: 140,
  },
  header: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: 6,
  },
  title: {
    color: ROBOT_CONFIG.COLORS.accentCyan,
    fontSize: 11,
    fontWeight: '700',
  },
  clearBtn: {
    paddingHorizontal: 6,
    paddingVertical: 2,
    backgroundColor: ROBOT_CONFIG.COLORS.cardBg,
    borderRadius: 4,
  },
  clearBtnText: {
    color: ROBOT_CONFIG.COLORS.textMuted,
    fontSize: 10,
    fontWeight: '600',
  },
  logBox: {
    flex: 1,
    backgroundColor: '#050914',
    borderRadius: 4,
    padding: 6,
  },
  emptyText: {
    color: ROBOT_CONFIG.COLORS.textMuted,
    fontSize: 10,
    fontStyle: 'italic',
  },
  logLine: {
    color: ROBOT_CONFIG.COLORS.textMain,
    fontSize: 10,
    fontFamily: 'monospace',
    lineHeight: 14,
  },
  txLine: {
    color: ROBOT_CONFIG.COLORS.accentCyan,
  },
  rxLine: {
    color: ROBOT_CONFIG.COLORS.accentGreen,
  },
});

