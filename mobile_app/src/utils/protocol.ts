/**
 * Utilitaires du protocole UART1 ESP32-S3 Firmware v2 (115200 bauds, ASCII).
 * - Calcul du Checksum XOR *HH
 * - Formule différentielle DRIVE:a,d (Gauche M1,M3 = a+d ; Droite M2,M4 = a-d)
 * - Décodage des trames CFG:n, GCFG, #T et #D n
 */

import { GlobalNvsConfig, MotorNvsConfig } from '../types/robot';

/**
 * Calcule le checksum XOR hexadécimal sur 2 caractères majuscules (*HH).
 * Exemples officiels :
 * - calculateXorChecksum("PING") === "10" -> "PING*10"
 * - calculateXorChecksum("M1:50") === "43" -> "M1:50*43"
 * - calculateXorChecksum("ARM") === "5E" -> "ARM*5E"
 * - calculateXorChecksum("ESTOP") === "5D" -> "ESTOP*5D"
 */
export function calculateXorChecksum(cmdWithoutStar: string): string {
  let crc = 0;
  const clean = cmdWithoutStar.trim();
  for (let i = 0; i < clean.length; i++) {
    crc ^= clean.charCodeAt(i);
  }
  return crc.toString(16).toUpperCase().padStart(2, '0');
}

/**
 * Formate une commande ASCII avec son checksum XOR *HH optionnel.
 */
export function formatCommandWithChecksum(cmd: string, appendChecksum = true): string {
  const clean = cmd.trim();
  if (!clean) return '';
  if (!appendChecksum || clean.includes('*')) return clean;
  return `${clean}*${calculateXorChecksum(clean)}`;
}

/**
 * Calcule les vitesses des roues gauches (M1, M3) et droites (M2, M4)
 * selon la loi de mélange différentiel :
 * Gauche = clamp(a + d, -100, +100)
 * Droite = clamp(a - d, -100, +100)
 */
export function computeDifferentialDrive(a: number, d: number): { left: number; right: number } {
  const clamp = (val: number) => Math.max(-100, Math.min(100, Math.round(val)));
  const normalize = (val: number) => (val > -1 && val < 1 ? 0 : clamp(val));
  return {
    left: normalize(a + d),
    right: normalize(a - d),
  };
}

/**
 * Parse la réponse de CFG:n
 * Exemple : "M1 INV:0 MIN:0 MAX:100 RAMP:100 TMAX:0.0"
 */
export function parseMotorCfgResponse(line: string, fallbackMotor = 1): MotorNvsConfig | null {
  const clean = line.trim();
  if (!clean.startsWith('M')) return null;

  const mMatch = clean.match(/^M(\d+)/);
  const motor = mMatch ? parseInt(mMatch[1], 10) : fallbackMotor;

  const getVal = (key: string, def: number): number => {
    const regex = new RegExp(`${key}:([+-]?\\d+(?:\\.\\d+)?)`, 'i');
    const match = clean.match(regex);
    return match ? parseFloat(match[1]) : def;
  };

  return {
    motor,
    inv: Math.round(getVal('INV', 0)),
    min: Math.round(getVal('MIN', 0)),
    max: Math.round(getVal('MAX', 100)),
    ramp: Math.round(getVal('RAMP', 100)),
    tmax: getVal('TMAX', 0),
  };
}

/**
 * Parse la réponse de GCFG
 * Exemple : "GCFG WDT:2000 UWDT:1000 AUTOSAVE:1 SAVED:1 saved=1"
 */
export function parseGlobalCfgResponse(line: string): GlobalNvsConfig | null {
  const clean = line.trim();
  if (!clean.startsWith('GCFG')) return null;

  const getVal = (key: string, def: number): number => {
    const regex = new RegExp(`${key}[:=](\\d+)`, 'i');
    const match = clean.match(regex);
    return match ? parseInt(match[1], 10) : def;
  };

  return {
    wdt: getVal('WDT', 2000),
    uwdt: getVal('UWDT', 1000),
    autosave: getVal('AUTOSAVE', 1),
    saved: getVal('SAVED', 1),
  };
}

export const MOTOR_DESCRIPTIONS: Record<number, { name: string; role: string; gpio: string }> = {
  1: { name: 'M1 Av. Gauche', role: 'Traction Roue Avant Gauche', gpio: 'BTS7960 #1' },
  2: { name: 'M2 Av. Droite', role: 'Traction Roue Avant Droite', gpio: 'BTS7960 #2' },
  3: { name: 'M3 Ar. Gauche', role: 'Traction Roue Arrière Gauche', gpio: 'BTS7960 #3' },
  4: { name: 'M4 Ar. Droite', role: 'Traction Roue Arrière Droite', gpio: 'BTS7960 #4' },
  5: { name: 'M5 Dents Terrage', role: 'Vérin Relevage Dents', gpio: 'GPIO 8 / 9' },
  6: { name: 'M6 Fraise Désherb.', role: 'Fraise Rotative Bineuse', gpio: 'GPIO 10 / 11' },
  7: { name: 'M7 Panneaux Solaires', role: 'Déploiement Ailes Solaires', gpio: 'GPIO 12 / 13' },
  8: { name: 'M8 Semoir Précision', role: 'Distribution Semoir 3 Rangs', gpio: 'GPIO 14 / 21' },
};
