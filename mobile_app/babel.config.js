const os = require('os');

/**
 * Détecte dynamiquement l'adresse IPv4 locale active du PC (Wi-Fi / Hotspot)
 * lors du démarrage de Metro (utile même en mode `npx expo start --tunnel`).
 */
function detectPcLanIp() {
  try {
    const interfaces = os.networkInterfaces();
    const virtualRegex = /vethernet|wsl|hyper-v|docker|vmware|virtualbox|vbox|loopback|bluetooth|npcap/i;
    const wifiRegex = /wi-fi|wifi|wlan|sans fil|wireless/i;
    const wifiCandidates = [];
    const otherCandidates = [];

    for (const [name, addrs] of Object.entries(interfaces)) {
      if (!addrs || virtualRegex.test(name)) continue;
      for (const addr of addrs) {
        if (addr.family === 'IPv4' && !addr.internal && !addr.address.startsWith('169.254.')) {
          if (wifiRegex.test(name)) {
            wifiCandidates.push(addr.address);
          } else {
            otherCandidates.push(addr.address);
          }
        }
      }
    }

    return wifiCandidates[0] || otherCandidates[0] || '';
  } catch {
    return '';
  }
}

const detectedIp = detectPcLanIp();
if (detectedIp && !process.env.EXPO_PUBLIC_PC_LAN_IP) {
  process.env.EXPO_PUBLIC_PC_LAN_IP = detectedIp;
}

module.exports = function (api) {
  api.cache(false);
  return {
    presets: ['babel-preset-expo'],
  };
};
