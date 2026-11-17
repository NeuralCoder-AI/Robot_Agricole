"""backend/api/network_utils.py - Fonctions de détection IP Wi-Fi et Hotspot."""
import os
import re
import socket
import subprocess
import time
from typing import List, Optional

SERVER_PORT = 8000
EXPO_PORT = 8081
WS_BRIDGE_PORT = 8082

VIRTUAL_ADAPTER_KEYWORDS = (
    "vethernet",
    "veth",
    "wsl",
    "hyper-v",
    "docker",
    "br-",
    "virbr",
    "vmware",
    "virtualbox",
    "vbox",
    "loopback",
    "pseudo",
    "teredo",
    "isatap",
    "bluetooth",
    "npcap",
    "tun",
    "tap",
    "tailscale",
    "wg",
)


def _is_valid_lan_or_hotspot_ip(ip: str) -> bool:
    """Vérifie qu'une IPv4 est une adresse LAN ou Partage de connexion (Hotspot) valide."""
    if not ip or ip.startswith("127.") or ip.startswith("169.254.") or ip.startswith("0."):
        return False
    # Exclure la plage Docker/WSL par défaut (172.17..172.31), mais CONSERVER 172.20.10.x (Hotspot iPhone iOS) !
    if ip.startswith("172."):
        if ip.startswith("172.20.10."):
            return True
        parts = ip.split(".")
        if len(parts) == 4 and parts[1].isdigit():
            second_octet = int(parts[1])
            if 16 <= second_octet <= 31:
                return False
    return True


_cached_ips_list: List[str] = []
_cached_ips_timestamp: float = 0.0

def detect_all_local_ips(deep_scan: bool = False) -> List[str]:
    """Retourne toutes les adresses IPv4 actives (Wi-Fi, Hotspot téléphone, Ethernet) sans les adaptateurs virtuels."""
    global _cached_ips_list, _cached_ips_timestamp
    now = time.time()
    if not deep_scan and _cached_ips_list and (now - _cached_ips_timestamp) < 3.0:
        return list(_cached_ips_list)

    candidates: List[str] = []

    def add_candidate(ip_str: str) -> None:
        ip_clean = ip_str.strip()
        if _is_valid_lan_or_hotspot_ip(ip_clean) and ip_clean not in candidates:
            candidates.append(ip_clean)

    # 1. Sondage de routage UDP en mémoire (fonctionne avec Internet OU sur un Hotspot local sans Internet)
    probe_targets = [
        ("8.8.8.8", 80),         # Route par défaut / Internet
        ("1.1.1.1", 80),         # Route secondaire
        ("172.20.10.1", 80),     # Passerelle Partage de connexion iPhone (iOS)
        ("192.168.43.1", 80),    # Passerelle Partage de connexion Android classique
        ("192.168.137.1", 80),   # Passerelle Point d'accès sans fil mobile Windows
        ("192.168.1.1", 80),     # Routeur Wi-Fi domestique standard
        ("192.168.0.1", 80),     # Routeur Wi-Fi secondaire
        ("10.0.0.1", 80),        # Réseau entreprise / Hotspot 10.x
    ]
    for target_ip, target_port in probe_targets:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.settimeout(0.02)
                s.connect((target_ip, target_port))
                ip = s.getsockname()[0]
                add_candidate(ip)
        except Exception:
            pass

    # 2. Inspection native des interfaces Linux (wlan0, wlp*, eth0, enp*, usb0, ip route get 1.1.1.1)
    if os.name != "nt":
        wifi_ips_linux: List[str] = []
        eth_ips_linux: List[str] = []

        # 2a. Route active via `ip -4 route get 1.1.1.1`
        try:
            route_out = subprocess.check_output(
                ["ip", "-4", "route", "get", "1.1.1.1"],
                stderr=subprocess.DEVNULL,
                timeout=1.0,
                encoding="utf-8",
                errors="replace",
            )
            m_src = re.search(r"\bsrc\s+(\d{1,3}(?:\.\d{1,3}){3})", route_out)
            m_dev = re.search(r"\bdev\s+(\S+)", route_out)
            if m_src:
                src_ip = m_src.group(1)
                dev_name = (m_dev.group(1).lower() if m_dev else "")
                if not any(k in dev_name for k in VIRTUAL_ADAPTER_KEYWORDS) and _is_valid_lan_or_hotspot_ip(src_ip):
                    if dev_name.startswith(("wlan", "wlp", "wlx", "wifi", "ap", "uap", "usb", "rndis")):
                        wifi_ips_linux.append(src_ip)
                    else:
                        eth_ips_linux.append(src_ip)
        except Exception:
            pass

        # 2b. Énumération des interfaces actives via `ip -4 -o addr show up`
        try:
            addr_out = subprocess.check_output(
                ["ip", "-4", "-o", "addr", "show", "up"],
                stderr=subprocess.DEVNULL,
                timeout=1.0,
                encoding="utf-8",
                errors="replace",
            )
            for raw_line in addr_out.splitlines():
                parts = raw_line.split()
                if len(parts) < 4:
                    continue
                ifname = parts[1].lower().rstrip(":")
                if ifname == "lo" or any(k in ifname for k in VIRTUAL_ADAPTER_KEYWORDS):
                    continue
                m_ip = re.search(r"inet\s+(\d{1,3}(?:\.\d{1,3}){3})", raw_line)
                if m_ip:
                    found_ip = m_ip.group(1)
                    if _is_valid_lan_or_hotspot_ip(found_ip):
                        if ifname.startswith(("wlan", "wlp", "wlx", "wifi", "ap", "uap", "usb", "rndis")):
                            if found_ip not in wifi_ips_linux:
                                wifi_ips_linux.append(found_ip)
                        else:
                            if found_ip not in eth_ips_linux:
                                eth_ips_linux.append(found_ip)
        except Exception:
            pass

        # 2c. Fallback `hostname -I` sous Linux
        try:
            host_out = subprocess.check_output(
                ["hostname", "-I"],
                stderr=subprocess.DEVNULL,
                timeout=1.0,
                encoding="utf-8",
                errors="replace",
            )
            for token in host_out.split():
                if re.match(r"^\d{1,3}(?:\.\d{1,3}){3}$", token):
                    if _is_valid_lan_or_hotspot_ip(token) and token not in eth_ips_linux:
                        eth_ips_linux.append(token)
        except Exception:
            pass

        for wip in reversed(wifi_ips_linux):
            if wip in candidates:
                candidates.remove(wip)
            candidates.insert(0, wip)
        for eip in eth_ips_linux:
            add_candidate(eip)

    # 3. Inspection des interfaces Windows via ipconfig (compatibilité multi-OS)
    if os.name == "nt" and (deep_scan or not candidates):
        try:
            out = subprocess.check_output(
                ["ipconfig"],
                stderr=subprocess.DEVNULL,
                timeout=1.0,
                encoding="cp850",
                errors="replace",
            )
            current_adapter = ""
            adapter_is_virtual = False
            adapter_is_wifi = False
            wifi_ips: List[str] = []
            other_ips: List[str] = []

            for raw_line in out.splitlines():
                line = raw_line.strip()
                if not line:
                    continue
                if not raw_line.startswith(" ") and line.endswith(":"):
                    current_adapter = line.lower()
                    adapter_is_virtual = any(k in current_adapter for k in VIRTUAL_ADAPTER_KEYWORDS)
                    adapter_is_wifi = any(
                        w in current_adapter for w in ("wi-fi", "wifi", "wlan", "sans fil", "wireless")
                    )
                    continue

                if adapter_is_virtual:
                    continue

                if "ipv4" in line.lower() or "ip address" in line.lower() or "adresse ip" in line.lower():
                    m = re.search(r"(\d{1,3}(?:\.\d{1,3}){3})", line)
                    if m:
                        found_ip = m.group(1)
                        if _is_valid_lan_or_hotspot_ip(found_ip):
                            if adapter_is_wifi:
                                wifi_ips.append(found_ip)
                            else:
                                other_ips.append(found_ip)

            for wip in wifi_ips:
                if wip in candidates:
                    candidates.remove(wip)
                candidates.insert(0, wip)
            for oip in other_ips:
                add_candidate(oip)
        except Exception:
            pass

    # 3. Fallback gethostbyname_ex
    try:
        hostname = socket.gethostname()
        _, _, ips = socket.gethostbyname_ex(hostname)
        for ip in ips:
            add_candidate(ip)
    except Exception:
        pass

    if not candidates:
        candidates.append("127.0.0.1")

    _cached_ips_list = list(candidates)
    _cached_ips_timestamp = now
    return candidates


def detect_local_wifi_ip(request_host: Optional[str] = None, deep_scan: bool = False) -> str:
    """Détecte en temps réel l'adresse IP locale Wi-Fi / Hotspot active du PC (jamais mise en cache)."""
    if request_host:
        clean_host = request_host.split(":")[0].strip()
        if (
            clean_host not in ("localhost", "127.0.0.1", "0.0.0.0", "testserver")
            and _is_valid_lan_or_hotspot_ip(clean_host)
        ):
            return clean_host

    ips = detect_all_local_ips(deep_scan=deep_scan)
    return ips[0] if ips else "127.0.0.1"


