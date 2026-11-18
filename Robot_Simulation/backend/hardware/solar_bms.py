"""backend/hardware/solar_bms.py
Modèle Énergétique Photovoltaïque Sahélien (Niger) et Gestionnaire de Batterie (BMS).

Conforme à la Section 7.3 (« Alimentation solaire en contexte sahélien ») de l'État de l'Art :
- Irradiation sahélienne nominale : G_Sahel ∈ [5,0 ; 7,0] kWh/m²/jour (défaut 6,2 kWh/m²/jour au Niger).
- Déclassement thermique des cellules photovoltaïques sous climat chaud : T_amb ∈ [35 ; 45] °C.
- Surface PV embarquée M7 (panneaux repliés 173 × 106 cm = 1,83 m², déployés +99 cm -> ~2,80 m²).
- BMS à intégration coulombienne de l'état de charge (SoC), bilan instantané
  P_net(t) = P_PV(t) - Σ_{i=1..8} P_Mi(t) - P_UC+ESP32,
  et protection active contre les pics de courant sur les actionneurs M5 (dents) et M6 (fraise).
"""

import logging
import threading
from typing import Any, Dict, Optional, Sequence

logger = logging.getLogger("SolarBMS")

SAHEL_IRRADIANCE_KWH_M2_DAY_RANGE = (5.0, 7.0)
SAHEL_TEMP_C_RANGE = (35.0, 45.0)


class SolarBMS:
    """Gestionnaire énergétique solaire et BMS temps réel pour robot agricole sahélien (24 V LiFePO4)."""

    def __init__(
        self,
        battery_voltage_v: float = 24.0,
        battery_capacity_ah: float = 100.0,
        initial_soc: float = 0.85,
        pv_area_closed_m2: float = 1.8338,   # 1,73 m × 1,06 m
        pv_area_deployed_m2: float = 2.80,   # Déploiement complet M7 (envergure +99 cm)
        pv_efficiency_stc: float = 0.205,    # Cellules monocristallines 20,5 % à 25 °C
        temp_coeff_per_c: float = -0.0040,   # Déclassement -0,40 % / °C au-dessus de 25 °C
        base_electronics_power_w: float = 24.0,  # UC Linux (Jetson/mini-PC + caméra) + ESP32-S3
        max_battery_current_a: float = 45.0,
        sahel_daily_irradiance_kwh_m2: float = 6.2,
    ):
        self._lock = threading.RLock()
        self.battery_voltage_v = float(battery_voltage_v)
        self.battery_capacity_ah = float(battery_capacity_ah)
        self.battery_energy_wh = self.battery_voltage_v * self.battery_capacity_ah  # 2400 Wh

        self.soc = max(0.0, min(1.0, float(initial_soc)))
        self.pv_area_closed_m2 = float(pv_area_closed_m2)
        self.pv_area_deployed_m2 = float(pv_area_deployed_m2)
        self.pv_efficiency_stc = float(pv_efficiency_stc)
        self.temp_coeff_per_c = float(temp_coeff_per_c)
        self.base_electronics_power_w = float(base_electronics_power_w)
        self.max_battery_current_a = float(max_battery_current_a)
        self.sahel_daily_irradiance_kwh_m2 = float(
            max(SAHEL_IRRADIANCE_KWH_M2_DAY_RANGE[0], min(SAHEL_IRRADIANCE_KWH_M2_DAY_RANGE[1], sahel_daily_irradiance_kwh_m2))
        )

        # Puissances nominales maximales à 100 % PWM des 8 canaux moteurs M1..M8 (en Watts)
        self.motor_rated_power_w = [
            110.0,  # M1 Traction AG
            110.0,  # M2 Traction AD
            110.0,  # M3 Traction RG
            110.0,  # M4 Traction RD
            85.0,   # M5 Vérin dents sarclage (couple élevé en terre)
            160.0,  # M6 Fraise rotative de désherbage
            40.0,   # M7 Actionneur déploiement solaire
            35.0,   # M8 Distributeur semoir double tuyau
        ]

        # Télémétrie instantanée
        self.ambient_temp_c: float = 40.0       # Milieu de plage sahélienne [35 ; 45] °C
        self.irradiance_w_m2: float = 820.0     # Soleil sahélien zénithal ~650–950 W/m²
        self.solar_deploy_ratio: float = 0.0
        self.pv_power_w: float = 0.0
        self.motors_power_w: float = 0.0
        self.total_load_w: float = self.base_electronics_power_w
        self.net_power_w: float = 0.0
        self.battery_current_a: float = 0.0
        self.cell_temp_c: float = 58.0
        self.thermal_derating_factor: float = 0.868
        self.overcurrent_protection_active: bool = False
        self.m5_m6_load_clamped: bool = False
        self.low_battery_cutoff: bool = False

        self.step(dt=0.0, motor_pwms=[0] * 8, solar_deploy_ratio=0.0)

    def compute_pv_power_w(
        self,
        solar_deploy_ratio: float = 1.0,
        irradiance_w_m2: float = 820.0,
        ambient_temp_c: float = 40.0,
    ) -> Dict[str, float]:
        """Calcule la puissance photovoltaïque réelle sous contrainte thermique sahélienne (35–45 °C)."""
        ratio = max(0.0, min(1.0, float(solar_deploy_ratio)))
        area_m2 = self.pv_area_closed_m2 + ratio * (self.pv_area_deployed_m2 - self.pv_area_closed_m2)
        irr = max(0.0, float(irradiance_w_m2))
        t_amb = float(ambient_temp_c)

        # Échauffement cellule PV (NOCT ~ +18..22 °C en plein soleil sahélien)
        t_cell = t_amb + 0.022 * irr
        derating = max(0.60, min(1.05, 1.0 + self.temp_coeff_per_c * (t_cell - 25.0)))
        eff_real = self.pv_efficiency_stc * derating
        p_pv = irr * area_m2 * eff_real
        return {
            "pv_power_w": round(p_pv, 2),
            "effective_area_m2": round(area_m2, 3),
            "cell_temp_c": round(t_cell, 1),
            "thermal_derating_factor": round(derating, 4),
            "effective_efficiency": round(eff_real, 4),
        }

    def compute_motors_power_w(
        self,
        motor_pwms: Sequence[float],
        soil_load_multiplier_m5_m6: float = 1.0,
    ) -> Dict[str, Any]:
        """Estime la puissance cumulée des 8 moteurs M1..M8 et vérifie les pics sur M5/M6."""
        pwms = [float(motor_pwms[i]) if i < len(motor_pwms) else 0.0 for i in range(8)]
        per_motor_w = []
        soil_mult = max(0.5, float(soil_load_multiplier_m5_m6))

        for idx, pwm in enumerate(pwms):
            duty = min(1.0, abs(pwm) / 100.0)
            p = self.motor_rated_power_w[idx] * (duty ** 1.15)
            # Surcroît de couple lors de la pénétration des dents (M5) ou du sarclage en croûte/sable (M6)
            if idx in (4, 5) and duty > 0.05:
                p *= soil_mult
            per_motor_w.append(round(p, 2))

        total_motors_w = sum(per_motor_w)
        return {
            "motors_power_w": round(total_motors_w, 2),
            "per_motor_w": per_motor_w,
            "m5_m6_power_w": round(per_motor_w[4] + per_motor_w[5], 2),
        }

    def estimate_daily_harvest_wh(
        self,
        solar_deploy_ratio: float = 1.0,
        daily_irradiance_kwh_m2: Optional[float] = None,
        ambient_temp_c: float = 40.0,
    ) -> float:
        """Estime la récolte journalière (Wh/jour) pour G_Sahel ∈ [5,0 ; 7,0] kWh/m²/j."""
        g_kwh = float(
            self.sahel_daily_irradiance_kwh_m2
            if daily_irradiance_kwh_m2 is None
            else daily_irradiance_kwh_m2
        )
        ratio = max(0.0, min(1.0, float(solar_deploy_ratio)))
        area_m2 = self.pv_area_closed_m2 + ratio * (self.pv_area_deployed_m2 - self.pv_area_closed_m2)
        t_cell = float(ambient_temp_c) + 18.0
        derating = max(0.60, min(1.05, 1.0 + self.temp_coeff_per_c * (t_cell - 25.0)))
        return round(g_kwh * 1000.0 * area_m2 * self.pv_efficiency_stc * derating, 1)

    def step(
        self,
        dt: float,
        motor_pwms: Sequence[float],
        solar_deploy_ratio: float = 0.0,
        irradiance_w_m2: Optional[float] = None,
        ambient_temp_c: Optional[float] = None,
        soil_load_multiplier_m5_m6: float = 1.0,
    ) -> Dict[str, Any]:
        """Met à jour le bilan de puissance, le SoC coulombien et la protection BMS."""
        with self._lock:
            if irradiance_w_m2 is not None:
                self.irradiance_w_m2 = max(0.0, float(irradiance_w_m2))
            if ambient_temp_c is not None:
                self.ambient_temp_c = float(ambient_temp_c)
            self.solar_deploy_ratio = max(0.0, min(1.0, float(solar_deploy_ratio)))

            pv_info = self.compute_pv_power_w(
                solar_deploy_ratio=self.solar_deploy_ratio,
                irradiance_w_m2=self.irradiance_w_m2,
                ambient_temp_c=self.ambient_temp_c,
            )
            mot_info = self.compute_motors_power_w(
                motor_pwms=motor_pwms,
                soil_load_multiplier_m5_m6=soil_load_multiplier_m5_m6,
            )

            self.pv_power_w = pv_info["pv_power_w"]
            self.cell_temp_c = pv_info["cell_temp_c"]
            self.thermal_derating_factor = pv_info["thermal_derating_factor"]
            self.motors_power_w = mot_info["motors_power_w"]
            self.total_load_w = round(self.motors_power_w + self.base_electronics_power_w, 2)
            self.net_power_w = round(self.pv_power_w - self.total_load_w, 2)

            # Courant batterie (positif en décharge)
            draw_w = max(0.0, self.total_load_w - self.pv_power_w)
            self.battery_current_a = round(draw_w / max(12.0, self.battery_voltage_v), 2)

            # Protection surintensité / pic M5-M6 & seuil batterie critique
            self.overcurrent_protection_active = bool(self.battery_current_a > self.max_battery_current_a)
            self.m5_m6_load_clamped = bool(
                self.overcurrent_protection_active and mot_info["m5_m6_power_w"] > 120.0
            )
            self.low_battery_cutoff = bool(self.soc <= 0.12)

            # Intégration coulombienne exacte du SoC : ΔSoC = (P_net * Δt_h) / E_batt_Wh
            if dt > 0.0:
                dt_h = float(dt) / 3600.0
                delta_soc = (self.net_power_w * dt_h) / self.battery_energy_wh
                self.soc = round(max(0.0, min(1.0, self.soc + delta_soc)), 6)

            return self.get_status()

    def get_status(self) -> Dict[str, Any]:
        """Retourne la télémétrie énergétique complète exposable par /api/status et /api/hardware/status."""
        with self._lock:
            stored_wh = self.soc * self.battery_energy_wh
            usable_wh = max(0.0, (self.soc - 0.12) * self.battery_energy_wh)
            if self.net_power_w >= 0.0:
                autonomy_hours: Optional[float] = None
                autonomy_label = f"RECHARGE_SOLAIRE (+{self.net_power_w:.0f} W)"
            else:
                autonomy_hours = round(usable_wh / max(1.0, -self.net_power_w), 2)
                autonomy_label = f"{autonomy_hours:.1f} h"

            return {
                "battery_voltage_v": self.battery_voltage_v,
                "battery_capacity_ah": self.battery_capacity_ah,
                "battery_energy_wh": self.battery_energy_wh,
                "stored_energy_wh": round(stored_wh, 1),
                "soc_pct": round(self.soc * 100.0, 2),
                "sahel_irradiance_kwh_m2_day": self.sahel_daily_irradiance_kwh_m2,
                "irradiance_w_m2": round(self.irradiance_w_m2, 1),
                "ambient_temp_c": round(self.ambient_temp_c, 1),
                "cell_temp_c": round(self.cell_temp_c, 1),
                "thermal_derating_factor": self.thermal_derating_factor,
                "solar_deploy_ratio": round(self.solar_deploy_ratio, 3),
                "pv_power_w": self.pv_power_w,
                "motors_power_w": self.motors_power_w,
                "electronics_power_w": self.base_electronics_power_w,
                "total_load_w": self.total_load_w,
                "net_power_w": self.net_power_w,
                "battery_current_a": self.battery_current_a,
                "daily_harvest_estimate_wh": self.estimate_daily_harvest_wh(
                    solar_deploy_ratio=self.solar_deploy_ratio,
                    ambient_temp_c=self.ambient_temp_c,
                ),
                "autonomy_hours": autonomy_hours,
                "autonomy_label": autonomy_label,
                "overcurrent_protection_active": self.overcurrent_protection_active,
                "m5_m6_load_clamped": self.m5_m6_load_clamped,
                "low_battery_cutoff": self.low_battery_cutoff,
            }
