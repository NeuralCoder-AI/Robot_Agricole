/**
 * static/js/core/state_store.js
 * Magasin d'état réactif partagé pour le Dashboard PC et l'App Mobile.
 */

class StateStore {
    constructor() {
        this.state = {
            armed: 0,
            mode: "MANUAL",
            executionMode: "SIMULATION_ONLY",
            simulationLocked: false,
            hardware: {
                connected: false,
                connecting: false,
                transport: "WIFI_HTTP",
                host: "192.168.4.1",
                port: 80,
                serial_port: "/dev/ttyUSB0",
                baudrate: 115200,
                use_xor_checksum: false,
                latency_ms: 0,
                firmware_version: "",
                status_line: "",
                last_error: ""
            },
            agronomics: {
                weeds_destroyed: 0,
                seeds_sown: 0,
                corridors_total: 10,
                seed_spacing_cm: 20.0,
                field_length_m: 15.0,
                field_width_m: 4.6,
                vision: {}
            },
            camera: {
                connected: false,
                is_synthetic: false,
                device_path: "/dev/video0",
                camera_pitch_deg: 32.0,
                auto_pitch: true
            },
            missionActive: false,
            missionPaused: false,
            missionSnapshot: {
                mission_active: false,
                mission_paused: false,
                mission_status: "IDLE",
                mission_type: "manual",
                mission_label: "4. Conduite Manuelle",
                current_action: "En attente",
                phase_label: "En attente",
                progress: 0,
                row_index: 1,
                total_rows_est: 10,
                distance_m: 0.0,
                elapsed_s: 0.0,
                parcel_length_m: 10.0,
                parcel_width_m: 4.6,
                direction_mode: "AUTO",
                locked_side: null,
                hardware_connected: false,
                hardware_port: "",
                armed: 0,
                camera_connected: false,
                camera_device: "/dev/video0",
                camera_pitch_deg: 32.0,
                auto_pitch: true,
                recent_logs: []
            },
            motors: Array(8).fill(0)
        };
        this.listeners = new Set();
    }

    getState() {
        return this.state;
    }

    update(patch) {
        if (!patch || typeof patch !== "object") return;
        this.state = { ...this.state, ...patch };
        if (patch.hardware) {
            this.state.hardware = { ...this.state.hardware, ...patch.hardware };
            this.state.simulationLocked = !!this.state.hardware.simulation_locked;
            if (this.state.hardware.execution_mode) {
                this.state.executionMode = this.state.hardware.execution_mode;
            }
        }
        if (patch.agronomics) {
            this.state.agronomics = { ...this.state.agronomics, ...patch.agronomics };
        }
        if (patch.camera) {
            this.state.camera = { ...this.state.camera, ...patch.camera };
        }
        const snap = patch.mission_snapshot || patch.missionSnapshot;
        if (snap && typeof snap === "object") {
            this.state.missionSnapshot = { ...this.state.missionSnapshot, ...snap };
            this.state.missionActive = Boolean(this.state.missionSnapshot.mission_active);
            this.state.missionPaused = Boolean(this.state.missionSnapshot.mission_paused);
        }
        this.notify();
    }

    subscribe(listener) {
        this.listeners.add(listener);
        listener(this.state);
        return () => this.listeners.delete(listener);
    }

    notify() {
        for (const listener of this.listeners) {
            try {
                listener(this.state);
            } catch (e) {
                console.error("[StateStore] listener error:", e);
            }
        }
    }
}

export const stateStore = new StateStore();

