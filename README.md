# ⚓ Sentinel Line — Collaborative Marine Risk Assessment, Ad-hoc Hazard Sharing & Border Alert System

**1st Prize Winner 🏆** at the VAC *"IoT Enabled Application Development using ESP32"* Project Expo at PSG Institute of Technology and Applied Research (September 26, 2026).

---

## 🌊 Overview
Sentinel Line is an offline-first, location-aware marine safety and risk management system engineered specifically for small fishing vessels operating offshore (e.g., India–Sri Lanka IMBL in the Palk Strait).

When 4G internet disappears offshore, Sentinel Line executes edge processing on an ESP32 hardware node—providing early weather warnings, capsize protection, offline boat-to-boat hazard sharing via LoRa, and border geofencing.

---

## 🚀 Key Features

* 🌩️ **Weather Prediction & Capsize Safety:** Uses a **BMP280** sensor (with a breathable waterproof vent) for pressure drop tracking and an **MPU6050** for hull roll/pitch dynamics to sound early capsize warnings.
* 📡 **Offline LoRa Ad-Hoc Mesh:** Relays emergency hazard packets across neighboring boats without cellular networks.
* 📍 **IMBL Border Geofencing:** Neo-6M GPS computes live distance to treaty boundaries on-device, triggering proximity warnings before illegal crossings.
* 🚨 **Emergency GSM & Dashboard:** SIM800L dispatches direct SMS alerts with Google Maps links on SOS press, while streaming MQTT telemetry to a Leaflet.js fleet dashboard.
* 🤖 **Edge Processing Risk Engine:** Executes sensor fusion and dynamic risk scoring locally on the ESP32 to eliminate cloud latency and internet dependency.

---

## 🛠️ Hardware & Tech Stack

* **Microcontroller:** ESP32-WROOM-32
* **Sensors:** MPU6050 (Accelerometer/Gyroscope), BMP280 (Barometric Pressure/Temp), Neo-6M GPS
* **Connectivity:** LoRa (Radio Mesh), SIM800L (GSM/SMS), HiveMQ MQTT
* **Dashboard & Analytics:** Leaflet.js, JavaScript, HTML/CSS
* **Enclosure:** IP-rated waterproof casing with breathable membrane vent

---

## 👥 Team — Triode Titans
* **Team Members:** Muhammed Nadeem S, Mugesh B, Mukhesh S, Pranesh S, Vibash Duraimurugan R
* **Supervisors:** Prof. Susithra & Prof. Sujin
* **Institution:** PSG Institute of Technology and Applied Research (ECE Dept.)
