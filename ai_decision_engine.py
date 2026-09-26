#!/usr/bin/env python3
"""
IoMT Patient Vitals Monitor & Threshold Alert Engine
Connects directly to the ESP32-S3 live serial stream (COM6) or simulated stream,
parses JSON telemetry packets, and checks vitals against baseline thresholds
to alert caregivers of abnormal readings or emergency events.
"""

import sys
import json
import time
from datetime import datetime

try:
    import serial
except ImportError:
    serial = None


def evaluate_clinical_status(packet):
    """
    Checks patient vitals against baseline safety thresholds
    and generates clear alert notices for caregivers.
    """
    vitals = packet.get("vitals", {})
    alerts = packet.get("alerts", {})
    sensors = packet.get("sensors", {})
    uptime = packet.get("uptime_s", 0)

    hr = vitals.get("heart_rate", 0)
    spo2 = vitals.get("spo2", 0)
    temp_core = vitals.get("temp_core", 0.0)
    finger_on = vitals.get("finger_on", False)
    motion_g = vitals.get("motion_g", 1.0)
    steps = vitals.get("steps", 0)
    sos = alerts.get("sos", False)
    fall = alerts.get("fall", False)

    findings = []
    actions = []
    risk_level = "GREEN (NORMAL)"

    # 1. Emergency Alerts
    if sos:
        risk_level = "RED (EMERGENCY)"
        findings.append("🚨 SOS Button pressed by patient.")
        actions.append("Check on patient immediately.")

    if fall:
        risk_level = "RED (EMERGENCY)"
        findings.append(f"💥 High impact movement detected ({motion_g:.2f} g).")
        actions.append("Check on patient immediately for possible fall.")

    # 2. Oxygen Saturation (SpO2)
    if finger_on:
        if spo2 < 88 and spo2 > 0:
            risk_level = "RED (EMERGENCY)"
            findings.append(f"🫁 SpO2 reading very low: {spo2}%.")
            actions.append("Vitals look abnormal — notify a caregiver or nurse immediately.")
        elif spo2 < 92 and spo2 > 0:
            if risk_level != "RED (EMERGENCY)":
                risk_level = "YELLOW (WARNING)"
            findings.append(f"🫁 SpO2 reading below target: {spo2}%.")
            actions.append("Recheck sensor placement; notify caregiver if reading stays low.")

    # 3. Heart Rate
    if finger_on:
        if hr > 130:
            risk_level = "RED (EMERGENCY)"
            findings.append(f"❤️ Pulse rate very high: {hr} BPM.")
            actions.append("Vitals look abnormal — notify a caregiver.")
        elif hr > 100:
            if risk_level != "RED (EMERGENCY)":
                risk_level = "YELLOW (WARNING)"
            findings.append(f"❤️ Pulse rate elevated: {hr} BPM.")
            actions.append("Check on patient; recheck vitals after resting.")
        elif hr < 45 and hr > 0:
            risk_level = "RED (EMERGENCY)"
            findings.append(f"❤️ Pulse rate very low: {hr} BPM.")
            actions.append("Vitals look abnormal — notify a caregiver.")

    # 4. Core Body Temperature
    if temp_core >= 39.0:
        risk_level = "RED (EMERGENCY)"
        findings.append(f"🌡️ High temperature reading: {temp_core:.1f} °C.")
        actions.append("Vitals look abnormal — notify a caregiver.")
    elif temp_core >= 38.0:
        if risk_level != "RED (EMERGENCY)":
            risk_level = "YELLOW (WARNING)"
        findings.append(f"🌡️ Temperature elevated: {temp_core:.1f} °C.")
        actions.append("Monitor patient comfort; notify caregiver if fever persists.")
    elif temp_core > 0 and temp_core < 35.0:
        if risk_level != "RED (EMERGENCY)":
            risk_level = "YELLOW (WARNING)"
        findings.append(f"❄️ Temperature reading low: {temp_core:.1f} °C.")
        actions.append("Check sensor placement; ensure patient is warm.")

    # 5. Fallback if all is normal
    if not findings:
        findings.append("All measured vital signs are within normal baseline thresholds.")
        actions.append("Continue routine monitoring.")

    # Construct the Structured Alert Summary Object
    alert_summary = {
        "timestamp": datetime.now().isoformat(),
        "uptime_seconds": uptime,
        "overall_status": risk_level,
        "patient_metrics": {
            "heart_rate_bpm": hr if finger_on else "UNATTACHED",
            "spo2_percent": spo2 if finger_on else "UNATTACHED",
            "core_temp_c": temp_core if temp_core > 0 else "PENDING",
            "step_count": steps,
            "motion_g": round(motion_g, 2),
            "hardware_health": f"{sensors.get('active', 0)}/4 sensors online"
        },
        "findings": findings,
        "recommended_actions": actions
    }

    return alert_summary


def print_alert_card(summary):
    """Prints a clean, honest Vital Signs & Alert Card in the terminal."""
    status = summary["overall_status"]
    metrics = summary["patient_metrics"]

    print("\n" + "=" * 65)
    print(f" 📊 PATIENT VITALS & ALERT CHECKER — [{summary['timestamp'][:19]}]")
    print("=" * 65)
    print(f"  PATIENT STATUS       : {status}")
    print(f"  Hardware Health      : {metrics['hardware_health']}")
    print("-" * 65)
    print(f"  Vitals Snapshot:")
    print(f"    • Heart Rate  : {metrics['heart_rate_bpm']} BPM")
    print(f"    • SpO2 (O2)   : {metrics['spo2_percent']} %")
    print(f"    • Body Temp   : {metrics['core_temp_c']} °C")
    print(f"    • Step Count  : {metrics['step_count']} steps | Motion: {metrics['motion_g']} g")
    print("-" * 65)
    print("  📋 Threshold Observations:")
    for f in summary["findings"]:
        print(f"    - {f}")
    print("  💡 Caregiver Recommendations:")
    for a in summary["recommended_actions"]:
        print(f"    ▶ {a}")
    print("=" * 65 + "\n")


def listen_serial(port="COM6", baud=115200):
    """Listens to the ESP32 serial stream and checks every JSON telemetry packet."""
    if serial is None:
        print("[ERROR] pyserial is not installed. Install via: pip install pyserial")
        return

    print(f"[INFO] Connecting to ESP32-S3 on {port} at {baud} baud...")
    try:
        ser = serial.Serial(port, baud, timeout=1.0)
        time.sleep(1.0)
        print("[SUCCESS] Connected! Listening for real-time telemetry stream...\n")
    except Exception as e:
        print(f"[ERROR] Could not open {port}: {e}")
        print("Tip: Make sure pio device monitor is closed so the port is free.")
        return

    try:
        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            # Look for structured JSON telemetry lines
            if line.startswith("[JSON]"):
                json_str = line[6:].strip()
                try:
                    packet = json.loads(json_str)
                    summary = evaluate_clinical_status(packet)
                    print_alert_card(summary)
                except json.JSONDecodeError as err:
                    print(f"[WARN] Failed to parse JSON: {err}")
    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")
    finally:
        ser.close()


if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else "COM6"
    listen_serial(port)

