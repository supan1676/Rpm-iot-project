#!/usr/bin/env python3
"""
IoMT Autonomous AI / LLM Clinical Decision Engine
Connects directly to the ESP32-S3 live serial stream (COM6) or simulated stream,
parses the high-precision JSON telemetry packets, and performs automated
clinical triage, risk assessment, and decision recommendations.
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
    Evaluates patient vitals using clinical thresholds (inspired by NEWS2 - National Early Warning Score)
    and generates structured decision recommendations for healthcare providers or automated AI workflows.
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
    risk_level = "GREEN (STABLE)"

    # 1. Emergency Alerts
    if sos:
        risk_level = "RED (CRITICAL EMERGENCY)"
        findings.append("🚨 SOS Panic Button triggered by patient.")
        actions.append("Dispatch rapid-response caregiver / emergency contact immediately.")

    if fall:
        risk_level = "RED (CRITICAL EMERGENCY)"
        findings.append(f"💥 High-G Fall impact detected ({motion_g:.2f} g).")
        actions.append("Initiate automated voice check-in and alert bedside nursing staff.")

    # 2. Oxygen Saturation (SpO2)
    if finger_on:
        if spo2 < 88 and spo2 > 0:
            risk_level = "RED (CRITICAL EMERGENCY)"
            findings.append(f"🫁 Severe Hypoxemia: SpO2 critically low at {spo2}%.")
            actions.append("Administer high-flow supplemental oxygen immediately.")
        elif spo2 < 92 and spo2 > 0:
            if risk_level != "RED (CRITICAL EMERGENCY)":
                risk_level = "YELLOW (WARNING)"
            findings.append(f"🫁 Mild Hypoxemia: SpO2 is {spo2}% (<92%).")
            actions.append("Recheck sensor placement; prepare nasal cannula support.")

    # 3. Heart Rate
    if finger_on:
        if hr > 130:
            risk_level = "RED (CRITICAL EMERGENCY)"
            findings.append(f"❤️ Severe Tachycardia: Heart rate at {hr} BPM.")
            actions.append("Check ECG for supraventricular tachycardia / atrial fibrillation.")
        elif hr > 100:
            if risk_level != "RED (CRITICAL EMERGENCY)":
                risk_level = "YELLOW (WARNING)"
            findings.append(f"❤️ Tachycardia: Heart rate elevated at {hr} BPM.")
            actions.append("Evaluate patient for fever, dehydration, or physical exertion.")
        elif hr < 45 and hr > 0:
            risk_level = "RED (CRITICAL EMERGENCY)"
            findings.append(f"❤️ Severe Bradycardia: Heart rate abnormally low at {hr} BPM.")
            actions.append("Assess hemodynamic stability; notify attending physician.")

    # 4. Core Body Temperature
    if temp_core >= 39.0:
        risk_level = "RED (CRITICAL EMERGENCY)"
        findings.append(f"🌡️ High Fever / Hyperpyrexia: Core temp at {temp_core:.1f} °C.")
        actions.append("Administer IV antipyretics and active cooling protocols.")
    elif temp_core >= 38.0:
        if risk_level != "RED (CRITICAL EMERGENCY)":
            risk_level = "YELLOW (WARNING)"
        findings.append(f"🌡️ Pyrexia / Fever: Core temp at {temp_core:.1f} °C.")
        actions.append("Monitor temperature curve; screen for infection/sepsis.")
    elif temp_core > 0 and temp_core < 35.0:
        if risk_level != "RED (CRITICAL EMERGENCY)":
            risk_level = "YELLOW (WARNING)"
        findings.append(f"❄️ Hypothermia: Core temp low at {temp_core:.1f} °C.")
        actions.append("Apply warm blankets; avoid cold fluid administration.")

    # 5. Fallback if all is normal
    if not findings:
        findings.append("All measured physiological parameters are within standard baseline ranges.")
        actions.append("Continue routine remote telemetry monitoring.")

    # Construct the Structured AI Decision Object
    ai_decision = {
        "timestamp": datetime.now().isoformat(),
        "uptime_seconds": uptime,
        "overall_triage": risk_level,
        "patient_metrics": {
            "heart_rate_bpm": hr if finger_on else "UNATTACHED",
            "spo2_percent": spo2 if finger_on else "UNATTACHED",
            "core_temp_c": temp_core if temp_core > 0 else "PENDING",
            "step_count": steps,
            "motion_g": round(motion_g, 2),
            "hardware_health": f"{sensors.get('active', 0)}/4 sensors online"
        },
        "clinical_findings": findings,
        "recommended_actions": actions
    }

    return ai_decision


def print_ai_decision_card(decision):
    """Prints a beautiful AI Triage Card in the terminal."""
    triage = decision["overall_triage"]
    metrics = decision["patient_metrics"]

    print("\n" + "=" * 65)
    print(f" 🤖 AI / LLM CLINICAL DECISION ENGINE — [{decision['timestamp'][:19]}]")
    print("=" * 65)
    print(f"  PATIENT TRIAGE STATUS: {triage}")
    print(f"  Hardware Health      : {metrics['hardware_health']}")
    print("-" * 65)
    print(f"  Vitals Snapshot:")
    print(f"    • Heart Rate  : {metrics['heart_rate_bpm']} BPM")
    print(f"    • SpO2 (O2)   : {metrics['spo2_percent']} %")
    print(f"    • Body Temp   : {metrics['core_temp_c']} °C")
    print(f"    • Step Count  : {metrics['step_count']} steps | Motion: {metrics['motion_g']} g")
    print("-" * 65)
    print("  📋 Clinical Findings:")
    for f in decision["clinical_findings"]:
        print(f"    - {f}")
    print("  💡 AI Recommended Actions:")
    for a in decision["recommended_actions"]:
        print(f"    ▶ {a}")
    print("=" * 65 + "\n")


def listen_serial(port="COM6", baud=115200):
    """Listens to the ESP32 serial stream and evaluates every JSON telemetry packet."""
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
                    decision = evaluate_clinical_status(packet)
                    print_ai_decision_card(decision)
                except json.JSONDecodeError as err:
                    print(f"[WARN] Failed to parse JSON: {err}")
    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")
    finally:
        ser.close()


if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else "COM6"
    listen_serial(port)
