"""
Intelligent Medical Triage Acuity Engine (ESI Index + Safety Overrides)
─────────────────────────────────────────────────────────────────────────────
Computes dynamic patient acuity score (1–5 Emergency Severity Index scale)
combining vital sign telemetry, chief complaints, pain index, and LLM summary context.
Features Deterministic Safety Overrides to eliminate False Negatives.
"""

import math
import logging
from typing import Dict, Any, List, Tuple

logger = logging.getLogger("AcuityEngine")

# Emergency Severity Index (ESI) Levels
ESI_LEVELS = {
    1: {"name": "CRITICAL", "color": "#ef4444", "action": "Immediate Resuscitation & Physician Attention"},
    2: {"name": "SEVERE",   "color": "#f97316", "action": "Immediate Urgent Placement & High Priority Queue"},
    3: {"name": "MODERATE", "color": "#eab308", "action": "Standard Multi-Resource Clinical Workup"},
    4: {"name": "LOW",      "color": "#3b82f6", "action": "Single Resource / Non-Urgent Priority"},
    5: {"name": "MINIMAL",  "color": "#10b981", "action": "Fast-Track / Deferred Follow-Up"},
}

CRITICAL_SYMPTOMS = [
    "chest pain", "difficulty breathing", "shortness of breath", "severe bleeding",
    "unconscious", "loss of consciousness", "stroke", "seizure", "altered mental status",
    "anaphylaxis", "severe burn", "cardiac arrest"
]

HIGH_RISK_CONDITIONS = [
    "heart disease", "hypertension", "diabetes", "copd", "asthma", "cancer",
    "immunocompromised", "kidney disease", "stroke history", "organ transplant"
]


def check_safety_overrides(vitals: Dict[str, Any], symptoms: List[str]) -> Tuple[bool, int, str]:
    """
    Evaluates physiological vitals against critical clinical emergency thresholds.
    Returns (tripped, force_esi_level, reason) to guarantee urgent escalation.
    """
    temp_c = vitals.get("temperature") or vitals.get("temp_body_c")
    sbp = vitals.get("sbp") or vitals.get("blood_pressure_systolic")
    dbp = vitals.get("dbp") or vitals.get("blood_pressure_diastolic")
    spo2 = vitals.get("spo2") or vitals.get("oxygen_saturation")
    hr = vitals.get("bpm") or vitals.get("heart_rate")

    # Critical SpO2 < 90%
    if spo2 is not None and spo2 < 90:
        return True, 1, f"Severe Hypoxia (SpO2 {spo2}%)"

    # Extreme Body Temperature (Fever > 40.5°C / 105°F or Severe Hypothermia < 35°C / 95°F)
    if temp_c is not None:
        if temp_c >= 40.5 or temp_c <= 35.0:
            return True, 1, f"Critical Body Temperature ({temp_c:.1f}°C)"

    # Hypertensive Crisis / Severe Hypotension
    if sbp is not None:
        if sbp >= 180 or sbp <= 85:
            return True, 1, f"Critical Systolic Blood Pressure ({sbp} mmHg)"
    if dbp is not None:
        if dbp >= 120 or dbp <= 50:
            return True, 1, f"Critical Diastolic Blood Pressure ({dbp} mmHg)"

    # Extreme Heart Rate
    if hr is not None:
        if hr >= 140 or hr <= 40:
            return True, 1, f"Critical Heart Rate ({hr} BPM)"

    # High-Risk Critical Symptoms
    if symptoms:
        for s in symptoms:
            if any(cs in s.lower() for cs in CRITICAL_SYMPTOMS):
                return True, 1, f"High-Risk Critical Symptom: '{s}'"

    return False, 5, ""


def compute_acuity(patient_record: Dict[str, Any]) -> Dict[str, Any]:
    """
    Computes patient acuity score (0-100 raw score -> ESI 1-5 level).
    """
    vitals = patient_record.get("latest_vitals", {}) or patient_record.get("intake_vitals", {})
    symptoms = patient_record.get("symptoms", [])
    history = patient_record.get("medical_history", [])
    pain_level = patient_record.get("pain_level", 0) or 0

    # 1. Evaluate Deterministic Safety Overrides
    tripped, forced_esi, override_reason = check_safety_overrides(vitals, symptoms)

    score = 0
    contributing_factors = []

    if tripped:
        contributing_factors.append(f"SAFETY OVERRIDE: {override_reason}")

    # Pain Score (0 to 30 pts)
    pain_pts = min(30, pain_level * 3)
    score += pain_pts
    if pain_level >= 8:
        contributing_factors.append(f"Severe Pain ({pain_level}/10)")
    elif pain_level >= 5:
        contributing_factors.append(f"Moderate Pain ({pain_level}/10)")

    # Vitals Points (0 to 40 pts)
    temp_c = vitals.get("temperature") or vitals.get("temp_body_c")
    if temp_c:
        if temp_c > 39.5 or temp_c < 35.5:
            score += 15
            contributing_factors.append(f"Abnormal Temperature ({temp_c:.1f}°C)")
        elif temp_c > 38.0:
            score += 8
            contributing_factors.append(f"Elevated Temperature ({temp_c:.1f}°C)")

    sbp = vitals.get("sbp") or vitals.get("blood_pressure_systolic")
    dbp = vitals.get("dbp") or vitals.get("blood_pressure_diastolic")
    if sbp or dbp:
        if (sbp and (sbp > 160 or sbp < 95)) or (dbp and (dbp > 105 or dbp < 60)):
            score += 15
            contributing_factors.append(f"Abnormal BP ({sbp or '?'}/{dbp or '?'} mmHg)")
        elif (sbp and sbp > 140) or (dbp and dbp > 90):
            score += 8
            contributing_factors.append(f"Stage 1 Hypertension ({sbp or '?'}/{dbp or '?'})")

    hr = vitals.get("bpm") or vitals.get("heart_rate")
    if hr:
        if hr > 120 or hr < 50:
            score += 10
            contributing_factors.append(f"Abnormal Heart Rate ({hr} BPM)")

    spo2 = vitals.get("spo2") or vitals.get("oxygen_saturation")
    if spo2:
        if spo2 < 92:
            score += 15
            contributing_factors.append(f"Low Oxygen Saturation ({spo2}%)")
        elif spo2 < 95:
            score += 8
            contributing_factors.append(f"Reduced Oxygen Saturation ({spo2}%)")

    # Symptom points (0 to 20 pts)
    if symptoms:
        for s in symptoms:
            if any(cs in s.lower() for cs in CRITICAL_SYMPTOMS):
                score += 20
                contributing_factors.append(f"Critical symptom: {s}")
                break

    # History points (0 to 10 pts)
    if history:
        for cond in history:
            if any(hr_c in cond.lower() for hr_c in HIGH_RISK_CONDITIONS):
                score += 10
                contributing_factors.append(f"High-risk background: {cond}")
                break

    # Determine final ESI Level
    if tripped:
        esi_level = forced_esi
    else:
        if score >= 70:
            esi_level = 1
        elif score >= 50:
            esi_level = 2
        elif score >= 30:
            esi_level = 3
        elif score >= 15:
            esi_level = 4
        else:
            esi_level = 5

    esi_info = ESI_LEVELS[esi_level]

    return {
        "esi_level": esi_level,
        "severity": esi_info["name"],
        "color": esi_info["color"],
        "action": esi_info["action"],
        "raw_score": score,
        "requires_immediate_attention": esi_level <= 2,
        "contributing_factors": contributing_factors,
    }
