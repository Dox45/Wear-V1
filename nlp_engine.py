"""
NLP & NVIDIA LLM Medical Summary Generator
─────────────────────────────────────────────────────────────────────────────
Generates concise, triage-ready medical summaries from patient intake data.
Powered by NVIDIA Nemotron LLM via OpenAI API client with local fallback.
"""

import os
import json
import logging
from typing import Dict, Any

logger = logging.getLogger("NLPEngine")

NVIDIA_API_KEY = os.getenv(
    "NVIDIA_API_KEY",
    "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"
)
NVIDIA_BASE_URL = os.getenv("NVIDIA_BASE_URL", "https://integrate.api.nvidia.com/v1")
NVIDIA_MODEL = os.getenv("NVIDIA_MODEL", "nvidia/nemotron-3-ultra-550b-a55b")

HAS_NVIDIA_LLM = False
openai_client = None

if NVIDIA_API_KEY:
    try:
        from openai import OpenAI
        openai_client = OpenAI(
            base_url=NVIDIA_BASE_URL,
            api_key=NVIDIA_API_KEY
        )
        HAS_NVIDIA_LLM = True
        logger.info(f"NVIDIA Nemotron LLM Engine ({NVIDIA_MODEL}) initialized.")
    except Exception as e:
        logger.warning(f"Could not initialize NVIDIA LLM: {e}")


def generate_fallback_summary(patient_data: Dict[str, Any]) -> str:
    """Generates structured, professional clinical summary locally."""
    details = patient_data.get("patient_details", {})
    name = f"{details.get('first_name', 'Patient')} {details.get('last_name', '')}".strip()
    complaint = patient_data.get("chief_complaint", "General Evaluation")
    symptoms = ", ".join(patient_data.get("symptoms", [])) or "None reported"
    duration = patient_data.get("symptom_duration", "Not specified")
    pain = patient_data.get("pain_level", 0)
    history = ", ".join(patient_data.get("medical_history", [])) or "No prior history"
    vitals = patient_data.get("vital_signs", {}) or patient_data.get("latest_vitals", {})

    temp = vitals.get("temperature") or vitals.get("temp_body_c", "N/A")
    sbp = vitals.get("blood_pressure_systolic") or vitals.get("sbp", "N/A")
    dbp = vitals.get("blood_pressure_diastolic") or vitals.get("dbp", "N/A")
    hr = vitals.get("heart_rate") or vitals.get("bpm", "N/A")
    spo2 = vitals.get("oxygen_saturation") or vitals.get("spo2", "N/A")

    return f"""### 🩺 CLINICAL SUMMARY
{name} presents with {complaint.lower()} of duration {duration}. Reported symptoms include: {symptoms}. Self-reported pain level is {pain}/10. Medical background includes: {history}.

### ⚠️ KEY CONCERNS
• Presenting Chief Complaint: {complaint} (Pain Level {pain}/10)
• Vitals Snapshot: Temp: {temp}°C | BP: {sbp}/{dbp} mmHg | HR: {hr} BPM | SpO2: {spo2}%
• Documented History: {history}

### 📋 RECOMMENDED INITIAL ACTIONS
1. Perform continuous physiological vitals monitoring and pulse transit time (PTT) check.
2. Clinical evaluation by attending triage physician.
3. Prepare point-of-care laboratory workup if symptoms persist.

### 🛡️ PRELIMINARY RISK ASSESSMENT
Risk Category: {"HIGH" if pain >= 7 or (isinstance(temp, (int, float)) and temp > 38.5) else "MODERATE" if pain >= 4 else "LOW"}
"""


def generate_medical_summary(patient_data: Dict[str, Any]) -> str:
    """Generates structured triage-ready medical summary using NVIDIA Nemotron LLM or Local Fallback."""
    if not HAS_NVIDIA_LLM or openai_client is None:
        return generate_fallback_summary(patient_data)

    try:
        details = patient_data.get("patient_details", {})
        name = f"{details.get('first_name', '')} {details.get('last_name', '')}".strip()
        vitals = patient_data.get("vital_signs", {}) or patient_data.get("latest_vitals", {})

        prompt = f"""
Generate a concise, triage-ready medical summary from the clinical data below.
Be factual, brief, and format with clear markdown headers.

=== PATIENT DETAILS ===
Name: {name}
Age/DOB: {details.get('date_of_birth', 'Unknown')}
Gender: {details.get('gender', 'Unknown')}

=== PRESENTING COMPLAINT ===
Chief Complaint: {patient_data.get('chief_complaint', 'N/A')}
Duration: {patient_data.get('symptom_duration', 'N/A')}
Pain Level: {patient_data.get('pain_level', 0)}/10
Symptoms: {', '.join(patient_data.get('symptoms', [])) or 'None'}

=== MEDICAL BACKGROUND ===
History: {', '.join(patient_data.get('medical_history', [])) or 'None'}
Medications: {', '.join(patient_data.get('current_medications', [])) or 'None'}
Allergies: {', '.join(patient_data.get('allergies', [])) or 'None'}

=== VITAL SIGNS ===
Heart Rate: {vitals.get('heart_rate') or vitals.get('bpm', 'N/A')} BPM
Blood Pressure: {vitals.get('blood_pressure_systolic', 'N/A')}/{vitals.get('blood_pressure_diastolic', 'N/A')} mmHg
Temperature: {vitals.get('temperature') or vitals.get('temp_body_c', 'N/A')}°C
SpO2: {vitals.get('oxygen_saturation') or vitals.get('spo2', 'N/A')}%

=== OUTPUT SECTIONS REQUIRED ===
### 🩺 CLINICAL SUMMARY (2-3 sentences)
### ⚠️ KEY CONCERNS (bulleted list)
### 📋 RECOMMENDED INITIAL ACTIONS
### 🛡️ PRELIMINARY RISK ASSESSMENT (Low / Moderate / High)
"""

        completion = openai_client.chat.completions.create(
            model=NVIDIA_MODEL,
            messages=[{"role": "user", "content": prompt}],
            temperature=1,
            top_p=0.95,
            max_tokens=16384,
            extra_body={"chat_template_kwargs": {"enable_thinking": True}},
            stream=True
        )

        full_content = []
        for chunk in completion:
            if not chunk.choices:
                continue
            reasoning = getattr(chunk.choices[0].delta, "reasoning_content", None)
            if reasoning:
                logger.debug(f"[NVIDIA Nemotron Reasoning]: {reasoning}")
            if chunk.choices[0].delta.content is not None:
                full_content.append(chunk.choices[0].delta.content)

        summary_text = "".join(full_content).strip()
        if summary_text:
            return summary_text
        return generate_fallback_summary(patient_data)
    except Exception as e:
        logger.error(f"NVIDIA Nemotron LLM error: {e}. Falling back to local summary.")
        return generate_fallback_summary(patient_data)
