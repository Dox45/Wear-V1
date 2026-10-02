"""
35 Benchmark Patient Seeder — BioWear Medical Triage System
─────────────────────────────────────────────────────────────────────────────
Generates and seeds 35 sampled patient records into SQLite database.
Includes realistic human vitals across all 5 ESI triage levels:
  • ESI Level 1 (CRITICAL): 7 Patients (PAT-BENCH-001 to 007)
  • ESI Level 2 (SEVERE):   7 Patients (PAT-BENCH-008 to 014)
  • ESI Level 3 (MODERATE): 7 Patients (PAT-BENCH-015 to 021)
  • ESI Level 4 (LOW):      7 Patients (PAT-BENCH-022 to 028)
  • ESI Level 5 (MINIMAL):  7 Patients (PAT-BENCH-029 to 035)

Handles device temperature sensor offset baseline (32/33°C skin sensor temp)
with a +2.5°C normalization constant to compute true core temperature.
"""

import json
import logging
import random
from datetime import datetime, timedelta, timezone
from typing import Dict, Any, List

from acuity_engine import compute_acuity

logger = logging.getLogger("SeedData")

# 35 Benchmark Patients (Exactly 7 per ESI level)
BENCHMARK_PATIENTS: List[Dict[str, Any]] = [
    # ─── ESI LEVEL 1: CRITICAL (7 PATIENTS) ─────────────────────────────────
    {
        "patient_id": "PAT-BENCH-001",
        "first_name": "Amina",
        "last_name": "Bello",
        "date_of_birth": "1968-04-12",
        "gender": "Female",
        "chief_complaint": "Acute retrosternal crushing chest pain radiating to left arm and jaw with diaphoresis",
        "pain_level": 9,
        "symptoms": ["Chest Pain", "Shortness of Breath", "Diaphoresis", "Dizziness"],
        "symptom_duration": "Acute (<45 minutes)",
        "medical_history": ["Hypertension", "Hyperlipidemia"],
        "current_medications": ["Amlodipine 10mg", "Atorvastatin 20mg"],
        "allergies": ["Penicillin"],
        "raw_temp_c": 34.6,  # Sensor raw reading (Core: 37.1°C)
        "bpm": 142,
        "spo2": 88,
        "sbp": 195,
        "dbp": 118,
        "ptt_ms": 315.0,
        "device_id": "esp32-BENCH-01"
    },
    {
        "patient_id": "PAT-BENCH-002",
        "first_name": "Emeka",
        "last_name": "Okeke",
        "date_of_birth": "1955-09-28",
        "gender": "Male",
        "chief_complaint": "Acute respiratory failure with severe hypoxic gasping and accessory muscle use",
        "pain_level": 7,
        "symptoms": ["Shortness of Breath", "Difficulty Breathing", "Cyanosis"],
        "symptom_duration": "Acute (<1 hour)",
        "medical_history": ["COPD / Asthma", "Smoking History"],
        "current_medications": ["Salbutamol Inhaler", "Prednisolone 20mg"],
        "allergies": ["None"],
        "raw_temp_c": 34.3,  # Sensor raw reading (Core: 36.8°C)
        "bpm": 138,
        "spo2": 84,
        "sbp": 168,
        "dbp": 98,
        "ptt_ms": 325.0,
        "device_id": "esp32-BENCH-02"
    },
    {
        "patient_id": "PAT-BENCH-003",
        "first_name": "Fatima",
        "last_name": "Abubakar",
        "date_of_birth": "1982-11-05",
        "gender": "Female",
        "chief_complaint": "Septic shock secondary to acute pyelonephritis, severe confusion and hypotension",
        "pain_level": 6,
        "symptoms": ["High Fever", "Altered Mental Status", "Chills", "Confusion"],
        "symptom_duration": "12 hours",
        "medical_history": ["Kidney Disease"],
        "current_medications": ["Ciprofloxacin 500mg"],
        "allergies": ["Sulfa Drugs"],
        "raw_temp_c": 38.2,  # Sensor raw reading (Core: 40.7°C - Hyperpyrexia)
        "bpm": 148,
        "spo2": 91,
        "sbp": 82,
        "dbp": 48,
        "ptt_ms": 310.0,
        "device_id": "esp32-BENCH-03"
    },
    {
        "patient_id": "PAT-BENCH-004",
        "first_name": "Babajide",
        "last_name": "Adebayo",
        "date_of_birth": "1970-03-15",
        "gender": "Male",
        "chief_complaint": "Acute ischemic stroke with sudden right-sided hemiparesis and dysarthria",
        "pain_level": 4,
        "symptoms": ["Stroke", "Altered Mental Status", "Loss of Balance"],
        "symptom_duration": "30 minutes",
        "medical_history": ["Hypertension", "Diabetes"],
        "current_medications": ["Metformin 850mg", "Lisinopril 20mg"],
        "allergies": ["Aspirin"],
        "raw_temp_c": 34.5,  # Sensor raw reading (Core: 37.0°C)
        "bpm": 126,
        "spo2": 93,
        "sbp": 210,
        "dbp": 124,
        "ptt_ms": 305.0,
        "device_id": "esp32-BENCH-04"
    },
    {
        "patient_id": "PAT-BENCH-005",
        "first_name": "Nkechi",
        "last_name": "Eze",
        "date_of_birth": "1990-07-22",
        "gender": "Female",
        "chief_complaint": "Severe anaphylactic shock following accidental allergen exposure with laryngeal edema",
        "pain_level": 8,
        "symptoms": ["Anaphylaxis", "Difficulty Breathing", "Severe Bleeding"],
        "symptom_duration": "Immediate (<20 minutes)",
        "medical_history": ["Severe Allergies"],
        "current_medications": ["EpiPen auto-injector"],
        "allergies": ["Peanuts", "Bee Venom"],
        "raw_temp_c": 34.0,  # Sensor raw reading (Core: 36.5°C)
        "bpm": 155,
        "spo2": 86,
        "sbp": 78,
        "dbp": 45,
        "ptt_ms": 295.0,
        "device_id": "esp32-BENCH-05"
    },
    {
        "patient_id": "PAT-BENCH-006",
        "first_name": "Usman",
        "last_name": "Garba",
        "date_of_birth": "1963-01-30",
        "gender": "Male",
        "chief_complaint": "Massive upper gastrointestinal hemorrhage with hematemesis and presyncope",
        "pain_level": 8,
        "symptoms": ["Severe Bleeding", "Dizziness", "Loss of Consciousness"],
        "symptom_duration": "2 hours",
        "medical_history": ["Peptic Ulcer", "Liver Disease"],
        "current_medications": ["Omeprazole 40mg"],
        "allergies": ["None"],
        "raw_temp_c": 33.8,  # Sensor raw reading (Core: 36.3°C)
        "bpm": 145,
        "spo2": 89,
        "sbp": 80,
        "dbp": 50,
        "ptt_ms": 300.0,
        "device_id": "esp32-BENCH-06"
    },
    {
        "patient_id": "PAT-BENCH-007",
        "first_name": "Chioma",
        "last_name": "Nnamdi",
        "date_of_birth": "1978-08-19",
        "gender": "Female",
        "chief_complaint": "Severe polytrauma with multiple flail chest segments following high-speed vehicle crash",
        "pain_level": 10,
        "symptoms": ["Severe Bleeding", "Chest Pain", "Difficulty Breathing"],
        "symptom_duration": "Immediate",
        "medical_history": ["None"],
        "current_medications": ["None"],
        "allergies": ["None"],
        "raw_temp_c": 33.5,  # Sensor raw reading (Core: 36.0°C)
        "bpm": 140,
        "spo2": 87,
        "sbp": 84,
        "dbp": 52,
        "ptt_ms": 308.0,
        "device_id": "esp32-BENCH-07"
    },

    # ─── ESI LEVEL 2: SEVERE (7 PATIENTS) ───────────────────────────────────
    {
        "patient_id": "PAT-BENCH-008",
        "first_name": "Tunde",
        "last_name": "Ogundipe",
        "date_of_birth": "1965-06-14",
        "gender": "Male",
        "chief_complaint": "Severe peritoneal acute abdominal pain with abdominal rigidity and bilious vomiting",
        "pain_level": 9,
        "symptoms": ["Severe Abdominal Pain", "Vomiting", "Nausea"],
        "symptom_duration": "6 hours",
        "medical_history": ["Hypertension"],
        "current_medications": ["Losartan 50mg"],
        "allergies": ["None"],
        "raw_temp_c": 35.8,  # Sensor raw reading (Core: 38.3°C)
        "bpm": 118,
        "spo2": 95,
        "sbp": 164,
        "dbp": 96,
        "ptt_ms": 375.0,
        "device_id": "esp32-BENCH-08"
    },
    {
        "patient_id": "PAT-BENCH-009",
        "first_name": "Zainab",
        "last_name": "Danjuma",
        "date_of_birth": "1993-02-18",
        "gender": "Female",
        "chief_complaint": "Severe acute asthma flare refractory to repeated inhaled bronchodilators",
        "pain_level": 8,
        "symptoms": ["Wheezing", "Cough", "Chest Tightness"],
        "symptom_duration": "3 hours",
        "medical_history": ["Asthma"],
        "current_medications": ["Fluticasone inhaler", "Salbutamol"],
        "allergies": ["Dust Mites"],
        "raw_temp_c": 34.7,  # Sensor raw reading (Core: 37.2°C)
        "bpm": 125,
        "spo2": 94,
        "sbp": 146,
        "dbp": 92,
        "ptt_ms": 360.0,
        "device_id": "esp32-BENCH-09"
    },
    {
        "patient_id": "PAT-BENCH-010",
        "first_name": "Oluwaseun",
        "last_name": "Alabi",
        "date_of_birth": "1980-12-03",
        "gender": "Male",
        "chief_complaint": "Severe hyperpyrexic fever with acute rigors, suspected complicated malaria",
        "pain_level": 7,
        "symptoms": ["Chills", "Severe Headache"],
        "symptom_duration": "24 hours",
        "medical_history": ["Hypertension"],
        "current_medications": ["Paracetamol 1000mg"],
        "allergies": ["None"],
        "raw_temp_c": 36.3,  # Sensor raw reading (Core: 38.8°C -> +8)
        "bpm": 124,
        "spo2": 95,
        "sbp": 144,
        "dbp": 88,
        "ptt_ms": 370.0,
        "device_id": "esp32-BENCH-10"
    },
    {
        "patient_id": "PAT-BENCH-011",
        "first_name": "Ifeoma",
        "last_name": "Chukwu",
        "date_of_birth": "1974-05-27",
        "gender": "Female",
        "chief_complaint": "Hypertensive urgency with severe occipital throbbing headache and blurred vision",
        "pain_level": 9,
        "symptoms": ["Blurred Vision", "Nausea"],
        "symptom_duration": "4 hours",
        "medical_history": ["Hypertension"],
        "current_medications": ["Nifedipine 20mg"],
        "allergies": ["Codeine"],
        "raw_temp_c": 34.6,  # Sensor raw reading (Core: 37.1°C)
        "bpm": 112,
        "spo2": 96,
        "sbp": 174,
        "dbp": 108,
        "ptt_ms": 350.0,
        "device_id": "esp32-BENCH-11"
    },
    {
        "patient_id": "PAT-BENCH-012",
        "first_name": "Musa",
        "last_name": "Ibrahim",
        "date_of_birth": "1958-10-10",
        "gender": "Male",
        "chief_complaint": "Uncontrolled diabetic ketoacidosis with Kussmaul respirations and dehydration",
        "pain_level": 7,
        "symptoms": ["Extreme Thirst", "Confusion", "Fatigue"],
        "symptom_duration": "2 days",
        "medical_history": ["Diabetes"],
        "current_medications": ["Insulin Glargine", "Metformin"],
        "allergies": ["None"],
        "raw_temp_c": 34.5,  # Sensor raw reading (Core: 37.0°C)
        "bpm": 125,
        "spo2": 94,
        "sbp": 152,
        "dbp": 94,
        "ptt_ms": 380.0,
        "device_id": "esp32-BENCH-12"
    },
    {
        "patient_id": "PAT-BENCH-013",
        "first_name": "Blessing",
        "last_name": "Nwosu",
        "date_of_birth": "1988-09-01",
        "gender": "Female",
        "chief_complaint": "Acute pyelonephritis with severe left costovertebral angle tenderness and high fever",
        "pain_level": 8,
        "symptoms": ["Flank Pain", "Dysuria", "Nausea"],
        "symptom_duration": "18 hours",
        "medical_history": ["Kidney Disease"],
        "current_medications": ["Nitrofurantoin 100mg"],
        "allergies": ["Ciprofloxacin"],
        "raw_temp_c": 36.4,  # Sensor raw reading (Core: 38.9°C -> +8)
        "bpm": 118,
        "spo2": 96,
        "sbp": 146,
        "dbp": 92,
        "ptt_ms": 385.0,
        "device_id": "esp32-BENCH-13"
    },
    {
        "patient_id": "PAT-BENCH-014",
        "first_name": "Kafayat",
        "last_name": "Lawal",
        "date_of_birth": "1961-03-08",
        "gender": "Female",
        "chief_complaint": "Suspected pulmonary embolism with acute unilateral calf swelling and pleuritic pain",
        "pain_level": 7,
        "symptoms": ["Leg Pain and Swelling", "Chest Discomfort"],
        "symptom_duration": "5 hours",
        "medical_history": ["Heart Disease"],
        "current_medications": ["Aspirin 81mg"],
        "allergies": ["None"],
        "raw_temp_c": 34.8,  # Sensor raw reading (Core: 37.3°C)
        "bpm": 122,
        "spo2": 94,
        "sbp": 148,
        "dbp": 92,
        "ptt_ms": 370.0,
        "device_id": "esp32-BENCH-14"
    },

    # ─── ESI LEVEL 3: MODERATE (7 PATIENTS) ─────────────────────────────────
    {
        "patient_id": "PAT-BENCH-015",
        "first_name": "Yakubu",
        "last_name": "Sani",
        "date_of_birth": "1985-04-17",
        "gender": "Male",
        "chief_complaint": "Acute renal colic flank pain radiating to ipsilateral groin with gross hematuria",
        "pain_level": 7,
        "symptoms": ["Flank Pain", "Hematuria", "Nausea"],
        "symptom_duration": "4 hours",
        "medical_history": ["Kidney Disease"],
        "current_medications": ["Tamsulosin 0.4mg"],
        "allergies": ["None"],
        "raw_temp_c": 34.4,  # Sensor raw reading (Core: 36.9°C)
        "bpm": 102,
        "spo2": 97,
        "sbp": 142,
        "dbp": 88,
        "ptt_ms": 420.0,
        "device_id": "esp32-BENCH-15"
    },
    {
        "patient_id": "PAT-BENCH-016",
        "first_name": "Funke",
        "last_name": "Adeleke",
        "date_of_birth": "1991-08-25",
        "gender": "Female",
        "chief_complaint": "Acute viral gastroenteritis with persistent vomiting and mild dehydration",
        "pain_level": 6,
        "symptoms": ["Vomiting", "Diarrhea", "Abdominal Cramps"],
        "symptom_duration": "1 day",
        "medical_history": ["Hypertension"],
        "current_medications": ["Oral Rehydration Solution"],
        "allergies": ["Metoclopramide"],
        "raw_temp_c": 35.8,  # Sensor raw reading (Core: 38.3°C -> +8)
        "bpm": 105,
        "spo2": 97,
        "sbp": 118,
        "dbp": 76,
        "ptt_ms": 440.0,
        "device_id": "esp32-BENCH-16"
    },
    {
        "patient_id": "PAT-BENCH-017",
        "first_name": "Damilola",
        "last_name": "Ojo",
        "date_of_birth": "1996-11-12",
        "gender": "Male",
        "chief_complaint": "Deep laceration on volar forearm from broken glass requiring wound exploration and repair",
        "pain_level": 8,
        "symptoms": ["Bleeding Wound", "Local Pain"],
        "symptom_duration": "1 hour",
        "medical_history": ["None"],
        "current_medications": ["None"],
        "allergies": ["Iodine"],
        "raw_temp_c": 34.2,  # Sensor raw reading (Core: 36.7°C)
        "bpm": 92,
        "spo2": 98,
        "sbp": 144,
        "dbp": 86,
        "ptt_ms": 450.0,
        "device_id": "esp32-BENCH-17"
    },
    {
        "patient_id": "PAT-BENCH-018",
        "first_name": "Grace",
        "last_name": "Umeh",
        "date_of_birth": "1979-01-09",
        "gender": "Female",
        "chief_complaint": "Moderate acute asthma symptoms with dry nocturnal cough and expiratory wheezing",
        "pain_level": 5,
        "symptoms": ["Wheezing", "Cough"],
        "symptom_duration": "2 days",
        "medical_history": ["COPD / Asthma"],
        "current_medications": ["Salbutamol Inhaler"],
        "allergies": ["None"],
        "raw_temp_c": 34.5,  # Sensor raw reading (Core: 37.0°C)
        "bpm": 98,
        "spo2": 94,
        "sbp": 132,
        "dbp": 84,
        "ptt_ms": 430.0,
        "device_id": "esp32-BENCH-18"
    },
    {
        "patient_id": "PAT-BENCH-019",
        "first_name": "Mustapha",
        "last_name": "Usman",
        "date_of_birth": "1967-07-04",
        "gender": "Male",
        "chief_complaint": "Acute bacterial bronchitis with low-grade fever and purulent sputum production",
        "pain_level": 4,
        "symptoms": ["Productive Cough", "Fatigue"],
        "symptom_duration": "3 days",
        "medical_history": ["Hypertension"],
        "current_medications": ["Amlodipine 5mg"],
        "allergies": ["None"],
        "raw_temp_c": 35.8,  # Sensor raw reading (Core: 38.3°C -> +8)
        "bpm": 96,
        "spo2": 96,
        "sbp": 142,
        "dbp": 88,
        "ptt_ms": 425.0,
        "device_id": "esp32-BENCH-19"
    },
    {
        "patient_id": "PAT-BENCH-020",
        "first_name": "Adaora",
        "last_name": "Nwachukwu",
        "date_of_birth": "1984-10-31",
        "gender": "Female",
        "chief_complaint": "Acute uncomplicated urinary tract infection with severe burning dysuria and urgency",
        "pain_level": 6,
        "symptoms": ["Dysuria", "Urinary Frequency", "Pelvic Discomfort"],
        "symptom_duration": "2 days",
        "medical_history": ["Kidney Disease"],
        "current_medications": ["Cranberry extract"],
        "allergies": ["Trimethoprim"],
        "raw_temp_c": 34.6,  # Sensor raw reading (Core: 37.1°C)
        "bpm": 90,
        "spo2": 98,
        "sbp": 142,
        "dbp": 86,
        "ptt_ms": 460.0,
        "device_id": "esp32-BENCH-20"
    },
    {
        "patient_id": "PAT-BENCH-021",
        "first_name": "Suleiman",
        "last_name": "Bello",
        "date_of_birth": "1973-12-21",
        "gender": "Male",
        "chief_complaint": "Acute migraine attack with unilateral throbbing headache, photophobia, and nausea",
        "pain_level": 7,
        "symptoms": ["Throbbing Headache", "Photophobia", "Nausea"],
        "symptom_duration": "8 hours",
        "medical_history": ["Hypertension"],
        "current_medications": ["Sumatriptan 50mg"],
        "allergies": ["None"],
        "raw_temp_c": 34.3,  # Sensor raw reading (Core: 36.8°C)
        "bpm": 92,
        "spo2": 98,
        "sbp": 142,
        "dbp": 84,
        "ptt_ms": 445.0,
        "device_id": "esp32-BENCH-21"
    },

    # ─── ESI LEVEL 4: LOW (7 PATIENTS) ──────────────────────────────────────
    {
        "patient_id": "PAT-BENCH-022",
        "first_name": "Kehinde",
        "last_name": "Popoola",
        "date_of_birth": "1998-05-19",
        "gender": "Male",
        "chief_complaint": "Right ankle inversion sprain during football match with localized edema",
        "pain_level": 6,
        "symptoms": ["Ankle Pain and Swelling", "Difficulty Bearing Weight"],
        "symptom_duration": "2 hours",
        "medical_history": ["None"],
        "current_medications": ["Ibuprofen 400mg"],
        "allergies": ["None"],
        "raw_temp_c": 34.2,  # Sensor raw reading (Core: 36.7°C)
        "bpm": 76,
        "spo2": 99,
        "sbp": 120,
        "dbp": 78,
        "ptt_ms": 470.0,
        "device_id": "esp32-BENCH-22"
    },
    {
        "patient_id": "PAT-BENCH-023",
        "first_name": "Bisi",
        "last_name": "Akande",
        "date_of_birth": "2001-09-14",
        "gender": "Female",
        "chief_complaint": "Mild acute viral pharyngitis with dry scratchy throat and mild throat discomfort",
        "pain_level": 5,
        "symptoms": ["Sore Throat", "Mild Odynophagia"],
        "symptom_duration": "1 day",
        "medical_history": ["None"],
        "current_medications": ["Throat Lozenges"],
        "allergies": ["None"],
        "raw_temp_c": 34.9,  # Sensor raw reading (Core: 37.4°C)
        "bpm": 80,
        "spo2": 99,
        "sbp": 116,
        "dbp": 74,
        "ptt_ms": 480.0,
        "device_id": "esp32-BENCH-23"
    },
    {
        "patient_id": "PAT-BENCH-024",
        "first_name": "Chinedu",
        "last_name": "Anya",
        "date_of_birth": "1994-02-08",
        "gender": "Male",
        "chief_complaint": "Superficial cut on left index finger while cooking, bleeding controlled",
        "pain_level": 6,
        "symptoms": ["Superficial Cut", "Minor Bleeding"],
        "symptom_duration": "30 minutes",
        "medical_history": ["None"],
        "current_medications": ["None"],
        "allergies": ["Latex"],
        "raw_temp_c": 34.1,  # Sensor raw reading (Core: 36.6°C)
        "bpm": 75,
        "spo2": 99,
        "sbp": 122,
        "dbp": 78,
        "ptt_ms": 475.0,
        "device_id": "esp32-BENCH-24"
    },
    {
        "patient_id": "PAT-BENCH-025",
        "first_name": "Aisha",
        "last_name": "Umar",
        "date_of_birth": "1997-07-11",
        "gender": "Female",
        "chief_complaint": "Localized erythematous skin rash on forearm suspected contact dermatitis from soap",
        "pain_level": 5,
        "symptoms": ["Skin Rash", "Pruritus"],
        "symptom_duration": "2 days",
        "medical_history": ["None"],
        "current_medications": ["Hydrocortisone cream"],
        "allergies": ["None"],
        "raw_temp_c": 34.3,  # Sensor raw reading (Core: 36.8°C)
        "bpm": 76,
        "spo2": 99,
        "sbp": 116,
        "dbp": 74,
        "ptt_ms": 485.0,
        "device_id": "esp32-BENCH-25"
    },
    {
        "patient_id": "PAT-BENCH-026",
        "first_name": "Opeyemi",
        "last_name": "Fashola",
        "date_of_birth": "1989-11-23",
        "gender": "Male",
        "chief_complaint": "Mild right ear canal pain and itching post-swimming (otitis externa)",
        "pain_level": 6,
        "symptoms": ["Ear Pain", "Pruritus"],
        "symptom_duration": "1 day",
        "medical_history": ["None"],
        "current_medications": ["Acetic acid ear drops"],
        "allergies": ["None"],
        "raw_temp_c": 34.4,  # Sensor raw reading (Core: 36.9°C)
        "bpm": 78,
        "spo2": 98,
        "sbp": 124,
        "dbp": 78,
        "ptt_ms": 465.0,
        "device_id": "esp32-BENCH-26"
    },
    {
        "patient_id": "PAT-BENCH-027",
        "first_name": "Halima",
        "last_name": "Shehu",
        "date_of_birth": "1992-04-05",
        "gender": "Female",
        "chief_complaint": "Mild bilateral band-like tension headache after long office work shift",
        "pain_level": 5,
        "symptoms": ["Dull Headache"],
        "symptom_duration": "4 hours",
        "medical_history": ["None"],
        "current_medications": ["Paracetamol 500mg"],
        "allergies": ["None"],
        "raw_temp_c": 34.2,  # Sensor raw reading (Core: 36.7°C)
        "bpm": 74,
        "spo2": 99,
        "sbp": 118,
        "dbp": 74,
        "ptt_ms": 475.0,
        "device_id": "esp32-BENCH-27"
    },
    {
        "patient_id": "PAT-BENCH-028",
        "first_name": "Emem",
        "last_name": "Bassey",
        "date_of_birth": "1987-08-30",
        "gender": "Female",
        "chief_complaint": "Mild left wrist extensor tendon strain following lifting heavy box at home",
        "pain_level": 6,
        "symptoms": ["Wrist Tenderness"],
        "symptom_duration": "1 day",
        "medical_history": ["None"],
        "current_medications": ["Diclofenac gel"],
        "allergies": ["None"],
        "raw_temp_c": 34.3,  # Sensor raw reading (Core: 36.8°C)
        "bpm": 72,
        "spo2": 99,
        "sbp": 120,
        "dbp": 76,
        "ptt_ms": 480.0,
        "device_id": "esp32-BENCH-28"
    },

    # ─── ESI LEVEL 5: MINIMAL (7 PATIENTS) ──────────────────────────────────
    {
        "patient_id": "PAT-BENCH-029",
        "first_name": "Folake",
        "last_name": "Soyinka",
        "date_of_birth": "1976-12-14",
        "gender": "Female",
        "chief_complaint": "Routine antihypertensive medication prescription refill, completely asymptomatic",
        "pain_level": 0,
        "symptoms": [],
        "symptom_duration": "N/A",
        "medical_history": ["Hypertension"],
        "current_medications": ["Lisinopril 10mg daily"],
        "allergies": ["None"],
        "raw_temp_c": 34.2,  # Sensor raw reading (Core: 36.7°C)
        "bpm": 68,
        "spo2": 99,
        "sbp": 122,
        "dbp": 78,
        "ptt_ms": 490.0,
        "device_id": "esp32-BENCH-29"
    },
    {
        "patient_id": "PAT-BENCH-030",
        "first_name": "Abubakar",
        "last_name": "Kalu",
        "date_of_birth": "1995-03-22",
        "gender": "Male",
        "chief_complaint": "Scheduled surgical suture removal from previous minor laceration repair, well healed",
        "pain_level": 0,
        "symptoms": [],
        "symptom_duration": "N/A",
        "medical_history": ["None"],
        "current_medications": ["None"],
        "allergies": ["None"],
        "raw_temp_c": 34.1,  # Sensor raw reading (Core: 36.6°C)
        "bpm": 65,
        "spo2": 100,
        "sbp": 116,
        "dbp": 74,
        "ptt_ms": 500.0,
        "device_id": "esp32-BENCH-30"
    },
    {
        "patient_id": "PAT-BENCH-031",
        "first_name": "Nafisat",
        "last_name": "Gwadabe",
        "date_of_birth": "1999-06-30",
        "gender": "Female",
        "chief_complaint": "Pre-employment medical clearance evaluation and routine baseline vitals documentation",
        "pain_level": 0,
        "symptoms": [],
        "symptom_duration": "N/A",
        "medical_history": ["None"],
        "current_medications": ["Multivitamins"],
        "allergies": ["None"],
        "raw_temp_c": 34.3,  # Sensor raw reading (Core: 36.8°C)
        "bpm": 66,
        "spo2": 99,
        "sbp": 114,
        "dbp": 72,
        "ptt_ms": 495.0,
        "device_id": "esp32-BENCH-31"
    },
    {
        "patient_id": "PAT-BENCH-032",
        "first_name": "Dayo",
        "last_name": "Towobola",
        "date_of_birth": "1981-10-18",
        "gender": "Male",
        "chief_complaint": "Post-viral illness clinical follow-up check, symptoms fully resolved",
        "pain_level": 0,
        "symptoms": [],
        "symptom_duration": "N/A",
        "medical_history": ["None"],
        "current_medications": ["None"],
        "allergies": ["None"],
        "raw_temp_c": 34.2,  # Sensor raw reading (Core: 36.7°C)
        "bpm": 70,
        "spo2": 99,
        "sbp": 120,
        "dbp": 76,
        "ptt_ms": 485.0,
        "device_id": "esp32-BENCH-32"
    },
    {
        "patient_id": "PAT-BENCH-033",
        "first_name": "Mercy",
        "last_name": "Effiong",
        "date_of_birth": "1983-01-27",
        "gender": "Female",
        "chief_complaint": "Seasonal allergic rhinitis medication advice and prescription review",
        "pain_level": 1,
        "symptoms": ["Mild Sneezing"],
        "symptom_duration": "1 week",
        "medical_history": ["Seasonal Allergies"],
        "current_medications": ["Loratadine 10mg"],
        "allergies": ["Pollen"],
        "raw_temp_c": 34.3,  # Sensor raw reading (Core: 36.8°C)
        "bpm": 68,
        "spo2": 99,
        "sbp": 118,
        "dbp": 76,
        "ptt_ms": 490.0,
        "device_id": "esp32-BENCH-33"
    },
    {
        "patient_id": "PAT-BENCH-034",
        "first_name": "Olamilekan",
        "last_name": "Balogun",
        "date_of_birth": "1990-09-09",
        "gender": "Male",
        "chief_complaint": "Request for routine drivers license blood pressure and fitness documentation form",
        "pain_level": 0,
        "symptoms": [],
        "symptom_duration": "N/A",
        "medical_history": ["None"],
        "current_medications": ["None"],
        "allergies": ["None"],
        "raw_temp_c": 34.1,  # Sensor raw reading (Core: 36.6°C)
        "bpm": 67,
        "spo2": 100,
        "sbp": 118,
        "dbp": 74,
        "ptt_ms": 495.0,
        "device_id": "esp32-BENCH-34"
    },
    {
        "patient_id": "PAT-BENCH-035",
        "first_name": "Joy",
        "last_name": "Onyekwere",
        "date_of_birth": "1997-04-15",
        "gender": "Female",
        "chief_complaint": "University athletics physical clearance form signoff and vitals screening",
        "pain_level": 0,
        "symptoms": [],
        "symptom_duration": "N/A",
        "medical_history": ["None"],
        "current_medications": ["None"],
        "allergies": ["None"],
        "raw_temp_c": 34.2,  # Sensor raw reading (Core: 36.7°C)
        "bpm": 66,
        "spo2": 99,
        "sbp": 112,
        "dbp": 72,
        "ptt_ms": 500.0,
        "device_id": "esp32-BENCH-35"
    }
]


def seed_35_patients(save_patient_fn, save_reading_fn, conn=None) -> int:
    """
    Seeds the 35 benchmark patients and their historical vital sign telemetry readings into SQLite.
    Returns the total number of patients seeded.
    """
    now = datetime.now(timezone.utc)
    seeded_count = 0

    for idx, data in enumerate(BENCHMARK_PATIENTS):
        p_id = data["patient_id"]
        raw_temp = data["raw_temp_c"]
        core_temp = round(raw_temp + 2.5, 1)

        vitals_dict = {
            "heart_rate": data["bpm"],
            "oxygen_saturation": data["spo2"],
            "temperature": raw_temp,
            "temp_body_c": raw_temp,
            "temp_core_c": core_temp,
            "temperature_f": round(raw_temp * 9.0 / 5.0 + 32.0, 1),
            "temperature_core_f": round(core_temp * 9.0 / 5.0 + 32.0, 1),
            "blood_pressure_systolic": data["sbp"],
            "blood_pressure_diastolic": data["dbp"],
            "ptt_ms": data["ptt_ms"],
            "sensor_baseline_c": 32.5,
            "normalization_offset_c": 2.5,
        }

        intake_record = {
            "patient_id": p_id,
            "device_id": data["device_id"],
            "is_simulated": False,
            "patient_details": {
                "first_name": data["first_name"],
                "last_name": data["last_name"],
                "date_of_birth": data["date_of_birth"],
                "gender": data["gender"],
            },
            "chief_complaint": data["chief_complaint"],
            "pain_level": data["pain_level"],
            "symptoms": data["symptoms"],
            "symptom_duration": data["symptom_duration"],
            "medical_history": data["medical_history"],
            "current_medications": data["current_medications"],
            "allergies": data["allergies"],
            "vital_signs": vitals_dict,
            "latest_vitals": {
                "bpm": data["bpm"],
                "spo2": data["spo2"],
                "temperature": raw_temp,
                "temp_core_c": core_temp,
                "sbp": data["sbp"],
                "dbp": data["dbp"],
                "ptt_ms": data["ptt_ms"],
            },
            "created_by_doctor": "Dr. Somtoo Okonkwo",
            "status": "TRIAGED",
            "doctor_notes": f"Initial clinical triage completed for {data['first_name']} {data['last_name']}.",
            "timestamp": (now - timedelta(minutes=random.randint(10, 120))).isoformat(),
        }

        acuity_res = compute_acuity(intake_record)
        intake_record["acuity"] = acuity_res
        intake_record["medical_summary"] = (
            f"Benchmark Triage Intake: {data['first_name']} {data['last_name']} ({data['gender']}, DOB: {data['date_of_birth']}). "
            f"Chief Complaint: {data['chief_complaint']}. Vitals: HR {data['bpm']} BPM, SpO2 {data['spo2']}%, "
            f"Raw Temp {raw_temp}°C (Core {core_temp}°C), BP {data['sbp']}/{data['dbp']} mmHg. Assigned ESI Level {acuity_res['esi_level']} ({acuity_res['severity']})."
        )

        saved = save_patient_fn(intake_record)
        if saved:
            seeded_count += 1

        num_readings = 18
        for r_i in range(num_readings):
            time_offset_sec = (num_readings - r_i) * 15
            reading_time = now - timedelta(seconds=time_offset_sec)
            
            bpm_var = data["bpm"] + random.randint(-2, 2)
            spo2_var = min(100.0, max(75.0, data["spo2"] + round(random.uniform(-0.5, 0.5), 1)))
            temp_var = round(raw_temp + random.uniform(-0.1, 0.1), 1)
            sbp_var = data["sbp"] + random.randint(-3, 3)
            dbp_var = data["dbp"] + random.randint(-2, 2)

            reading_payload = {
                "device_id": data["device_id"],
                "patient_id": p_id,
                "session_id": f"SESS-BENCH-{idx+1:03d}",
                "timestamp_ms": int(reading_time.timestamp() * 1000),
                "bpm": bpm_var,
                "spo2": spo2_var,
                "temp_body_c": temp_var,
                "temp_body_f": round(temp_var * 9.0 / 5.0 + 32.0, 1),
                "temp_die_c": 32.5,
                "temp_die_f": 90.5,
                "sbp": sbp_var,
                "dbp": dbp_var,
                "ptt_ms": data["ptt_ms"],
                "latency_ms": round(random.uniform(45.0, 85.0), 1),
                "finger_detected": True,
                "is_simulated": False,
            }
            save_reading_fn(reading_payload)

    logger.info(f"Successfully seeded {seeded_count} benchmark patients and historical telemetry into SQLite!")
    return seeded_count
