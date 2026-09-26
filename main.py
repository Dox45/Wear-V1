"""
HealthMonitor Cloud Backend API & Triage Controller — v3.0
─────────────────────────────────────────────────────────────────────────────
Full-stack REST & Streaming API powering the Medical Triage System.
Features:
  • Real-time SSE event stream (/readings/stream) for UI waveform & vitals sync
  • IoT sensor ingestion endpoint (/readings) with PTT-based BP estimation
  • Patient intake API (/api/patient-intake) with NVIDIA Nemotron LLM summarization
  • Intelligent Acuity Scoring & ESI Triage Queue (/api/triage/queue)
  • Calibration endpoint (/calibrate) for per-device PTT blood pressure constants
  • Integration with Apache Kafka streaming & resilient fallback
"""

import asyncio
import json
import os
import uuid
from collections import deque
from datetime import datetime, timezone
from typing import Any, Deque, Dict, List, Optional

from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse, JSONResponse, StreamingResponse, FileResponse
from pydantic import BaseModel, Field

from acuity_engine import compute_acuity
from kafka_config import get_producer, init_kafka_topics
from nlp_engine import generate_medical_summary
from database import (
    init_db,
    create_doctor,
    authenticate_doctor,
    get_doctor_by_token,
    delete_session,
    save_patient,
    get_all_patients,
    get_patient_by_id,
    update_patient_status,
    delete_patient,
    save_reading,
    create_monitoring_session,
    get_active_session_by_device,
    get_active_session_by_patient,
    close_monitoring_session_by_patient,
    close_monitoring_session_by_device,
    get_all_active_sessions,
    get_patient_readings,
    get_latest_patient_reading,
    get_patient_profile_data,
    clear_simulated_data,
    clear_all_demo_patients,
)

# Initialize SQLite database on startup and purge old demo patients
init_db()
clear_all_demo_patients()

# Initialize Kafka topics in background
init_kafka_topics()

# Kafka Producer
kafka_producer, send_kafka_message = get_producer()

# ─── Configuration & Defaults ────────────────────────────────────────────────
REALTIME_SIZE = 120   # ~2 minutes at 1 Hz
TEN_MIN_SIZE  = 600   # ~10 minutes at 1 Hz

DEFAULT_CAL = {
    "sbp_slope":     -37.0,
    "sbp_intercept":  198.0,
    "dbp_slope":     -21.0,
    "dbp_intercept":  130.0,
}

PTT_MIN_MS = 300
PTT_MAX_MS = 1500

# ─── App Setup ───────────────────────────────────────────────────────────────
app = FastAPI(title="BioWear Cloud Triage System", version="3.0.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# ─── In-Memory Data Stores ───────────────────────────────────────────────────
realtime_history: Deque[dict] = deque(maxlen=REALTIME_SIZE)
tenmin_history:   Deque[dict] = deque(maxlen=TEN_MIN_SIZE)
latest_reading:   Optional[dict] = None
calibrations:     Dict[str, dict] = {}
patient_store:    Dict[str, dict] = {}  # patient_id -> patient record
sse_subscribers:  List[asyncio.Queue] = []
touch_onset_times: Dict[str, float] = {} # device_id -> onset timestamp in seconds


# ─── Blood Pressure PTT Estimator ───────────────────────────────────────────
def estimate_bp(ptt_ms: float, cal: dict) -> tuple:
    if not (PTT_MIN_MS <= ptt_ms <= PTT_MAX_MS):
        return None, None

    inv_ptt = 1.0 / (ptt_ms / 1000.0)
    sbp = cal["sbp_slope"] * inv_ptt + cal["sbp_intercept"]
    dbp = cal["dbp_slope"] * inv_ptt + cal["dbp_intercept"]

    sbp = max(60.0, min(220.0, sbp))
    dbp = max(40.0, min(140.0, dbp))

    if sbp <= dbp:
        return None, None

    return round(sbp, 1), round(dbp, 1)


# ─── Pydantic Data Models ───────────────────────────────────────────────────
class UserRegisterRequest(BaseModel):
    full_name: str = Field(..., min_length=1)
    device_id: Optional[str] = Field("esp32-01", description="Assigned device ID")


class Reading(BaseModel):
    device_id:        str
    timestamp_ms:     int
    bpm:              int   = Field(..., ge=0)
    bpm_valid:        bool  = True
    spo2:             float = Field(..., ge=0, le=100)
    spo2_valid:       bool  = True
    temp_body_c:      float
    temp_body_f:      Optional[float] = None
    temp_die_c:       Optional[float] = None
    temp_die_f:       Optional[float] = None
    finger_detected:  bool  = True
    device_connected: Optional[bool]  = True
    is_simulated:     Optional[bool]  = False
    ir_raw:           Optional[int]   = None
    ptt_ms:           Optional[float] = Field(
        None,
        description="Pulse Transit Time in ms from MAX30102 waveform peaks."
    )


class PatientDetails(BaseModel):
    first_name:        str
    last_name:         str
    date_of_birth:     Optional[str] = "Unknown"
    gender:            Optional[str] = "Unknown"
    contact_number:    Optional[str] = ""
    emergency_contact: Optional[str] = ""


class VitalSignsIntake(BaseModel):
    heart_rate:               Optional[int]   = None
    blood_pressure_systolic:  Optional[int]   = None
    blood_pressure_diastolic: Optional[int]   = None
    temperature:              Optional[float] = None
    oxygen_saturation:        Optional[int]   = None


class PatientIntakeForm(BaseModel):
    patient_id:          Optional[str] = None
    device_id:           Optional[str] = Field(None, description="ESP32 hardware device ID to assign (e.g. esp32-01)")
    patient_details:     PatientDetails
    chief_complaint:     str
    symptoms:            List[str]     = []
    symptom_duration:    Optional[str] = "Recent"
    pain_level:          int           = Field(0, ge=0, le=10)
    medical_history:     List[str]     = []
    current_medications: List[str]     = []
    allergies:           List[str]     = []
    vital_signs:         Optional[VitalSignsIntake] = None


class SessionStartRequest(BaseModel):
    patient_id: str
    device_id:  str


class SessionStopRequest(BaseModel):
    patient_id: Optional[str] = None
    device_id:  Optional[str] = None



class CalibrationRequest(BaseModel):
    device_id:     str
    sbp_slope:     float = Field(DEFAULT_CAL["sbp_slope"])
    sbp_intercept: float = Field(DEFAULT_CAL["sbp_intercept"])
    dbp_slope:     float = Field(DEFAULT_CAL["dbp_slope"])
    dbp_intercept: float = Field(DEFAULT_CAL["dbp_intercept"])
    ref_sbp:       Optional[float] = None
    ref_dbp:       Optional[float] = None
    ref_ptt_ms:    Optional[float] = None


class DoctorSignUp(BaseModel):
    username:       str = Field(..., min_length=3)
    password:       str = Field(..., min_length=6)
    full_name:      str = Field(...)
    license_number: str = Field(...)
    department:     str = Field("Emergency Triage")


class DoctorLogin(BaseModel):
    username: str = Field(...)
    password: str = Field(...)


class PatientStatusUpdate(BaseModel):
    status:       str = Field(..., description="TRIAGED, UNDER_CARE, DISCHARGED")
    doctor_notes: Optional[str] = None


# ─── Helper to Broadcast SSE Events & Validate Doctor Token ──────────────────
def get_current_doctor(request: Request) -> Dict[str, Any]:
    """Helper to extract and validate Doctor Bearer token."""
    auth_header = request.headers.get("Authorization", "")
    token = auth_header.replace("Bearer ", "").strip() if auth_header.startswith("Bearer ") else request.query_params.get("token", "")
    if not token:
        raise HTTPException(401, "Doctor authentication required. Please log in to the Doctor Portal.")
    doc = get_doctor_by_token(token)
    if not doc:
        raise HTTPException(401, "Invalid or expired doctor session. Please log in again.")
    return doc


async def broadcast_sse(event_data: dict):
    """Sends event payload to all active web SSE clients."""
    json_str = json.dumps(event_data)
    for q in sse_subscribers[:]:
        try:
            q.put_nowait(json_str)
        except asyncio.QueueFull:
            sse_subscribers.remove(q)


# ─── Endpoints ──────────────────────────────────────────────────────────────

@app.get("/", response_class=HTMLResponse)
async def serve_dashboard():
    """Serves index.html UI."""
    dashboard_path = os.path.join(os.path.dirname(__file__), "index.html")
    if os.path.exists(dashboard_path):
        with open(dashboard_path, "r", encoding="utf-8") as f:
            return HTMLResponse(content=f.read())
    return HTMLResponse("<h2>BioWear Dashboard File Not Found</h2>", status_code=404)


@app.get("/dashboard.css")
async def serve_css():
    """Serves dashboard.css stylesheet."""
    css_path = os.path.join(os.path.dirname(__file__), "dashboard.css")
    if os.path.exists(css_path):
        return FileResponse(css_path, media_type="text/css")
    raise HTTPException(404, "dashboard.css not found")


@app.get("/dashboard.js")
async def serve_js():
    """Serves dashboard.js application logic."""
    js_path = os.path.join(os.path.dirname(__file__), "dashboard.js")
    if os.path.exists(js_path):
        return FileResponse(js_path, media_type="application/javascript")
    raise HTTPException(404, "dashboard.js not found")


# ─── Doctor Auth Endpoints ────────────────────────────────────────────────────

@app.post("/api/auth/signup")
async def doctor_signup(req: DoctorSignUp):
    """Registers a new doctor account."""
    doc = create_doctor(req.username, req.password, req.full_name, req.license_number, req.department)
    if not doc:
        raise HTTPException(400, f"Doctor username '{req.username}' already exists.")
    return {"status": "success", "message": "Doctor registered successfully", "doctor": doc}


@app.post("/api/auth/login")
async def doctor_login(req: DoctorLogin):
    """Authenticates doctor and returns session token."""
    res = authenticate_doctor(req.username, req.password)
    if not res:
        raise HTTPException(401, "Invalid doctor username or password.")
    return {"status": "success", **res}


@app.get("/api/auth/me")
async def doctor_me(request: Request):
    """Returns currently authenticated doctor profile."""
    auth_header = request.headers.get("Authorization", "")
    token = auth_header.replace("Bearer ", "").strip() if auth_header.startswith("Bearer ") else request.query_params.get("token", "")
    if not token:
        raise HTTPException(401, "Missing authentication token.")
    doc = get_doctor_by_token(token)
    if not doc:
        raise HTTPException(401, "Invalid or expired doctor session token.")
    return {"status": "success", "doctor": doc}


@app.post("/api/auth/logout")
async def doctor_logout(request: Request):
    """Logs out doctor and invalidates session token."""
    auth_header = request.headers.get("Authorization", "")
    token = auth_header.replace("Bearer ", "").strip() if auth_header.startswith("Bearer ") else request.query_params.get("token", "")
    if token:
        delete_session(token)
    return {"status": "success", "message": "Logged out successfully"}


@app.post("/readings", status_code=201)
async def ingest_reading(reading: Reading):
    """IoT Telemetry Ingestion Endpoint called by ESP32/ESP8266."""
    global latest_reading
    import time

    dev_id = reading.device_id
    now_sec = time.time()
    latency_ms = None

    if reading.finger_detected:
        if dev_id not in touch_onset_times:
            touch_onset_times[dev_id] = now_sec
        latency_ms = round((now_sec - touch_onset_times[dev_id]) * 1000.0, 1)
    else:
        touch_onset_times.pop(dev_id, None)

    cal = calibrations.get(reading.device_id, DEFAULT_CAL)
    sbp, dbp = (None, None)
    bp_valid = False

    if reading.ptt_ms is not None:
        sbp, dbp = estimate_bp(reading.ptt_ms, cal)
        bp_valid = sbp is not None

    record = reading.model_dump()

    # Look up active monitoring session linking device_id to a registered patient
    active_session = get_active_session_by_device(reading.device_id)
    patient_id = active_session["patient_id"] if active_session else None
    session_id = active_session["session_id"] if active_session else None

    temp_die_f = reading.temp_die_f
    if reading.temp_die_c is not None and temp_die_f is None:
        temp_die_f = round(reading.temp_die_c * 9.0 / 5.0 + 32.0, 2)

    record.update({
        "server_time":     datetime.now(timezone.utc).isoformat(),
        "temp_body_f":     round(reading.temp_body_c * 9.0 / 5.0 + 32.0, 2) if reading.temp_body_c else 0.0,
        "temp_die_f":      temp_die_f,
        "sbp":             sbp,
        "dbp":             dbp,
        "bp_valid":        bp_valid,
        "map":             round((sbp + 2 * dbp) / 3, 1) if bp_valid else None,
        "pulse_pressure":  round(sbp - dbp, 1) if bp_valid else None,
        "device_connected": True,
        "patient_id":      patient_id,
        "session_id":      session_id,
        "latency_ms":      latency_ms,
    })

    realtime_history.append(record)
    tenmin_history.append(record)
    latest_reading = record

    # Persist to SQLite readings log (with patient_id & session_id)
    save_reading(record)

    # Produce to Kafka topic using patient_id key if bound, else hardware device_id
    kafka_key = patient_id or reading.device_id
    send_kafka_message("vital-signs", key=kafka_key, value=record)

    # Broadcast to SSE clients asynchronously
    asyncio.create_task(broadcast_sse({"type": "telemetry", "data": record}))

    return {"status": "ok", "sbp": sbp, "dbp": dbp, "bp_valid": bp_valid, "patient_id": patient_id, "session_id": session_id, "latency_ms": latency_ms}


@app.get("/readings/stream")
async def stream_readings(request: Request):
    """Server-Sent Events (SSE) streaming endpoint for live UI updates."""
    async def event_generator():
        q: asyncio.Queue = asyncio.Queue(maxsize=50)
        sse_subscribers.append(q)
        try:
            # Send initial state immediately
            if latest_reading:
                yield f"data: {json.dumps({'type': 'telemetry', 'data': latest_reading})}\n\n"

            while True:
                if await request.is_disconnected():
                    break
                try:
                    data = await asyncio.wait_for(q.get(), timeout=1.5)
                    yield f"data: {data}\n\n"
                except asyncio.TimeoutError:
                    # Heartbeat
                    hb = {"type": "heartbeat", "time": datetime.now(timezone.utc).isoformat()}
                    yield f"data: {json.dumps(hb)}\n\n"
        finally:
            if q in sse_subscribers:
                sse_subscribers.remove(q)

    return StreamingResponse(event_generator(), media_type="text/event-stream")


@app.post("/api/patient-intake")
async def submit_patient_intake(form: PatientIntakeForm, request: Request):
    """Submits patient intake form, generates LLM summary, computes Acuity score (Doctor Auth Required)."""
    doctor = get_current_doctor(request)
    p_id = form.patient_id or f"PAT-{uuid.uuid4().hex[:6].upper()}"
    timestamp = datetime.now(timezone.utc).isoformat()

    intake_data = form.model_dump()
    intake_data["patient_id"] = p_id
    intake_data["timestamp"] = timestamp
    intake_data["created_by_doctor"] = doctor["full_name"]

    # Bind patient to assigned ESP32 device via active Monitoring Session if provided
    if form.device_id:
        session = create_monitoring_session(p_id, form.device_id)
        intake_data["device_id"] = form.device_id

        # Retrieve latest vitals ONLY for this patient/device from SQLite reading history
        patient_reading = get_latest_patient_reading(p_id)
        if patient_reading:
            intake_data["latest_vitals"] = {
                "bpm": patient_reading.get("bpm"),
                "spo2": patient_reading.get("spo2"),
                "temperature": patient_reading.get("temp_body_c"),
                "sbp": patient_reading.get("sbp"),
                "dbp": patient_reading.get("dbp"),
                "ptt_ms": patient_reading.get("ptt_ms"),
            }
    elif form.vital_signs:
        v_dict = form.vital_signs.model_dump()
        intake_data["latest_vitals"] = {
            "bpm": v_dict.get("heart_rate"),
            "spo2": v_dict.get("oxygen_saturation"),
            "temperature": v_dict.get("temperature"),
            "sbp": v_dict.get("blood_pressure_systolic"),
            "dbp": v_dict.get("blood_pressure_diastolic"),
            "ptt_ms": None,
        }

    # Generate LLM medical summary
    medical_summary = generate_medical_summary(intake_data)
    intake_data["medical_summary"] = medical_summary

    # Compute Acuity & ESI Level
    acuity_result = compute_acuity(intake_data)
    intake_data["acuity"] = acuity_result

    # Save to SQLite Database for persistent retention
    save_patient(intake_data)
    patient_store[p_id] = intake_data

    # Produce to Kafka topics
    send_kafka_message("patient-intake", key=p_id, value=intake_data)
    send_kafka_message("nlp-summary", key=p_id, value={"patient_id": p_id, "summary": medical_summary})
    send_kafka_message("acuity-score", key=p_id, value={"patient_id": p_id, **acuity_result})

    if acuity_result.get("requires_immediate_attention"):
        send_kafka_message("alerts", key=p_id, value={
            "patient_id": p_id,
            "patient_name": f"{form.patient_details.first_name} {form.patient_details.last_name}",
            "esi_level": acuity_result.get("esi_level"),
            "severity": acuity_result.get("severity"),
            "contributing_factors": acuity_result.get("contributing_factors"),
        })

    # Broadcast updated triage queue to UI SSE clients
    asyncio.create_task(broadcast_sse({
        "type": "triage_update",
        "patient_id": p_id,
        "acuity": acuity_result,
    }))

    return {
        "status": "success",
        "patient_id": p_id,
        "acuity": acuity_result,
        "medical_summary": medical_summary,
    }


# ─── Monitoring Session Endpoints ───────────────────────────────────────────

@app.post("/api/sessions/start")
async def start_monitoring_session(req: SessionStartRequest, request: Request):
    """Assigns an IoT device to a patient and opens an active monitoring session (Doctor Auth Required)."""
    get_current_doctor(request)
    patient = get_patient_by_id(req.patient_id)
    if not patient:
        raise HTTPException(404, f"Patient {req.patient_id} not found.")

    session = create_monitoring_session(req.patient_id, req.device_id)

    asyncio.create_task(broadcast_sse({
        "type": "triage_update",
        "patient_id": req.patient_id,
        "action": "session_started",
        "device_id": req.device_id
    }))

    return {"status": "success", "session": session}


@app.post("/api/sessions/stop")
async def stop_monitoring_session(req: SessionStopRequest, request: Request):
    """Closes active monitoring session for a patient or device (Doctor Auth Required)."""
    get_current_doctor(request)
    closed = False
    if req.patient_id:
        closed = close_monitoring_session_by_patient(req.patient_id)
    elif req.device_id:
        closed = close_monitoring_session_by_device(req.device_id)
    else:
        raise HTTPException(400, "Must provide patient_id or device_id to stop session.")

    asyncio.create_task(broadcast_sse({
        "type": "triage_update",
        "action": "session_stopped"
    }))

    return {"status": "success", "closed": closed}


@app.get("/api/sessions/active")
async def get_active_sessions():
    """Lists all active monitoring sessions."""
    sessions = get_all_active_sessions()
    return {"status": "success", "count": len(sessions), "sessions": sessions}


@app.get("/api/triage/patient/{patient_id}/readings")
async def get_patient_vital_readings(patient_id: str, limit: int = 100):
    """Retrieves vital sign reading history for a specific patient."""
    readings = get_patient_readings(patient_id, limit=limit)
    return {"status": "success", "patient_id": patient_id, "count": len(readings), "readings": readings}


@app.post("/api/users/register-touch")
async def register_user_touch(req: UserRegisterRequest):
    """
    Registers a user dynamically upon touching the sensor.
    Binds the user to an active monitoring session on the hardware device.
    """
    p_id = f"PAT-{uuid.uuid4().hex[:6].upper()}"
    full_name_clean = req.full_name.strip()
    names = full_name_clean.split(" ", 1)
    first_name = names[0]
    last_name = names[1] if len(names) > 1 else ""

    now_iso = datetime.now(timezone.utc).isoformat()
    patient_record = {
        "patient_id": p_id,
        "patient_details": {
            "first_name": first_name,
            "last_name": last_name,
            "date_of_birth": "Unknown",
            "gender": "Unknown"
        },
        "chief_complaint": "Touch Sensor Vitals Tracking",
        "pain_level": 0,
        "symptoms": [],
        "symptom_duration": "Recent",
        "medical_history": [],
        "current_medications": [],
        "allergies": [],
        "vital_signs": {},
        "acuity": {
            "esi_level": 3,
            "severity": "MODERATE",
            "raw_score": 50,
            "action": "Live Vitals Profiling",
            "contributing_factors": []
        },
        "medical_summary": f"User profile created via touch sensor for {full_name_clean}.",
        "status": "TRIAGED",
        "doctor_notes": "",
        "created_by_doctor": "TOUCH_REGISTER",
        "device_id": req.device_id or "esp32-01",
        "is_simulated": False,
        "timestamp": now_iso
    }

    save_patient(patient_record)
    patient_store[p_id] = patient_record
    session = create_monitoring_session(p_id, req.device_id or "esp32-01")

    asyncio.create_task(broadcast_sse({
        "type": "triage_update",
        "patient_id": p_id,
        "action": "user_registered",
        "device_id": req.device_id or "esp32-01"
    }))

    return {"status": "success", "patient_id": p_id, "session": session, "patient": patient_record}


@app.get("/api/users/{patient_id}/profile")
@app.get("/api/triage/patient/{patient_id}/profile")
async def get_user_profile(patient_id: str):
    """Retrieves temperature, blood pressure, and latency historical profiling data for a user."""
    data = get_patient_profile_data(patient_id)
    if data.get("status") == "error":
        raise HTTPException(404, data.get("message", "User profile not found"))
    return data



@app.get("/api/triage/queue")
async def get_triage_queue():
    """Returns all triaged patients from SQLite database sorted by Acuity (ESI 1 to 5)."""
    queue = get_all_patients()
    return {"status": "success", "count": len(queue), "queue": queue}


@app.get("/api/triage/patient/{patient_id}")
async def get_patient_detail(patient_id: str):
    """Returns single patient detailed record from SQLite database."""
    patient = get_patient_by_id(patient_id)
    if not patient:
        patient = patient_store.get(patient_id)
    if not patient:
        raise HTTPException(404, f"Patient {patient_id} not found.")
    return patient


@app.put("/api/triage/patient/{patient_id}/status")
async def update_triage_status(patient_id: str, req: PatientStatusUpdate, request: Request):
    """Updates patient triage status and doctor clinical notes in SQLite database (Doctor Auth Required)."""
    doctor = get_current_doctor(request)
    valid_statuses = {"TRIAGED", "UNDER_CARE", "DISCHARGED"}
    if req.status.upper() not in valid_statuses:
        raise HTTPException(400, f"Invalid status '{req.status}'. Must be one of {valid_statuses}")

    notes_with_doc = f"[{doctor['full_name']}]: {req.doctor_notes}" if req.doctor_notes else None
    updated = update_patient_status(patient_id, req.status.upper(), notes_with_doc)
    if not updated:
        raise HTTPException(404, f"Patient {patient_id} not found.")

    asyncio.create_task(broadcast_sse({
        "type": "triage_update",
        "patient_id": patient_id,
        "status": req.status.upper()
    }))

    return {"status": "success", "patient": updated}


@app.delete("/api/triage/patient/{patient_id}")
async def discharge_patient(patient_id: str, request: Request):
    """Discharges/deletes a patient record from SQLite database (Doctor Auth Required)."""
    get_current_doctor(request)
    deleted = delete_patient(patient_id)
    if patient_id in patient_store:
        del patient_store[patient_id]
    if not deleted:
        raise HTTPException(404, f"Patient {patient_id} not found.")

    asyncio.create_task(broadcast_sse({
        "type": "triage_update",
        "patient_id": patient_id,
        "action": "deleted"
    }))

    return {"status": "success", "message": f"Patient {patient_id} discharged successfully."}


@app.post("/calibrate")
async def calibrate(req: CalibrationRequest):
    """Updates PTT blood pressure calibration constants for a device."""
    cal = {
        "sbp_slope":     req.sbp_slope,
        "sbp_intercept": req.sbp_intercept,
        "dbp_slope":     req.dbp_slope,
        "dbp_intercept": req.dbp_intercept,
    }

    if req.ref_sbp and req.ref_dbp and req.ref_ptt_ms:
        if not (PTT_MIN_MS <= req.ref_ptt_ms <= PTT_MAX_MS):
            raise HTTPException(400, f"ref_ptt_ms must be {PTT_MIN_MS}–{PTT_MAX_MS} ms")
        inv_ptt = 1.0 / (req.ref_ptt_ms / 1000.0)
        cal["sbp_intercept"] = req.ref_sbp - req.sbp_slope * inv_ptt
        cal["dbp_intercept"] = req.ref_dbp - req.dbp_slope * inv_ptt

    calibrations[req.device_id] = cal
    sbp_check, dbp_check = estimate_bp(req.ref_ptt_ms or 300, cal)

    return {
        "status":        "calibration saved",
        "device_id":     req.device_id,
        "calibration":   cal,
        "sample_output": {"ptt_ms": req.ref_ptt_ms or 300, "sbp": sbp_check, "dbp": dbp_check},
    }


@app.get("/readings/latest")
async def get_latest():
    if latest_reading is None:
        return JSONResponse({
            "device_id": "esp32-01",
            "device_connected": False,
            "finger_detected": False,
            "bpm": 0,
            "spo2": 0,
            "temp_body_c": 0.0,
            "ptt_ms": None,
            "sbp": None,
            "dbp": None
        })
    return latest_reading


@app.get("/readings/realtime")
async def get_realtime():
    return list(realtime_history)


@app.get("/readings/ten_min")
async def get_ten_min():
    return list(tenmin_history)


@app.get("/api/health")
async def health_check():
    return {
        "status": "healthy",
        "service": "BioWear Medical Triage Cloud API",
        "sse_subscribers": len(sse_subscribers),
        "total_triaged_patients": len(patient_store),
        "simulation_state": sim_state,
    }


# ─── Simulation System & Triage Controller ────────────────────────────────────

sim_state = {
    "mode": "device_only",  # device_only, simulation_only, device_with_simulation
    "running": False,
    "allow_severe": True,
}
sim_task: Optional[asyncio.Task] = None


class SimModeReq(BaseModel):
    mode: str = Field(..., description="device_only, simulation_only, device_with_simulation")


class SimSevereReq(BaseModel):
    allow_severe: bool


async def run_simulation_loop():
    """Background task generating natural & severe simulated telemetry and triaged patients."""
    import random
    sim_count = 1
    
    sim_profiles = [
        {"name": ("Amina", "Yusuf"), "complaint": "Severe crushing chest pain radiating to jaw", "symptoms": ["Chest Pain", "Shortness of Breath"], "history": ["Hypertension"]},
        {"name": ("Chidi", "Okonkwo"), "complaint": "High fever with acute chills and rigors", "symptoms": ["High Fever", "Altered Mental Status"], "history": ["Diabetes"]},
        {"name": ("Fatima", "Bello"), "complaint": "Persistent lightheadedness and headache", "symptoms": ["Dizziness"], "history": []},
        {"name": ("Emeka", "Nnamdi"), "complaint": "Wheezing and shortness of breath", "symptoms": ["Shortness of Breath"], "history": ["COPD / Asthma"]},
    ]

    while sim_state["running"]:
        try:
            await asyncio.sleep(2.0)
            if not sim_state["running"]:
                break

            # Determine whether this cycle generates severe critical vitals
            is_severe = sim_state["allow_severe"] and (random.random() < 0.45)

            if is_severe:
                bpm = random.randint(138, 165)
                spo2 = float(random.randint(84, 89))
                temp_ds_c = round(random.uniform(40.1, 41.3), 1)
                temp_max_c = round(random.uniform(38.2, 39.6), 1)
                sbp = random.randint(180, 205)
                dbp = random.randint(110, 125)
                ptt_ms = round(random.uniform(310.0, 350.0), 1)
            else:
                bpm = random.randint(68, 86)
                spo2 = float(random.randint(96, 99))
                temp_ds_c = round(random.uniform(36.5, 37.2), 1)
                temp_max_c = round(random.uniform(33.5, 35.0), 1)
                sbp = random.randint(115, 128)
                dbp = random.randint(76, 84)
                ptt_ms = round(random.uniform(430.0, 490.0), 1)

            reading_payload = {
                "device_id": "sim-esp32-01",
                "timestamp_ms": int(datetime.now(timezone.utc).timestamp() * 1000),
                "bpm": bpm,
                "bpm_valid": True,
                "spo2": spo2,
                "spo2_valid": True,
                "temp_body_c": temp_ds_c,
                "temp_body_f": round(temp_ds_c * 9.0 / 5.0 + 32.0, 1),
                "temp_die_c": temp_max_c,
                "temp_die_f": round(temp_max_c * 9.0 / 5.0 + 32.0, 1),
                "finger_detected": True,
                "device_connected": True,
                "is_simulated": True,
                "ptt_ms": ptt_ms,
                "sbp": sbp,
                "dbp": dbp,
                "bp_valid": True,
                "server_time": datetime.now(timezone.utc).isoformat(),
            }

            global latest_reading
            latest_reading = reading_payload
            realtime_history.append(reading_payload)
            tenmin_history.append(reading_payload)
            save_reading(reading_payload)

            await broadcast_sse({"type": "telemetry", "data": reading_payload})

            # Periodically generate simulated patient intake for triage queue testing
            if random.random() < 0.35:
                prof = random.choice(sim_profiles)
                p_id = f"SIM-PAT-{sim_count:03d}"
                sim_count += 1

                p_record = {
                    "patient_id": p_id,
                    "device_id": "sim-esp32-01",
                    "is_simulated": True,
                    "patient_details": {
                        "first_name": prof["name"][0],
                        "last_name": prof["name"][1],
                        "date_of_birth": "1988-03-24",
                        "gender": "Female" if sim_count % 2 == 0 else "Male"
                    },
                    "chief_complaint": prof["complaint"],
                    "pain_level": random.randint(8, 10) if is_severe else random.randint(2, 5),
                    "symptoms": prof["symptoms"],
                    "medical_history": prof["history"],
                    "symptom_duration": "Acute",
                    "latest_vitals": {
                        "bpm": bpm,
                        "spo2": spo2,
                        "temperature": temp_ds_c,
                        "sbp": sbp,
                        "dbp": dbp,
                        "ptt_ms": ptt_ms,
                    },
                    "created_by_doctor": "SIMULATOR",
                    "timestamp": datetime.now(timezone.utc).isoformat()
                }
                acuity_res = compute_acuity(p_record)
                p_record["acuity"] = acuity_res
                p_record["medical_summary"] = f"Simulated clinical intake for {prof['name'][0]} {prof['name'][1]}. Chief Complaint: {prof['complaint']}. ESI Level: {acuity_res['esi_level']}."

                save_patient(p_record)
                patient_store[p_id] = p_record
                await broadcast_sse({"type": "triage_update", "patient_id": p_id, "acuity": acuity_res})

        except asyncio.CancelledError:
            break
        except Exception as e:
            await asyncio.sleep(2)


@app.get("/api/simulation/state")
async def get_simulation_state():
    """Returns current simulation mode and execution state."""
    return {"status": "success", "state": sim_state}


@app.post("/api/simulation/mode")
async def set_simulation_mode(req: SimModeReq):
    """Sets system operation mode: device_only, simulation_only, device_with_simulation."""
    valid_modes = {"device_only", "simulation_only", "device_with_simulation"}
    if req.mode not in valid_modes:
        raise HTTPException(400, f"Invalid mode '{req.mode}'. Must be one of {valid_modes}")
    
    sim_state["mode"] = req.mode
    
    # Auto start/stop simulation based on mode selection
    global sim_task
    if req.mode in ("simulation_only", "device_with_simulation"):
        if not sim_state["running"]:
            sim_state["running"] = True
            sim_task = asyncio.create_task(run_simulation_loop())
    else:  # device_only
        if sim_state["running"]:
            sim_state["running"] = False
            if sim_task and not sim_task.done():
                sim_task.cancel()
                sim_task = None

    await broadcast_sse({"type": "simulation_state", "state": sim_state})
    return {"status": "success", "state": sim_state}


@app.post("/api/simulation/start")
async def start_simulation():
    """Starts simulation loop."""
    global sim_task
    if not sim_state["running"]:
        sim_state["running"] = True
        sim_task = asyncio.create_task(run_simulation_loop())
    await broadcast_sse({"type": "simulation_state", "state": sim_state})
    return {"status": "success", "message": "Simulation started", "state": sim_state}


@app.post("/api/simulation/stop")
async def stop_simulation():
    """Stops simulation loop without resetting database."""
    global sim_task
    sim_state["running"] = False
    if sim_task and not sim_task.done():
        sim_task.cancel()
        sim_task = None
    await broadcast_sse({"type": "simulation_state", "state": sim_state})
    return {"status": "success", "message": "Simulation stopped", "state": sim_state}


@app.post("/api/simulation/reset")
async def reset_simulation():
    """Stops simulation loop, purges simulated patients and readings from DB and memory."""
    global sim_task, latest_reading
    sim_state["running"] = False
    if sim_task and not sim_task.done():
        sim_task.cancel()
        sim_task = None

    # Clear simulated and test entries from SQLite database
    clear_simulated_data()
    clear_all_demo_patients()

    # Clear in-memory state
    realtime_history.clear()
    tenmin_history.clear()
    latest_reading = None
    patient_store.clear()

    await broadcast_sse({"type": "reset", "message": "Simulation reset completed"})
    await broadcast_sse({"type": "triage_update", "action": "reset"})

    return {"status": "success", "message": "Simulation reset and cleared successfully", "state": sim_state}


@app.post("/api/simulation/severe")
async def toggle_severe_vitals(req: SimSevereReq):
    """Toggles severe consequences setting in simulation."""
    sim_state["allow_severe"] = req.allow_severe
    await broadcast_sse({"type": "simulation_state", "state": sim_state})
    return {"status": "success", "allow_severe": sim_state["allow_severe"]}


@app.post("/api/simulation/inject_severe")
async def inject_severe_patient():
    """Immediately injects a severe critical patient to verify triage priority ranking."""
    import random
    p_id = f"CRIT-PAT-{random.randint(100, 999)}"
    record = {
        "patient_id": p_id,
        "device_id": "sim-esp32-CRIT",
        "is_simulated": True,
        "patient_details": {
            "first_name": "Emergency",
            "last_name": "Critical Patient",
            "date_of_birth": "1975-11-04",
            "gender": "Male"
        },
        "chief_complaint": "CRITICAL HYPOXIA & CHEST PAIN - Resuscitation Needed",
        "pain_level": 10,
        "symptoms": ["Chest Pain", "Shortness of Breath", "Altered Mental Status"],
        "medical_history": ["Heart Disease", "Hypertension"],
        "symptom_duration": "Immediate",
        "latest_vitals": {
            "bpm": 158,
            "spo2": 84,
            "temperature": 40.8,
            "sbp": 195,
            "dbp": 120,
            "ptt_ms": 315.0,
        },
        "created_by_doctor": "SIMULATOR_TEST",
        "timestamp": datetime.now(timezone.utc).isoformat()
    }
    acuity_res = compute_acuity(record)
    record["acuity"] = acuity_res
    record["medical_summary"] = "CRITICAL EMERGENCY INTAKE: Patient exhibits severe hypoxia (SpO2 84%), hyperpyrexia (40.8°C), and hypertensive crisis."

    save_patient(record)
    patient_store[p_id] = record

    await broadcast_sse({"type": "triage_update", "patient_id": p_id, "acuity": acuity_res})
    return {"status": "success", "patient_id": p_id, "acuity": acuity_res}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)
