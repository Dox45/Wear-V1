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
)

# Initialize SQLite database on startup
init_db()

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
    finger_detected:  bool  = True
    device_connected: Optional[bool]  = True
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
    """Serves biowear_dashboard.html UI."""
    dashboard_path = os.path.join(os.path.dirname(__file__), "biowear_dashboard.html")
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

    record.update({
        "server_time":     datetime.now(timezone.utc).isoformat(),
        "temp_body_f":     round(reading.temp_body_c * 9.0 / 5.0 + 32.0, 2),
        "sbp":             sbp,
        "dbp":             dbp,
        "bp_valid":        bp_valid,
        "map":             round((sbp + 2 * dbp) / 3, 1) if bp_valid else None,
        "pulse_pressure":  round(sbp - dbp, 1) if bp_valid else None,
        "device_connected": True,
        "patient_id":      patient_id,
        "session_id":      session_id,
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

    return {"status": "ok", "sbp": sbp, "dbp": dbp, "bp_valid": bp_valid, "patient_id": patient_id, "session_id": session_id}


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
    }


if __name__ == "__main__":
    import uvicorn
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)
