"""
HealthMonitor FastAPI Backend — v2
─────────────────────────────────────────────────────────────────────────────
New in v2:
  • PTT/PAT-based blood pressure estimation (systolic + diastolic)
  • Per-device calibration constants stored in memory
  • Dual history rings: 60-second (real-time) + 10-minute
  • /calibrate  endpoint to set per-device BP calibration
  • /readings/realtime  →  last 60 readings  (1 Hz → ~1 min)
  • /readings/ten_min   →  last 600 readings (1 Hz → ~10 min)
  • /readings/latest    →  single most-recent reading
  • /          →  serves dashboard.html

Run locally:
    pip install fastapi uvicorn
    uvicorn main:app --host 0.0.0.0 --port 8000 --reload

Deploy to Render:
    Start command:  uvicorn main:app --host 0.0.0.0 --port $PORT
    (Render injects $PORT automatically)

─────────────────────────────────────────────────────────────────────────────
PTT / PAT Blood Pressure Model
─────────────────────────────────────────────────────────────────────────────
The ESP32 sends `ptt_ms` — the Pulse Transit Time derived from the
interval between consecutive IR peaks in the MAX30102 waveform.

BP is estimated with the linear PTT model:
    SBP = sbp_slope * (1 / PTT_s) + sbp_intercept
    DBP = dbp_slope * (1 / PTT_s) + dbp_intercept

Default calibration constants are population averages from literature:
    sbp_slope=-37.0, sbp_intercept=198.0
    dbp_slope=-21.0, dbp_intercept=130.0

These MUST be personalised per user via POST /calibrate.

Reference: Poon & Zhang (2005), IEEE EMBC; Kachuee et al. (2015).
developed by: Chima Emmanuel
─────────────────────────────────────────────────────────────────────────────
"""

from __future__ import annotations

import os
from collections import deque
from datetime import datetime, timezone
from typing import Deque, Dict, Optional

from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field

# ─── Config ──────────────────────────────────────────────────────────────────
REALTIME_SIZE = 60    # ~1 minute  at 1 Hz
TEN_MIN_SIZE  = 600   # ~10 minutes at 1 Hz

DEFAULT_CAL = {
    "sbp_slope":     -37.0,
    "sbp_intercept":  198.0,
    "dbp_slope":     -21.0,
    "dbp_intercept":  130.0,
}

PTT_MIN_MS = 100
PTT_MAX_MS = 600

# ─── App ─────────────────────────────────────────────────────────────────────
app = FastAPI(title="HealthMonitor API", version="2.0.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# ─── In-memory stores ────────────────────────────────────────────────────────
realtime_history: Deque[dict] = deque(maxlen=REALTIME_SIZE)
tenmin_history:   Deque[dict] = deque(maxlen=TEN_MIN_SIZE)
latest_reading:   Optional[dict] = None
calibrations:     Dict[str, dict] = {}


# ─── PTT → Blood Pressure ────────────────────────────────────────────────────
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


# ─── Schemas ─────────────────────────────────────────────────────────────────
class Reading(BaseModel):
    device_id:       str
    timestamp_ms:    int
    bpm:             int   = Field(..., ge=0)
    bpm_valid:       bool
    spo2:            float = Field(..., ge=0, le=100)
    spo2_valid:      bool
    temp_body_c:     float
    temp_body_f:     float
    temp_die_c:      float
    finger_detected: bool
    ptt_ms:          Optional[float] = Field(
        None,
        description="Pulse Transit Time in ms from consecutive IR waveform peaks."
    )


class CalibrationRequest(BaseModel):
    device_id:       str
    sbp_slope:       float = Field(DEFAULT_CAL["sbp_slope"])
    sbp_intercept:   float = Field(DEFAULT_CAL["sbp_intercept"])
    dbp_slope:       float = Field(DEFAULT_CAL["dbp_slope"])
    dbp_intercept:   float = Field(DEFAULT_CAL["dbp_intercept"])
    # Optional single-point calibration via reference cuff reading
    ref_sbp:         Optional[float] = None
    ref_dbp:         Optional[float] = None
    ref_ptt_ms:      Optional[float] = None


# ─── Routes ──────────────────────────────────────────────────────────────────
@app.post("/readings", status_code=201)
async def ingest_reading(reading: Reading):
    global latest_reading

    cal = calibrations.get(reading.device_id, DEFAULT_CAL)
    sbp, dbp = (None, None)
    bp_valid = False

    if reading.ptt_ms is not None:
        sbp, dbp = estimate_bp(reading.ptt_ms, cal)
        bp_valid = sbp is not None

    record = reading.model_dump()
    record.update({
        "server_time":    datetime.now(timezone.utc).isoformat(),
        "sbp":            sbp,
        "dbp":            dbp,
        "bp_valid":       bp_valid,
        "map":            round((sbp + 2 * dbp) / 3, 1) if bp_valid else None,
        "pulse_pressure": round(sbp - dbp, 1)            if bp_valid else None,
    })

    realtime_history.append(record)
    tenmin_history.append(record)
    latest_reading = record

    return {"status": "ok", "sbp": sbp, "dbp": dbp, "bp_valid": bp_valid}


@app.post("/calibrate")
async def calibrate(req: CalibrationRequest):

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
        return JSONResponse({"error": "No data yet"}, status_code=404)
    return latest_reading


@app.get("/readings/realtime")
async def get_realtime():
    return list(realtime_history)


@app.get("/readings/ten_min")
async def get_ten_min():
    return list(tenmin_history)


@app.get("/calibrations")
async def get_calibrations():
    return calibrations


@app.get("/", response_class=HTMLResponse)
async def dashboard():
    with open("dashboard.html", "r") as f:
        return f.read()
