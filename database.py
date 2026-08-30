"""
SQLite Database Layer for BioWear Medical Triage System
─────────────────────────────────────────────────────────────────────────────
Handles persistent data storage, auto-generation of tables (IF NOT EXISTS),
doctor authentication (password hashing), patient triage record retention,
and vital sign readings logging.
"""

import os
import json
import sqlite3
import hashlib
import secrets
import logging
from datetime import datetime, timezone
from typing import Dict, Any, List, Optional, Tuple

logger = logging.getLogger("Database")

DB_PATH = os.getenv("SQLITE_DB_PATH", os.path.join(os.path.dirname(__file__), "biowear_triage.db"))


def get_db_connection() -> sqlite3.Connection:
    """Returns a SQLite connection with Row factory enabled."""
    conn = sqlite3.connect(DB_PATH, check_same_thread=False)
    conn.row_factory = sqlite3.Row
    return conn


def hash_password(password: str, salt: Optional[str] = None) -> str:
    """Hashes a password using PBKDF2 with SHA-256."""
    if not salt:
        salt = secrets.token_hex(16)
    key = hashlib.pbkdf2_hmac(
        'sha256',
        password.encode('utf-8'),
        salt.encode('utf-8'),
        100000
    )
    return f"{salt}${key.hex()}"


def verify_password(password: str, hashed: str) -> bool:
    """Verifies a plain password against the stored salt$hash format."""
    try:
        salt, key_hex = hashed.split('$')
        computed = hashlib.pbkdf2_hmac(
            'sha256',
            password.encode('utf-8'),
            salt.encode('utf-8'),
            100000
        ).hex()
        return secrets.compare_digest(computed, key_hex)
    except Exception:
        return False


def init_db() -> bool:
    """
    Initializes SQLite database tables using 'CREATE TABLE IF NOT EXISTS'.
    Auto-seeds a default doctor admin account if doctors table is empty.
    """
    try:
        conn = get_db_connection()
        cursor = conn.cursor()

        # 1. Doctors Table
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS doctors (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                username TEXT UNIQUE NOT NULL,
                password_hash TEXT NOT NULL,
                full_name TEXT NOT NULL,
                license_number TEXT NOT NULL,
                department TEXT NOT NULL,
                created_at TEXT NOT NULL
            )
        """)

        # 2. Patients Table
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS patients (
                patient_id TEXT PRIMARY KEY,
                first_name TEXT NOT NULL,
                last_name TEXT NOT NULL,
                date_of_birth TEXT,
                gender TEXT,
                chief_complaint TEXT,
                pain_level INTEGER DEFAULT 0,
                symptoms TEXT,
                symptom_duration TEXT,
                medical_history TEXT,
                current_medications TEXT,
                allergies TEXT,
                vital_signs TEXT,
                acuity_esi INTEGER DEFAULT 5,
                acuity_severity TEXT,
                acuity_score INTEGER DEFAULT 0,
                acuity_action TEXT,
                contributing_factors TEXT,
                medical_summary TEXT,
                status TEXT DEFAULT 'TRIAGED',
                doctor_notes TEXT DEFAULT '',
                created_by_doctor TEXT DEFAULT 'SYSTEM',
                created_at TEXT NOT NULL,
                updated_at TEXT NOT NULL
            )
        """)

        # 3. Readings Table
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS readings (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                device_id TEXT NOT NULL,
                timestamp_ms INTEGER NOT NULL,
                bpm INTEGER,
                spo2 REAL,
                temp_body_c REAL,
                temp_body_f REAL,
                sbp REAL,
                dbp REAL,
                ptt_ms REAL,
                finger_detected INTEGER DEFAULT 1,
                created_at TEXT NOT NULL
            )
        """)

        # 4. Doctor Sessions Table
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS doctor_sessions (
                token TEXT PRIMARY KEY,
                doctor_id INTEGER NOT NULL,
                username TEXT NOT NULL,
                created_at TEXT NOT NULL,
                FOREIGN KEY (doctor_id) REFERENCES doctors (id) ON DELETE CASCADE
            )
        """)

        # 5. Monitoring Sessions Table (Binds patient_id and device_id)
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS monitoring_sessions (
                session_id TEXT PRIMARY KEY,
                patient_id TEXT NOT NULL,
                device_id TEXT NOT NULL,
                status TEXT NOT NULL DEFAULT 'ACTIVE',
                started_at TEXT NOT NULL,
                ended_at TEXT,
                FOREIGN KEY (patient_id) REFERENCES patients (patient_id) ON DELETE CASCADE
            )
        """)

        # Database Schema Migrations for pre-existing tables
        for migration in [
            "ALTER TABLE patients ADD COLUMN device_id TEXT",
            "ALTER TABLE readings ADD COLUMN patient_id TEXT",
            "ALTER TABLE readings ADD COLUMN session_id TEXT",
        ]:
            try:
                cursor.execute(migration)
            except sqlite3.OperationalError:
                pass  # Column already exists


        # Auto-seed default admin doctor if no doctor exists
        cursor.execute("SELECT COUNT(*) FROM doctors")
        if cursor.fetchone()[0] == 0:
            default_pass_hash = hash_password("Doctor123!")
            now_iso = datetime.now(timezone.utc).isoformat()
            cursor.execute("""
                INSERT INTO doctors (username, password_hash, full_name, license_number, department, created_at)
                VALUES (?, ?, ?, ?, ?, ?)
            """, ("admin_doc", default_pass_hash, "Dr. Somtoo Okonkwo", "MD-84920-NG", "Emergency Triage", now_iso))
            logger.info("Default doctor account created: admin_doc / Doctor123!")

        conn.commit()
        conn.close()
        logger.info(f"SQLite Database initialized successfully at: {DB_PATH}")
        return True
    except Exception as e:
        logger.error(f"Error initializing SQLite database: {e}")
        return False


# ─── Doctor Auth Helpers ──────────────────────────────────────────────────────

def create_doctor(username: str, password: str, full_name: str, license_number: str, department: str) -> Optional[Dict[str, Any]]:
    """Registers a new doctor."""
    conn = get_db_connection()
    cursor = conn.cursor()
    pass_hash = hash_password(password)
    now_iso = datetime.now(timezone.utc).isoformat()

    try:
        cursor.execute("""
            INSERT INTO doctors (username, password_hash, full_name, license_number, department, created_at)
            VALUES (?, ?, ?, ?, ?, ?)
        """, (username.strip().lower(), pass_hash, full_name.strip(), license_number.strip(), department.strip(), now_iso))
        conn.commit()
        doc_id = cursor.lastrowid
        conn.close()
        return {
            "id": doc_id,
            "username": username.strip().lower(),
            "full_name": full_name.strip(),
            "license_number": license_number.strip(),
            "department": department.strip(),
            "created_at": now_iso
        }
    except sqlite3.IntegrityError:
        conn.close()
        return None


def authenticate_doctor(username: str, password: str) -> Optional[Dict[str, Any]]:
    """Validates doctor credentials and creates a session token."""
    conn = get_db_connection()
    cursor = conn.cursor()
    cursor.execute("SELECT * FROM doctors WHERE username = ?", (username.strip().lower(),))
    row = cursor.fetchone()

    if not row:
        conn.close()
        return None

    if verify_password(password, row["password_hash"]):
        token = secrets.token_hex(32)
        now_iso = datetime.now(timezone.utc).isoformat()
        cursor.execute("""
            INSERT INTO doctor_sessions (token, doctor_id, username, created_at)
            VALUES (?, ?, ?, ?)
        """, (token, row["id"], row["username"], now_iso))
        conn.commit()
        conn.close()

        return {
            "token": token,
            "doctor": {
                "id": row["id"],
                "username": row["username"],
                "full_name": row["full_name"],
                "license_number": row["license_number"],
                "department": row["department"]
            }
        }
    conn.close()
    return None


def get_doctor_by_token(token: str) -> Optional[Dict[str, Any]]:
    """Returns doctor record associated with active session token."""
    conn = get_db_connection()
    cursor = conn.cursor()
    cursor.execute("""
        SELECT d.id, d.username, d.full_name, d.license_number, d.department, s.created_at as login_time
        FROM doctor_sessions s
        JOIN doctors d ON s.doctor_id = d.id
        WHERE s.token = ?
    """, (token,))
    row = cursor.fetchone()
    conn.close()
    if row:
        return dict(row)
    return None


def delete_session(token: str):
    """Deletes doctor session token on logout."""
    conn = get_db_connection()
    cursor = conn.cursor()
    cursor.execute("DELETE FROM doctor_sessions WHERE token = ?", (token,))
    conn.commit()
    conn.close()


# ─── Monitoring Session Helpers ───────────────────────────────────────────────

import uuid

def create_monitoring_session(patient_id: str, device_id: str) -> Dict[str, Any]:
    """
    Creates a new active monitoring session connecting a patient to an IoT device.
    Automatically closes any existing active session for this device or patient.
    """
    conn = get_db_connection()
    cursor = conn.cursor()
    now_iso = datetime.now(timezone.utc).isoformat()
    session_id = f"SESS-{uuid.uuid4().hex[:8].upper()}"

    # Close previous active sessions for this device or patient
    cursor.execute("""
        UPDATE monitoring_sessions
        SET status = 'CLOSED', ended_at = ?
        WHERE (device_id = ? OR patient_id = ?) AND status = 'ACTIVE'
    """, (now_iso, device_id, patient_id))

    # Update patient device_id
    cursor.execute("""
        UPDATE patients SET device_id = ?, updated_at = ? WHERE patient_id = ?
    """, (device_id, now_iso, patient_id))

    # Insert new active session
    cursor.execute("""
        INSERT INTO monitoring_sessions (session_id, patient_id, device_id, status, started_at)
        VALUES (?, ?, ?, 'ACTIVE', ?)
    """, (session_id, patient_id, device_id, now_iso))

    conn.commit()
    conn.close()

    return {
        "session_id": session_id,
        "patient_id": patient_id,
        "device_id": device_id,
        "status": "ACTIVE",
        "started_at": now_iso
    }


def get_active_session_by_device(device_id: str) -> Optional[Dict[str, Any]]:
    """Returns active monitoring session for a given device_id."""
    conn = get_db_connection()
    cursor = conn.cursor()
    cursor.execute("""
        SELECT * FROM monitoring_sessions WHERE device_id = ? AND status = 'ACTIVE' LIMIT 1
    """, (device_id,))
    row = cursor.fetchone()
    conn.close()
    if row:
        return dict(row)
    return None


def get_active_session_by_patient(patient_id: str) -> Optional[Dict[str, Any]]:
    """Returns active monitoring session for a given patient_id."""
    conn = get_db_connection()
    cursor = conn.cursor()
    cursor.execute("""
        SELECT * FROM monitoring_sessions WHERE patient_id = ? AND status = 'ACTIVE' LIMIT 1
    """, (patient_id,))
    row = cursor.fetchone()
    conn.close()
    if row:
        return dict(row)
    return None


def close_monitoring_session_by_patient(patient_id: str) -> bool:
    """Closes all active monitoring sessions for a patient."""
    conn = get_db_connection()
    cursor = conn.cursor()
    now_iso = datetime.now(timezone.utc).isoformat()
    cursor.execute("""
        UPDATE monitoring_sessions SET status = 'CLOSED', ended_at = ? WHERE patient_id = ? AND status = 'ACTIVE'
    """, (now_iso, patient_id))
    cursor.execute("UPDATE patients SET device_id = NULL, updated_at = ? WHERE patient_id = ?", (now_iso, patient_id))
    closed = cursor.rowcount > 0
    conn.commit()
    conn.close()
    return closed


def close_monitoring_session_by_device(device_id: str) -> bool:
    """Closes active monitoring session for a device."""
    conn = get_db_connection()
    cursor = conn.cursor()
    now_iso = datetime.now(timezone.utc).isoformat()
    cursor.execute("""
        UPDATE monitoring_sessions SET status = 'CLOSED', ended_at = ? WHERE device_id = ? AND status = 'ACTIVE'
    """, (now_iso, device_id))
    closed = cursor.rowcount > 0
    conn.commit()
    conn.close()
    return closed


def get_all_active_sessions() -> List[Dict[str, Any]]:
    """Fetches all currently active monitoring sessions."""
    conn = get_db_connection()
    cursor = conn.cursor()
    cursor.execute("SELECT * FROM monitoring_sessions WHERE status = 'ACTIVE' ORDER BY started_at DESC")
    rows = cursor.fetchall()
    conn.close()
    return [dict(r) for r in rows]


# ─── Patient Storage Helpers ──────────────────────────────────────────────────

def save_patient(patient_record: Dict[str, Any]) -> bool:
    """Inserts or replaces patient record in SQLite."""
    conn = get_db_connection()
    cursor = conn.cursor()

    p_id = patient_record["patient_id"]
    details = patient_record.get("patient_details", {})
    acuity = patient_record.get("acuity", {})
    device_id = patient_record.get("device_id")
    now_iso = datetime.now(timezone.utc).isoformat()

    try:
        cursor.execute("""
            INSERT OR REPLACE INTO patients (
                patient_id, first_name, last_name, date_of_birth, gender,
                chief_complaint, pain_level, symptoms, symptom_duration, medical_history,
                current_medications, allergies, vital_signs,
                acuity_esi, acuity_severity, acuity_score, acuity_action, contributing_factors,
                medical_summary, status, doctor_notes, created_by_doctor, device_id, created_at, updated_at
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """, (
            p_id,
            details.get("first_name", ""),
            details.get("last_name", ""),
            details.get("date_of_birth", "Unknown"),
            details.get("gender", "Unknown"),
            patient_record.get("chief_complaint", ""),
            patient_record.get("pain_level", 0),
            json.dumps(patient_record.get("symptoms", [])),
            patient_record.get("symptom_duration", "Recent"),
            json.dumps(patient_record.get("medical_history", [])),
            json.dumps(patient_record.get("current_medications", [])),
            json.dumps(patient_record.get("allergies", [])),
            json.dumps(patient_record.get("vital_signs", {})),
            acuity.get("esi_level", 5),
            acuity.get("severity", "MINIMAL"),
            acuity.get("raw_score", 0),
            acuity.get("action", "Routine triage"),
            json.dumps(acuity.get("contributing_factors", [])),
            patient_record.get("medical_summary", ""),
            patient_record.get("status", "TRIAGED"),
            patient_record.get("doctor_notes", ""),
            patient_record.get("created_by_doctor", "SYSTEM"),
            device_id,
            patient_record.get("timestamp", now_iso),
            now_iso
        ))
        conn.commit()
        conn.close()
        return True
    except Exception as e:
        logger.error(f"Error saving patient {p_id} to SQLite: {e}")
        conn.close()
        return False


def get_all_patients() -> List[Dict[str, Any]]:
    """Fetches all patient records from SQLite."""
    conn = get_db_connection()
    cursor = conn.cursor()
    cursor.execute("SELECT * FROM patients ORDER BY acuity_esi ASC, acuity_score DESC, updated_at DESC")
    rows = cursor.fetchall()
    conn.close()

    result = []
    for r in rows:
        result.append(format_patient_row(r))
    return result


def get_patient_by_id(patient_id: str) -> Optional[Dict[str, Any]]:
    """Fetches single patient record by patient_id."""
    conn = get_db_connection()
    cursor = conn.cursor()
    cursor.execute("SELECT * FROM patients WHERE patient_id = ?", (patient_id,))
    row = cursor.fetchone()
    conn.close()
    if row:
        return format_patient_row(row)
    return None


def update_patient_status(patient_id: str, status: str, doctor_notes: Optional[str] = None) -> Optional[Dict[str, Any]]:
    """Updates patient triage status and doctor clinical notes."""
    conn = get_db_connection()
    cursor = conn.cursor()
    now_iso = datetime.now(timezone.utc).isoformat()

    if status.upper() == "DISCHARGED":
        close_monitoring_session_by_patient(patient_id)

    if doctor_notes is not None:
        cursor.execute("""
            UPDATE patients SET status = ?, doctor_notes = ?, updated_at = ? WHERE patient_id = ?
        """, (status.upper(), doctor_notes, now_iso, patient_id))
    else:
        cursor.execute("""
            UPDATE patients SET status = ?, updated_at = ? WHERE patient_id = ?
        """, (status.upper(), now_iso, patient_id))

    conn.commit()
    conn.close()
    return get_patient_by_id(patient_id)


def delete_patient(patient_id: str) -> bool:
    """Deletes/discharges patient record from SQLite and closes monitoring session."""
    close_monitoring_session_by_patient(patient_id)
    conn = get_db_connection()
    cursor = conn.cursor()
    cursor.execute("DELETE FROM patients WHERE patient_id = ?", (patient_id,))
    deleted = cursor.rowcount > 0
    conn.commit()
    conn.close()
    return deleted


def format_patient_row(r: sqlite3.Row) -> Dict[str, Any]:
    """Helper to convert sqlite3.Row into dict matching API schema."""
    row_keys = r.keys()
    device_id = r["device_id"] if "device_id" in row_keys else None
    return {
        "patient_id": r["patient_id"],
        "device_id": device_id,
        "timestamp": r["created_at"],
        "updated_at": r["updated_at"],
        "status": r["status"],
        "doctor_notes": r["doctor_notes"] or "",
        "created_by_doctor": r["created_by_doctor"] or "SYSTEM",
        "patient_details": {
            "first_name": r["first_name"],
            "last_name": r["last_name"],
            "date_of_birth": r["date_of_birth"],
            "gender": r["gender"]
        },
        "chief_complaint": r["chief_complaint"],
        "pain_level": r["pain_level"],
        "symptom_duration": r["symptom_duration"],
        "symptoms": json.loads(r["symptoms"] or "[]"),
        "medical_history": json.loads(r["medical_history"] or "[]"),
        "current_medications": json.loads(r["current_medications"] or "[]"),
        "allergies": json.loads(r["allergies"] or "[]"),
        "vital_signs": json.loads(r["vital_signs"] or "{}"),
        "acuity": {
            "esi_level": r["acuity_esi"],
            "severity": r["acuity_severity"],
            "raw_score": r["acuity_score"],
            "action": r["acuity_action"],
            "contributing_factors": json.loads(r["contributing_factors"] or "[]")
        },
        "medical_summary": r["medical_summary"]
    }


# ─── Sensor Readings Storage ──────────────────────────────────────────────────

def save_reading(reading_data: Dict[str, Any]) -> bool:
    """Logs raw telemetry reading into readings table with optional patient_id & session_id."""
    conn = get_db_connection()
    cursor = conn.cursor()
    now_iso = datetime.now(timezone.utc).isoformat()

    try:
        cursor.execute("""
            INSERT INTO readings (
                device_id, patient_id, session_id, timestamp_ms, bpm, spo2, temp_body_c, temp_body_f, sbp, dbp, ptt_ms, finger_detected, created_at
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """, (
            reading_data.get("device_id", "esp32-01"),
            reading_data.get("patient_id"),
            reading_data.get("session_id"),
            reading_data.get("timestamp_ms", 0),
            reading_data.get("bpm", 0),
            reading_data.get("spo2", 0.0),
            reading_data.get("temp_body_c", 0.0),
            reading_data.get("temp_body_f", 0.0),
            reading_data.get("sbp"),
            reading_data.get("dbp"),
            reading_data.get("ptt_ms"),
            1 if reading_data.get("finger_detected") else 0,
            now_iso
        ))
        conn.commit()
        conn.close()
        return True
    except Exception as e:
        logger.error(f"Error logging reading to SQLite: {e}")
        conn.close()
        return False


def get_patient_readings(patient_id: str, limit: int = 100) -> List[Dict[str, Any]]:
    """Fetches vital sign reading history for a specific patient."""
    conn = get_db_connection()
    cursor = conn.cursor()
    cursor.execute("""
        SELECT * FROM readings WHERE patient_id = ? ORDER BY created_at DESC LIMIT ?
    """, (patient_id, limit))
    rows = cursor.fetchall()
    conn.close()
    return [dict(r) for r in rows]


def get_latest_patient_reading(patient_id: str) -> Optional[Dict[str, Any]]:
    """Fetches latest vital sign reading for a specific patient."""
    conn = get_db_connection()
    cursor = conn.cursor()
    cursor.execute("""
        SELECT * FROM readings WHERE patient_id = ? ORDER BY created_at DESC LIMIT 1
    """, (patient_id,))
    row = cursor.fetchone()
    conn.close()
    if row:
        return dict(row)
    return None

