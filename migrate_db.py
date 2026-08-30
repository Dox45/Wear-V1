#!/usr/bin/env python3
"""
Production Database Migration & Verification Script — BioWear Medical System
─────────────────────────────────────────────────────────────────────────────
Verifies SQLite schema integrity, executes pending table/column migrations,
creates performance indexes, and seeds default admin account for deployment.

Usage:
    python3 migrate_db.py
"""

import os
import sys
import sqlite3
import logging
from datetime import datetime, timezone

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
)
logger = logging.getLogger("DBMigrator")

DB_PATH = os.getenv("SQLITE_DB_PATH", os.path.join(os.path.dirname(__file__), "biowear_triage.db"))

# Required Schema Definitions
TABLE_SCHEMAS = {
    "doctors": """
        CREATE TABLE IF NOT EXISTS doctors (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            username TEXT UNIQUE NOT NULL,
            password_hash TEXT NOT NULL,
            full_name TEXT NOT NULL,
            license_number TEXT NOT NULL,
            department TEXT NOT NULL,
            created_at TEXT NOT NULL
        )
    """,
    "patients": """
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
            device_id TEXT,
            created_at TEXT NOT NULL,
            updated_at TEXT NOT NULL
        )
    """,
    "readings": """
        CREATE TABLE IF NOT EXISTS readings (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            device_id TEXT NOT NULL,
            patient_id TEXT,
            session_id TEXT,
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
    """,
    "doctor_sessions": """
        CREATE TABLE IF NOT EXISTS doctor_sessions (
            token TEXT PRIMARY KEY,
            doctor_id INTEGER NOT NULL,
            username TEXT NOT NULL,
            created_at TEXT NOT NULL,
            FOREIGN KEY (doctor_id) REFERENCES doctors (id) ON DELETE CASCADE
        )
    """,
    "monitoring_sessions": """
        CREATE TABLE IF NOT EXISTS monitoring_sessions (
            session_id TEXT PRIMARY KEY,
            patient_id TEXT NOT NULL,
            device_id TEXT NOT NULL,
            status TEXT NOT NULL DEFAULT 'ACTIVE',
            started_at TEXT NOT NULL,
            ended_at TEXT,
            FOREIGN KEY (patient_id) REFERENCES patients (patient_id) ON DELETE CASCADE
        )
    """
}

# Required Column Migrations for Schema Upgrades
COLUMN_MIGRATIONS = [
    ("patients", "device_id", "TEXT"),
    ("readings", "patient_id", "TEXT"),
    ("readings", "session_id", "TEXT"),
]

# Production Performance Indexes
INDEXES = [
    ("idx_readings_patient_id", "CREATE INDEX IF NOT EXISTS idx_readings_patient_id ON readings(patient_id)"),
    ("idx_readings_device_id", "CREATE INDEX IF NOT EXISTS idx_readings_device_id ON readings(device_id)"),
    ("idx_monitoring_sessions_active", "CREATE INDEX IF NOT EXISTS idx_monitoring_sessions_active ON monitoring_sessions(device_id, status)"),
    ("idx_monitoring_sessions_patient", "CREATE INDEX IF NOT EXISTS idx_monitoring_sessions_patient ON monitoring_sessions(patient_id, status)"),
    ("idx_patients_status", "CREATE INDEX IF NOT EXISTS idx_patients_status ON patients(status, acuity_esi)"),
]


def run_migrations() -> bool:
    """Runs database verification, table auto-creation, column migrations, and index setup."""
    logger.info(f"Target Database File: {DB_PATH}")

    try:
        conn = sqlite3.connect(DB_PATH)
        conn.row_factory = sqlite3.Row
        cursor = conn.cursor()

        # Enable WAL Mode for High Performance Production Writes
        cursor.execute("PRAGMA journal_mode=WAL;")
        cursor.execute("PRAGMA foreign_keys=ON;")

        logger.info("Step 1/4: Checking and creating required database tables...")
        for table_name, create_sql in TABLE_SCHEMAS.items():
            cursor.execute(create_sql)
            logger.info(f"  ✓ Table verified: {table_name}")

        logger.info("Step 2/4: Checking and executing column migrations...")
        for table_name, column_name, column_type in COLUMN_MIGRATIONS:
            cursor.execute(f"PRAGMA table_info({table_name})")
            existing_cols = [row["name"] for row in cursor.fetchall()]
            if column_name not in existing_cols:
                logger.info(f"  + Migrating table '{table_name}': Adding column '{column_name}' ({column_type})...")
                cursor.execute(f"ALTER TABLE {table_name} ADD COLUMN {column_name} {column_type}")
            else:
                logger.info(f"  ✓ Column verified: {table_name}.{column_name}")

        logger.info("Step 3/4: Creating production database indexes...")
        for index_name, index_sql in INDEXES:
            cursor.execute(index_sql)
            logger.info(f"  ✓ Index verified: {index_name}")

        logger.info("Step 4/4: Verifying admin doctor seed account...")
        cursor.execute("SELECT COUNT(*) FROM doctors")
        if cursor.fetchone()[0] == 0:
            from database import hash_password
            default_pass_hash = hash_password("Doctor123!")
            now_iso = datetime.now(timezone.utc).isoformat()
            cursor.execute("""
                INSERT INTO doctors (username, password_hash, full_name, license_number, department, created_at)
                VALUES (?, ?, ?, ?, ?, ?)
            """, ("admin_doc", default_pass_hash, "Dr. Somtoo Okonkwo", "MD-84920-NG", "Emergency Triage", now_iso))
            logger.info("  + Auto-seeded default admin account: admin_doc / Doctor123!")
        else:
            logger.info("  ✓ Doctor accounts present.")

        conn.commit()
        conn.close()

        logger.info("✅ Database migration and verification successfully completed!")
        return True

    except Exception as e:
        logger.error(f"❌ Error during database migration: {e}", exc_info=True)
        return False


def verify_tables() -> bool:
    """Inspects SQLite database and prints schema report."""
    try:
        conn = sqlite3.connect(DB_PATH)
        conn.row_factory = sqlite3.Row
        cursor = conn.cursor()

        cursor.execute("SELECT name FROM sqlite_master WHERE type='table' ORDER BY name")
        tables = [r["name"] for r in cursor.fetchall() if not r["name"].startswith("sqlite_")]

        logger.info("─── Database Schema Verification Report ───")
        for table in tables:
            cursor.execute(f"PRAGMA table_info({table})")
            cols = [f"{r['name']} ({r['type']})" for r in cursor.fetchall()]
            cursor.execute(f"SELECT COUNT(*) FROM {table}")
            row_count = cursor.fetchone()[0]
            logger.info(f"Table: {table} | Rows: {row_count} | Columns ({len(cols)}): {', '.join(cols[:5])}...")

        conn.close()
        return True
    except Exception as e:
        logger.error(f"Failed schema verification check: {e}")
        return False


if __name__ == "__main__":
    success = run_migrations()
    if success:
        verify_tables()
        sys.exit(0)
    else:
        sys.exit(1)
