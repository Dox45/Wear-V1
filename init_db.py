#!/usr/bin/env python3
"""
Standalone Database Initialization Script for BioWear Medical Triage System
─────────────────────────────────────────────────────────────────────────────
Autogenerates the SQLite database and all required tables using 'IF NOT EXISTS'.
Can be executed during initial deployment or server startup:
    python3 init_db.py
"""

import sys
import logging
from migrate_db import run_migrations, verify_tables, DB_PATH

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")
logger = logging.getLogger("InitDB")

def main():
    logger.info("Initializing & Migrating BioWear SQLite Database...")
    logger.info(f"Target Database File: {DB_PATH}")

    success = run_migrations()
    if success:
        verify_tables()
        logger.info("✅ Database initialization & schema migration complete.")
        sys.exit(0)
    else:
        logger.error("❌ Database initialization & migration failed.")
        sys.exit(1)

if __name__ == "__main__":
    main()

