"""
Kafka Stream Manager with Resilient Confluent Cloud & In-Memory Fallback
─────────────────────────────────────────────────────────────────────────────
Handles production and consumption of events across 5 core triage topics:
  • patient-intake
  • vital-signs
  • nlp-summary
  • acuity-score
  • alerts
"""

import json
import logging
import os
from typing import Any, Callable, Dict, List, Optional, Tuple

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("KafkaStream")

def _load_env_credentials():
    """Auto-load credentials from .env or ccloud-python-client/client.properties."""
    for path in [".env", "ccloud-python-client/client.properties"]:
        if os.path.exists(path):
            try:
                with open(path, "r") as f:
                    for line in f:
                        line = line.strip()
                        if line and not line.startswith("#") and "=" in line:
                            k, v = line.split("=", 1)
                            k, v = k.strip(), v.strip()
                            if k == "bootstrap.servers" and "KAFKA_BOOTSTRAP_SERVERS" not in os.environ:
                                os.environ["KAFKA_BOOTSTRAP_SERVERS"] = v
                            elif k == "sasl.username" and "KAFKA_API_KEY" not in os.environ:
                                os.environ["KAFKA_API_KEY"] = v
                            elif k == "sasl.password" and "KAFKA_API_SECRET" not in os.environ:
                                os.environ["KAFKA_API_SECRET"] = v
                            elif k not in os.environ:
                                os.environ[k] = v
            except Exception as e:
                logger.warning(f"Could not read credential file {path}: {e}")

_load_env_credentials()

KAFKA_BOOTSTRAP_SERVERS = os.getenv("KAFKA_BOOTSTRAP_SERVERS", "127.0.0.1:9092")
KAFKA_API_KEY = os.getenv("KAFKA_API_KEY") or os.getenv("KAFKA_SASL_USERNAME")
KAFKA_API_SECRET = os.getenv("KAFKA_API_SECRET") or os.getenv("KAFKA_SASL_PASSWORD")

TOPICS = {
    "PATIENT_INTAKE": "patient-intake",
    "VITAL_SIGNS":    "vital-signs",
    "NLP_SUMMARY":    "nlp-summary",
    "ACUITY_SCORE":   "acuity-score",
    "ALERTS":         "alerts",
}


# Try importing confluent_kafka; fallback gracefully if not installed/broker unavailable
HAS_KAFKA = False
try:
    from confluent_kafka import Consumer, Producer
    from confluent_kafka.admin import AdminClient, NewTopic
    HAS_KAFKA = True
except ImportError:
    logger.warning("confluent-kafka not installed. Using in-memory fallback pipeline.")


def get_kafka_base_config() -> Dict[str, Any]:
    """Build base configuration for Confluent Cloud or local Kafka."""
    config: Dict[str, Any] = {
        "bootstrap.servers": KAFKA_BOOTSTRAP_SERVERS,
    }
    
    # Configure SASL_SSL if Confluent Cloud credentials exist
    if KAFKA_API_KEY and KAFKA_API_SECRET:
        config.update({
            "security.protocol": os.getenv("KAFKA_SECURITY_PROTOCOL", "SASL_SSL"),
            "sasl.mechanisms": os.getenv("KAFKA_SASL_MECHANISM", "PLAIN"),
            "sasl.username": KAFKA_API_KEY,
            "sasl.password": KAFKA_API_SECRET,
        })
        logger.info(f"Confluent Cloud SASL configuration detected for cluster: {KAFKA_BOOTSTRAP_SERVERS}")
    else:
        logger.info(f"Using standard Kafka connection to: {KAFKA_BOOTSTRAP_SERVERS}")
        
    return config


def init_kafka_topics() -> bool:
    """Create all required Kafka topics asynchronously if broker is active."""
    if not HAS_KAFKA:
        return False
    try:
        admin_config = get_kafka_base_config()
        admin_config["socket.timeout.ms"] = 5000
        
        # Confluent Cloud automatically creates topics, but replication_factor needs to be 3 for cloud clusters
        rep_factor = 3 if KAFKA_API_KEY else 1
        
        admin_client = AdminClient(admin_config)
        new_topics = [
            NewTopic(TOPICS[key], num_partitions=3, replication_factor=rep_factor)
            for key in TOPICS
        ]
        futures = admin_client.create_topics(new_topics, validate_only=False)
        for topic, f in futures.items():
            try:
                f.result(timeout=5.0)
                logger.info(f"Kafka topic '{topic}' ready.")
            except Exception as e:
                if "TopicExistsError" in str(e) or "already exists" in str(e):
                    logger.info(f"Kafka topic '{topic}' exists.")
                else:
                    logger.warning(f"Could not create topic '{topic}': {e}")
        return True
    except Exception as e:
        logger.warning(f"Kafka Admin connection failed: {e}. Falling back to resilient mode.")
        return False


def get_producer() -> Tuple[Any, Callable[[str, str, Dict[str, Any]], None]]:
    """
    Returns a producer instance and send_message helper.
    Falls back to safe logging/in-memory routing if Kafka broker unavailable.
    """
    kafka_prod = None
    if HAS_KAFKA:
        try:
            prod_config = get_kafka_base_config()
            prod_config.update({
                "message.timeout.ms": 5000,
                "socket.timeout.ms": 5000,
            })
            kafka_prod = Producer(prod_config)
            logger.info("Kafka Producer initialized successfully.")
        except Exception as e:
            logger.warning(f"Kafka producer init warning: {e}")

    def delivery_report(err: Any, msg: Any) -> None:
        if err is not None:
            logger.error(f"Kafka delivery error on key {msg.key()}: {err}")
        else:
            logger.debug(f"Message delivered to {msg.topic()} [{msg.partition()}] @ {msg.offset()}")

    def send_message(topic: str, key: str, value: Dict[str, Any]) -> None:
        payload_bytes = json.dumps(value).encode("utf-8")
        key_bytes = key.encode("utf-8") if key else None

        if kafka_prod:
            try:
                kafka_prod.produce(
                    topic=topic,
                    key=key_bytes,
                    value=payload_bytes,
                    callback=delivery_report,
                )
                kafka_prod.poll(0)
                return
            except Exception as e:
                logger.warning(f"Kafka produce error on {topic}: {e}. Retrying in-memory broadcast.")

        logger.info(f"[InMemory Stream -> {topic}] Key: {key} Payload: {list(value.keys())}")

    return kafka_prod, send_message


def get_consumer(group_id: str, topics: List[str]) -> Optional[Any]:
    """
    Returns a configured Consumer subscribed to specified topics.
    """
    if not HAS_KAFKA:
        return None
    try:
        cons_config = get_kafka_base_config()
        cons_config.update({
            "group.id": group_id,
            "auto.offset.reset": "earliest",
            "enable.auto.commit": True,
        })
        consumer = Consumer(cons_config)
        consumer.subscribe(topics)
        logger.info(f"Kafka Consumer ({group_id}) subscribed to: {topics}")
        return consumer
    except Exception as e:
        logger.error(f"Failed to create Kafka Consumer ({group_id}): {e}")
        return None

