"""
Configuration Module
Loads and manages application configuration from environment variables.
"""
import os
from dotenv import load_dotenv
from src.utils.logger import setup_logger


logger = setup_logger()

# Load environment variables
load_dotenv()


class Config:
    """Configuration class to hold all settings."""

    def __init__(self):
        # API Keys
        self.COHERE_API_KEY = os.getenv("COHERE_API_KEY")
        self.QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
        self.QDRANT_URL = os.getenv("QDRANT_URL")

        # Model settings
        self.COHERE_MODEL = os.getenv("COHERE_MODEL", "embed-english-v3.0")

        # Book site URL
        self.BOOK_SITE_URL = os.getenv("BOOK_SITE_URL") or os.getenv("DEPLOY_VERCEL_URL")

        # Chunking settings
        self.CHUNK_SIZE_MIN = int(os.getenv("CHUNK_SIZE_MIN", "500"))
        self.CHUNK_SIZE_MAX = int(os.getenv("CHUNK_SIZE_MAX", "800"))
        self.CHUNK_OVERLAP = int(os.getenv("CHUNK_OVERLAP", "100"))

        # Request settings
        self.REQUEST_TIMEOUT = int(os.getenv("REQUEST_TIMEOUT", "30"))
        self.RATE_LIMIT_DELAY = float(os.getenv("RATE_LIMIT_DELAY", "1.0"))

        # Qdrant settings
        self.QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "book_embeddings")

        # Validation settings
        self.VALIDATION_THRESHOLD = float(os.getenv("VALIDATION_THRESHOLD", "0.99"))

        # Validate required settings
        self._validate_config()

    def _validate_config(self):
        """Validate that required configuration is present."""
        required_vars = [
            "COHERE_API_KEY",
            "QDRANT_API_KEY",
            "QDRANT_URL"
        ]

        missing_vars = []
        for var in required_vars:
            if not getattr(self, var):
                missing_vars.append(var)

        if missing_vars:
            raise ValueError(f"Missing required environment variables: {', '.join(missing_vars)}")

        logger.info("Configuration loaded and validated successfully")


def load_config():
    """Load and return configuration object."""
    return Config()