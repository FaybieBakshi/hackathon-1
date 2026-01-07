try:
    from pydantic.v1 import BaseSettings
except ImportError:
    from pydantic import BaseSettings
from typing import List, Optional
import os


class Settings(BaseSettings):
    # API Settings
    api_title: str = "RAG Chatbot API"
    api_description: str = "API for RAG-based chatbot with documentation querying capabilities"
    api_version: str = "1.0.0"

    # Environment-based configuration
    environment: str = "development"  # development, staging, production

    # CORS Settings
    allowed_origins: List[str] = [
        "http://localhost:3000",      # Local development
        "http://localhost:3001",      # Alternative local dev
        "http://localhost:3002",      # Docusaurus default
        "http://localhost:4000",      # Docusaurus alternate port
        "http://127.0.0.1:3000",      # Alternative localhost
        "http://127.0.0.1:3001",      # Alternative localhost
        "http://127.0.0.1:3002",      # Alternative localhost
        "http://127.0.0.1:4000",      # Alternative localhost
    ]

    # Add production domains based on environment
    @property
    def production_origins(self) -> List[str]:
        if self.environment == "production":
            return [
                os.getenv("PRODUCTION_ORIGIN", "https://your-production-domain.com"),
                os.getenv("PRODUCTION_ORIGIN_ALT", "https://www.your-production-domain.com")
            ]
        return []

    # RAG Agent Settings
    rag_model_name: str = "default-rag-model"
    rag_max_tokens: int = 2048
    rag_temperature: float = 0.7

    # Rate Limiting
    rate_limit_requests: int = 100  # requests per hour per IP
    rate_limit_window: int = 3600   # seconds

    # Logging
    log_level: str = "INFO"

    # Documentation Sources
    documentation_sources: List[str] = []

    class Config:
        env_file = ".env"
        case_sensitive = False


# Global settings instance
_settings: Optional[Settings] = None


def get_settings() -> Settings:
    global _settings
    if _settings is None:
        _settings = Settings()
    return _settings