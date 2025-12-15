"""
Core configuration module for the AI Textbook Platform backend.
Loads environment variables and provides settings management.
"""

from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # Application
    app_name: str = "AI Textbook Platform API"
    app_version: str = "1.0.0"
    debug: bool = False

    # Database - Neon Serverless Postgres
    neon_database_url: str

    # Vector Database - Qdrant Cloud
    qdrant_url: str
    qdrant_api_key: str
    qdrant_collection_name: str = "textbook_embeddings"

    # OpenAI API
    openai_api_key: str
    embedding_model: str = "text-embedding-3-small"
    embedding_dimensions: int = 1536
    chat_model: str = "gpt-4-turbo-preview"

    # Authentication
    jwt_secret: str
    jwt_algorithm: str = "HS256"
    jwt_expiration_hours: int = 24

    # CORS
    cors_origins: list[str] = [
        "http://localhost:3000",
        "https://*.vercel.app"
    ]

    # Rate Limiting
    rate_limit_per_minute: int = 100

    # Chunking settings
    chunk_size: int = 512
    chunk_overlap: int = 50

    class Config:
        env_file = ".env"
        case_sensitive = False


# Global settings instance
settings = Settings()
