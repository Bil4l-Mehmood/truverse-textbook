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

    # LLM API - Using Groq (free tier)
    groq_api_key: str = ""  # Optional: Get free key at console.groq.com
    chat_model: str = "llama-3.1-8b-instant"  # Free, fast model

    # Embeddings - Using Cohere API (free tier)
    cohere_api_key: str  # Required: Get free key at dashboard.cohere.com/api-keys
    embedding_model: str = "embed-english-light-v3.0"
    embedding_dimensions: int = 384

    # Authentication
    jwt_secret: str
    jwt_algorithm: str = "HS256"
    jwt_expiration_hours: int = 24

    # Better Auth Secret (shared with auth-server)
    better_auth_secret: str = "09d25e094faa6ca2556c818166b7a9563b93f7099f6f0f4caa6cf63b88e8d3e7"

    # CORS
    cors_origins: list[str] = [
        "http://localhost:3000",
        "http://localhost:3001",
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
