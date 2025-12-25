"""
Main FastAPI application for the AI Textbook Platform.
"""

from dotenv import load_dotenv

# Load environment variables FIRST
load_dotenv()

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
import logging

from src.core.config import settings
from src.database.postgres import init_db, close_db
from src.database.qdrant import qdrant_manager
from src.utils.logging import setup_logging
from src.models import User, ChatSession, ChatMessage, PersonalizationPreference  # Import models to register with SQLAlchemy

# Setup logging
logger = setup_logging()


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan context manager for startup and shutdown events.
    """
    # Startup
    logger.info("Starting up AI Textbook Platform API...")

    # Initialize databases
    try:
        await init_db()
        logger.info("Neon Postgres database initialized")

        await qdrant_manager.create_collection()
        logger.info("Qdrant vector store initialized")

    except Exception as e:
        logger.error(f"Error during startup: {e}")
        raise

    yield

    # Shutdown
    logger.info("Shutting down AI Textbook Platform API...")
    await close_db()


# Create FastAPI app
app = FastAPI(
    title=settings.app_name,
    version=settings.app_version,
    lifespan=lifespan,
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/")
async def root():
    """Root endpoint."""
    return {
        "message": "AI Textbook Platform API",
        "version": settings.app_version,
        "docs": "/docs",
    }


@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {"status": "healthy"}


@app.get("/api/v1/status")
async def status():
    """
    Status endpoint to verify database connections and system readiness.

    Returns:
        dict: System status including database connections
    """
    status_response = {
        "api": "online",
        "version": settings.app_version,
    }

    # Check Neon Postgres connection
    try:
        from src.database.postgres import engine

        async with engine.connect() as conn:
            await conn.execute("SELECT 1")
        status_response["postgres"] = "connected"
    except Exception as e:
        logger.error(f"Postgres connection error: {e}")
        status_response["postgres"] = f"error: {str(e)}"

    # Check Qdrant connection
    try:
        collection_info = qdrant_manager.get_collection_info()
        if "error" in collection_info:
            status_response["qdrant"] = f"error: {collection_info['error']}"
        else:
            status_response["qdrant"] = {
                "status": "connected",
                "collection": collection_info.get("name"),
                "vectors_count": collection_info.get("vectors_count", 0),
            }
    except Exception as e:
        logger.error(f"Qdrant connection error: {e}")
        status_response["qdrant"] = f"error: {str(e)}"

    # Check Cohere API key (for embeddings)
    if settings.cohere_api_key:
        status_response["cohere"] = "api_key_configured"
    else:
        status_response["cohere"] = "api_key_missing"

    # Check Groq API key (for chat LLM)
    if settings.groq_api_key:
        status_response["groq"] = "api_key_configured"
    else:
        status_response["groq"] = "api_key_missing (optional)"

    return status_response


# Include API routers
from src.api.routes.search import router as search_router
from src.api.routes.auth import router as auth_router
from src.api.skills import router as skills_router
from src.api.personalization import router as personalization_router
from src.api.translation import router as translation_router

app.include_router(search_router, prefix="/api/v1", tags=["search"])
app.include_router(auth_router, prefix="/api/v1/auth", tags=["authentication"])
app.include_router(skills_router, prefix="/api/v1/skills", tags=["skills"])
app.include_router(personalization_router, prefix="/api/v1/personalize", tags=["personalization"])
app.include_router(translation_router, prefix="/api/v1/translate", tags=["translation"])


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
    )
