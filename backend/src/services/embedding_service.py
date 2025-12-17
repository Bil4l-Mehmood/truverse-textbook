"""
Embedding service using Cohere API (free tier, no dependencies).
Get free API key at: https://dashboard.cohere.com/api-keys
"""

import httpx
import logging
from typing import List

logger = logging.getLogger(__name__)

# Cohere API configuration (FREE tier: 100 requests/min)
COHERE_API_URL = "https://api.cohere.ai/v1/embed"


def _get_api_key() -> str:
    """Get Cohere API key from settings."""
    from src.core.config import settings
    return settings.cohere_api_key


def generate_embedding(text: str) -> List[float]:
    """
    Generate embedding for text using Cohere API.

    Args:
        text: Input text

    Returns:
        Embedding vector (1024 dimensions)
    """
    api_key = _get_api_key()
    if not api_key:
        raise ValueError(
            "COHERE_API_KEY not found in environment variables. "
            "Get free key at: https://dashboard.cohere.com/api-keys"
        )

    try:
        headers = {
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json",
        }

        with httpx.Client(timeout=30.0) as client:
            response = client.post(
                COHERE_API_URL,
                headers=headers,
                json={
                    "texts": [text],
                    "model": "embed-english-light-v3.0",  # Fast, free tier model
                    "input_type": "search_document",
                },
            )
            response.raise_for_status()
            result = response.json()
            return result["embeddings"][0]

    except Exception as e:
        logger.error(f"Error generating embedding: {e}")
        raise


def generate_embeddings_batch(texts: List[str]) -> List[List[float]]:
    """
    Generate embeddings for multiple texts in batch.

    Args:
        texts: List of input texts (max 96 texts per batch)

    Returns:
        List of embedding vectors
    """
    api_key = _get_api_key()
    if not api_key:
        raise ValueError(
            "COHERE_API_KEY not found in environment variables. "
            "Get free key at: https://dashboard.cohere.com/api-keys"
        )

    try:
        headers = {
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json",
        }

        # Cohere allows up to 96 texts per request
        batch_size = 96
        all_embeddings = []

        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]

            with httpx.Client(timeout=60.0) as client:
                response = client.post(
                    COHERE_API_URL,
                    headers=headers,
                    json={
                        "texts": batch,
                        "model": "embed-english-light-v3.0",
                        "input_type": "search_document",
                    },
                )
                response.raise_for_status()
                result = response.json()
                all_embeddings.extend(result["embeddings"])

        return all_embeddings

    except Exception as e:
        logger.error(f"Error generating batch embeddings: {e}")
        raise


def get_embedding_dimensions() -> int:
    """Get the embedding dimensions for this model."""
    return 384  # embed-english-light-v3.0 produces 384-dimensional embeddings
