"""
Qdrant Cloud vector database connection and operations.
"""

from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from typing import List, Dict, Any
import logging

from src.core.config import settings

logger = logging.getLogger(__name__)


class QdrantManager:
    """Manager class for Qdrant vector database operations."""

    def __init__(self):
        """Initialize Qdrant client with cloud credentials."""
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
        )
        self.collection_name = settings.qdrant_collection_name

    async def create_collection(self):
        """Create collection for textbook embeddings if it doesn't exist."""
        try:
            # Check if collection exists
            collections = self.client.get_collections().collections
            collection_names = [c.name for c in collections]

            if self.collection_name not in collection_names:
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(
                        size=settings.embedding_dimensions,
                        distance=Distance.COSINE,
                    ),
                )
                logger.info(f"Created Qdrant collection: {self.collection_name}")
            else:
                logger.info(f"Collection {self.collection_name} already exists")

        except Exception as e:
            logger.error(f"Error creating Qdrant collection: {e}")
            raise

    def upsert_vectors(
        self,
        vectors: List[List[float]],
        payloads: List[Dict[str, Any]],
        ids: List[int],
    ):
        """
        Insert or update vectors in Qdrant.

        Args:
            vectors: List of embedding vectors
            payloads: List of metadata dictionaries
            ids: List of unique IDs for each vector
        """
        try:
            points = [
                PointStruct(
                    id=point_id,
                    vector=vector,
                    payload=payload,
                )
                for point_id, vector, payload in zip(ids, vectors, payloads)
            ]

            self.client.upsert(
                collection_name=self.collection_name,
                points=points,
            )
            logger.info(f"Upserted {len(points)} vectors to Qdrant")

        except Exception as e:
            logger.error(f"Error upserting vectors to Qdrant: {e}")
            raise

    def search_similar(
        self,
        query_vector: List[float],
        limit: int = 5,
        score_threshold: float = 0.7,
    ):
        """
        Search for similar vectors in Qdrant.

        Args:
            query_vector: Query embedding vector
            limit: Maximum number of results to return
            score_threshold: Minimum similarity score (0-1)

        Returns:
            List of search results with payload and score
        """
        try:
            # Use query_points method for newer qdrant-client versions
            results = self.client.query_points(
                collection_name=self.collection_name,
                query=query_vector,
                limit=limit,
                score_threshold=score_threshold,
            )

            # Return the points from the QueryResponse
            return results.points

        except Exception as e:
            logger.error(f"Error searching Qdrant: {e}")
            raise

    def get_collection_info(self) -> Dict[str, Any]:
        """Get information about the collection."""
        try:
            info = self.client.get_collection(collection_name=self.collection_name)
            return {
                "name": self.collection_name,
                "vectors_count": info.vectors_count,
                "points_count": info.points_count,
                "status": info.status,
            }
        except Exception as e:
            logger.error(f"Error getting collection info: {e}")
            return {"error": str(e)}

    def delete_collection(self):
        """Delete the collection (use with caution)."""
        try:
            self.client.delete_collection(collection_name=self.collection_name)
            logger.info(f"Deleted collection: {self.collection_name}")
        except Exception as e:
            logger.error(f"Error deleting collection: {e}")
            raise


# Global Qdrant manager instance
qdrant_manager = QdrantManager()
