"""
Schemas for search and RAG endpoints.
"""

from pydantic import BaseModel, Field
from typing import List, Optional


class SearchRequest(BaseModel):
    """Request model for semantic search."""

    query: str = Field(..., description="Search query text", min_length=1)
    limit: int = Field(default=5, description="Maximum number of results", ge=1, le=20)
    score_threshold: float = Field(
        default=0.3,
        description="Minimum similarity score (0-1)",
        ge=0.0,
        le=1.0,
    )


class SearchResult(BaseModel):
    """Single search result."""

    id: int = Field(..., description="Result ID")
    score: float = Field(..., description="Similarity score")
    chapter_id: str = Field(..., description="Chapter identifier")
    title: str = Field(..., description="Chapter title")
    text: str = Field(..., description="Matching text content")
    chunk_index: int = Field(..., description="Chunk index within chapter")
    total_chunks: int = Field(..., description="Total chunks in chapter")


class SearchResponse(BaseModel):
    """Response model for semantic search."""

    query: str = Field(..., description="Original search query")
    results: List[SearchResult] = Field(..., description="Search results")
    count: int = Field(..., description="Number of results returned")


class ChatRequest(BaseModel):
    """Request model for RAG chat."""

    question: str = Field(..., description="User question", min_length=1)
    session_id: Optional[str] = Field(None, description="Chat session ID")
    use_history: bool = Field(default=True, description="Include chat history in context")
    top_k: int = Field(default=3, description="Number of context chunks to retrieve", ge=1, le=10)


class ChatResponse(BaseModel):
    """Response model for RAG chat."""

    answer: str = Field(..., description="AI-generated answer")
    session_id: str = Field(..., description="Chat session ID")
    sources: List[SearchResult] = Field(..., description="Source documents used")
    latency_ms: float = Field(..., description="Response latency in milliseconds")
