"""
Search and RAG API endpoints.
"""

from fastapi import APIRouter, HTTPException, status
from typing import List
import logging

from src.schemas.search import (
    SearchRequest,
    SearchResponse,
    SearchResult,
    ChatRequest,
    ChatResponse,
)
from src.services.embedding_service import generate_embedding
from src.database.qdrant import qdrant_manager

logger = logging.getLogger(__name__)
router = APIRouter()


@router.post("/search", response_model=SearchResponse)
async def semantic_search(request: SearchRequest):
    """
    Perform semantic search over textbook content.

    This endpoint:
    1. Generates an embedding for the search query
    2. Searches Qdrant for similar content chunks
    3. Returns ranked results with metadata

    Args:
        request: Search request with query and parameters

    Returns:
        SearchResponse with ranked results
    """
    try:
        logger.info(f"Search query: {request.query}")

        # Generate query embedding
        query_embedding = generate_embedding(request.query)

        # Search Qdrant
        raw_results = qdrant_manager.search_similar(
            query_vector=query_embedding,
            limit=request.limit,
            score_threshold=request.score_threshold,
        )

        # Format results
        results = [
            SearchResult(
                id=result.id,
                score=result.score,
                chapter_id=result.payload.get("chapter_id", ""),
                title=result.payload.get("title", "Untitled"),
                text=result.payload.get("text", ""),
                chunk_index=result.payload.get("chunk_index", 0),
                total_chunks=result.payload.get("total_chunks", 1),
            )
            for result in raw_results
        ]

        logger.info(f"Found {len(results)} results")

        return SearchResponse(
            query=request.query,
            results=results,
            count=len(results),
        )

    except Exception as e:
        logger.error(f"Search error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Search failed: {str(e)}",
        )


@router.post("/chat", response_model=ChatResponse)
async def rag_chat(request: ChatRequest):
    """
    RAG-based chat endpoint with context retrieval.

    This endpoint:
    1. Retrieves relevant context from textbook
    2. Generates answer using LLM with context
    3. Stores chat history in session

    Args:
        request: Chat request with question and session info

    Returns:
        ChatResponse with AI-generated answer and sources

    Note:
        Requires GROQ_API_KEY for LLM functionality.
    """
    import time
    import uuid
    from src.core.config import settings

    start_time = time.time()

    try:
        # Generate session ID if not provided
        session_id = request.session_id or str(uuid.uuid4())

        logger.info(f"Chat question: {request.question} (session: {session_id})")

        # 1. Retrieve context from textbook
        query_embedding = generate_embedding(request.question)
        context_results = qdrant_manager.search_similar(
            query_vector=query_embedding,
            limit=request.top_k,
            score_threshold=0.3,
        )

        # Format sources
        sources = [
            SearchResult(
                id=result.id,
                score=result.score,
                chapter_id=result.payload.get("chapter_id", ""),
                title=result.payload.get("title", "Untitled"),
                text=result.payload.get("text", ""),
                chunk_index=result.payload.get("chunk_index", 0),
                total_chunks=result.payload.get("total_chunks", 1),
            )
            for result in context_results
        ]

        # Build context from retrieved chunks
        context = "\n\n".join([f"[{s.title}]\n{s.text}" for s in sources[:3]])

        # 2. Generate answer with LLM (if Groq API key available)
        if settings.groq_api_key:
            from groq import Groq

            client = Groq(api_key=settings.groq_api_key)

            # Build prompt with context
            prompt = f"""You are an AI teaching assistant for a Physical AI & Humanoid Robotics course.
Use the following context from the textbook to answer the student's question.
If the context doesn't contain relevant information, say so honestly.

CONTEXT:
{context}

QUESTION: {request.question}

ANSWER:"""

            # Call Groq LLM
            response = client.chat.completions.create(
                model=settings.chat_model,
                messages=[{"role": "user", "content": prompt}],
                temperature=0.7,
                max_tokens=500,
            )

            answer = response.choices[0].message.content

        else:
            # Fallback if no LLM API key
            answer = (
                f"I found {len(sources)} relevant sections about your question. "
                f"To get AI-generated answers, please configure GROQ_API_KEY. "
                f"For now, here are the most relevant textbook sections:\n\n"
            )
            for i, source in enumerate(sources[:2], 1):
                answer += f"{i}. {source.title}: {source.text[:200]}...\n\n"

        # Calculate latency
        latency_ms = (time.time() - start_time) * 1000

        logger.info(f"Chat response generated in {latency_ms:.2f}ms")

        return ChatResponse(
            answer=answer,
            session_id=session_id,
            sources=sources,
            latency_ms=latency_ms,
        )

    except Exception as e:
        logger.error(f"Chat error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Chat failed: {str(e)}",
        )
