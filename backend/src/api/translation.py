"""
API endpoints for content translation to Urdu.
"""

import logging
from fastapi import APIRouter, Header, HTTPException
from pydantic import BaseModel
from sqlalchemy import select
from datetime import datetime, timedelta

from src.services.translation_service import TranslationService
from src.core.config import settings
from src.database.postgres import get_db
from src.models import User

logger = logging.getLogger(__name__)
router = APIRouter()

translation_service = TranslationService()


class TranslateRequest(BaseModel):
    """Request to translate chapter content to Urdu."""
    chapter_id: str
    chapter_title: str = "Chapter"
    chapter_content: str


class TranslateResponse(BaseModel):
    """Response with translated chapter content."""
    success: bool
    urdu_content: str
    message: str


async def get_user_from_token(authorization: str = Header(None)) -> dict:
    """Extract user ID from JWT token."""
    if not authorization or not authorization.startswith("Bearer "):
        raise HTTPException(status_code=401, detail="Missing or invalid authorization header")

    token = authorization.replace("Bearer ", "")

    try:
        from src.core.security import verify_token
        user_id = verify_token(token)
        if not user_id:
            raise HTTPException(status_code=401, detail="Invalid token")
        return {"user_id": user_id}
    except Exception as e:
        logger.error(f"Token verification error: {str(e)}")
        raise HTTPException(status_code=401, detail="Invalid token")


@router.post("/chapter-to-urdu", response_model=TranslateResponse)
async def translate_chapter_to_urdu(
    request: TranslateRequest,
    authorization: str = Header(None)
) -> TranslateResponse:
    """
    Translate chapter content to Urdu.

    Authenticated endpoint that translates English textbook chapters to Urdu
    while preserving code blocks and technical terminology.

    Args:
        request: Chapter content and metadata
        authorization: JWT token in Authorization header

    Returns:
        Translated chapter content in Urdu with proper RTL formatting
    """
    try:
        # Verify authentication
        user_info = await get_user_from_token(authorization)
        user_id = user_info["user_id"]

        # Translate the content
        result = await translation_service.translate_to_urdu(
            content=request.chapter_content,
            chapter_title=request.chapter_title,
        )

        return TranslateResponse(
            success=result["success"],
            urdu_content=result["urdu_content"],
            message=result["message"],
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Translation API error: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Translation failed: {str(e)}")
