"""
API endpoints for content personalization.
"""

import logging
from fastapi import APIRouter, Header, HTTPException
from pydantic import BaseModel
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession
from datetime import datetime, timedelta
import os

from src.services.personalization_service import PersonalizationService
from src.core.config import settings
from src.database.postgres import get_db
from src.models import User, PersonalizationPreference

logger = logging.getLogger(__name__)
router = APIRouter()

personalization_service = PersonalizationService()


class PersonalizeRequest(BaseModel):
    """Request to personalize chapter content."""
    chapter_id: str
    chapter_title: str = "Chapter"
    chapter_content: str


class PersonalizeResponse(BaseModel):
    """Response with personalized chapter content."""
    success: bool
    personalized_content: str
    level: str
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


@router.post("/chapter", response_model=PersonalizeResponse)
async def personalize_chapter(
    request: PersonalizeRequest,
    authorization: str = Header(None)
) -> PersonalizeResponse:
    """
    Personalize chapter content based on user's background.

    Query the authenticated user's background profile, determine their experience level,
    and adapt the chapter content accordingly.

    Args:
        request: Chapter content and metadata
        authorization: JWT token in Authorization header

    Returns:
        Personalized chapter content with metadata
    """
    try:
        # Verify authentication
        user_info = await get_user_from_token(authorization)
        user_id = user_info["user_id"]

        # Get database session
        db = next(await get_db())

        # Fetch user background from database
        try:
            stmt = select(User).where(User.id == user_id)
            result = await db.execute(stmt)
            user = result.scalar_one_or_none()

            if not user:
                raise HTTPException(status_code=404, detail="User not found")

            user_background = {
                "ros2_experience": user.ros2_experience or "Beginner",
                "gpu_model": user.gpu_model,
                "gpu_vram": user.gpu_vram,
                "operating_system": user.operating_system,
                "robotics_knowledge": user.robotics_knowledge or "Beginner",
            }
        except HTTPException:
            raise
        except Exception as e:
            logger.error(f"Error fetching user background: {str(e)}")
            raise HTTPException(status_code=500, detail="Error fetching user background")

        # Check if personalization is cached and fresh (< 7 days old)
        try:
            stmt = select(PersonalizationPreference).where(
                (PersonalizationPreference.user_id == user_id) &
                (PersonalizationPreference.chapter_id == request.chapter_id)
            )
            result = await db.execute(stmt)
            cached = result.scalar_one_or_none()

            if cached:
                # Check if cache is fresh (less than 7 days old)
                cache_age = datetime.utcnow() - cached.created_at.replace(tzinfo=None)
                if cache_age < timedelta(days=7):
                    logger.info(f"Returning cached personalization for user {user_id}, chapter {request.chapter_id}")
                    return PersonalizeResponse(
                        success=True,
                        personalized_content=cached.personalized_content,
                        level=cached.personalization_level,
                        message=f"Content personalized for {cached.personalization_level} level (cached)",
                    )
        except Exception as e:
            logger.warning(f"Error checking cache: {str(e)}")
            # Continue with personalization if cache check fails

        # Personalize the content
        result = await personalization_service.personalize_content(
            chapter_content=request.chapter_content,
            user_background=user_background,
            chapter_title=request.chapter_title,
        )

        # Cache the personalization result in database
        try:
            # Delete old cache entry if it exists
            if cached:
                await db.delete(cached)

            # Create new cache entry
            new_cache = PersonalizationPreference(
                user_id=user_id,
                chapter_id=request.chapter_id,
                chapter_title=request.chapter_title,
                personalization_level=result["level"],
                original_content=request.chapter_content,
                personalized_content=result["personalized_content"],
            )
            db.add(new_cache)
            await db.commit()
            logger.info(f"Cached personalization for user {user_id}, chapter {request.chapter_id}")
        except Exception as e:
            logger.warning(f"Error caching personalization: {str(e)}")
            # Don't fail the request if caching fails
            await db.rollback()

        return PersonalizeResponse(
            success=result["success"],
            personalized_content=result["personalized_content"],
            level=result["level"],
            message=result["message"],
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Personalization API error: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Personalization failed: {str(e)}")
