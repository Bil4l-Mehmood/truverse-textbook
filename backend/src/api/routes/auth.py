"""Authentication API routes."""

import logging
from fastapi import APIRouter, Depends, HTTPException, status, Header
from sqlalchemy.ext.asyncio import AsyncSession

from src.database.postgres import get_db
from src.schemas.auth import (
    SignUpRequest,
    SignInRequest,
    AuthResponse,
    UserProfile,
    UpdateProfileRequest,
)
from src.services.auth_service import (
    create_user,
    authenticate_user,
    create_access_token,
    get_user_by_id,
    update_user_profile,
    verify_token,
)

logger = logging.getLogger(__name__)
router = APIRouter()


def get_bearer_token(authorization: str = Header(None)) -> str:
    """Extract bearer token from Authorization header."""
    if not authorization or not authorization.startswith("Bearer "):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or missing authorization header",
        )
    return authorization[7:]  # Remove "Bearer " prefix


@router.post("/signup", response_model=AuthResponse, status_code=status.HTTP_201_CREATED)
async def signup(request: SignUpRequest, db: AsyncSession = Depends(get_db)):
    """Sign up a new user with email, password, and optional background data."""
    try:
        # Truncate password to bcrypt max length (72 bytes) - silently handles long passwords
        password = request.password[:72] if len(request.password) > 72 else request.password

        user = await create_user(
            db=db,
            email=request.email,
            password=password,
            name=request.name,
            ros2_experience=request.ros2_experience or "Beginner",
            gpu_model=request.gpu_model,
            gpu_vram=request.gpu_vram,
            operating_system=request.operating_system,
            robotics_knowledge=request.robotics_knowledge or "Beginner",
        )

        access_token = create_access_token(user_id=user.id)
        logger.info(f"User signed up: {user.email}")

        return AuthResponse(
            access_token=access_token,
            token_type="bearer",
            user=UserProfile.model_validate(user),
        )

    except ValueError as e:
        logger.warning(f"Sign-up validation error: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e),
        )
    except Exception as e:
        logger.error(f"Sign-up error: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred during sign-up",
        )


@router.post("/signin", response_model=AuthResponse)
async def signin(request: SignInRequest, db: AsyncSession = Depends(get_db)):
    """Sign in user with email and password."""
    try:
        # Truncate password to bcrypt max length (72 bytes) - silently handles long passwords
        password = request.password[:72] if len(request.password) > 72 else request.password
        user = await authenticate_user(db, request.email, password)

        if not user:
            logger.warning(f"Failed signin attempt for: {request.email}")
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Incorrect email or password",
            )

        access_token = create_access_token(user_id=user.id)
        logger.info(f"User signed in: {user.email}")

        return AuthResponse(
            access_token=access_token,
            token_type="bearer",
            user=UserProfile.model_validate(user),
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Sign-in error: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred during sign-in",
        )


@router.get("/profile", response_model=UserProfile)
async def get_profile(
    token: str = Depends(get_bearer_token),
    db: AsyncSession = Depends(get_db),
):
    """Get current user profile (requires JWT token in Authorization header)."""
    user_id = verify_token(token)

    if not user_id:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired token",
        )

    user = await get_user_by_id(db, user_id)

    if not user:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User not found",
        )

    logger.info(f"Profile retrieved for user: {user.id}")
    return UserProfile.model_validate(user)


@router.put("/profile", response_model=UserProfile)
async def update_profile(
    request: UpdateProfileRequest,
    token: str = Depends(get_bearer_token),
    db: AsyncSession = Depends(get_db),
):
    """Update user profile (requires JWT token in Authorization header)."""
    user_id = verify_token(token)

    if not user_id:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired token",
        )

    try:
        user = await update_user_profile(
            db=db,
            user_id=user_id,
            ros2_experience=request.ros2_experience,
            gpu_model=request.gpu_model,
            gpu_vram=request.gpu_vram,
            operating_system=request.operating_system,
            robotics_knowledge=request.robotics_knowledge,
        )

        if not user:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="User not found",
            )

        logger.info(f"Profile updated for user: {user.id}")
        return UserProfile.model_validate(user)

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Profile update error: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while updating profile",
        )
