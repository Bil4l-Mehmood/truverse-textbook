"""
Authentication API endpoints for user registration, login, and profile management.
"""

import logging
from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession

from src.database.postgres import get_db
from src.schemas.auth import (
    SignUpRequest,
    SignInRequest,
    AuthResponse,
    UserProfile,
    UpdateProfileRequest,
    ErrorResponse,
)
from src.services.auth_service import (
    create_user,
    authenticate_user,
    create_access_token,
    update_user_profile,
)
from src.middleware.auth import get_current_user
from src.models.user import User


logger = logging.getLogger(__name__)
router = APIRouter()


@router.post("/signup", response_model=AuthResponse, status_code=status.HTTP_201_CREATED)
async def signup(request: SignUpRequest, db: AsyncSession = Depends(get_db)):
    """
    Register a new user with optional background questionnaire data.

    **Request Body:**
    - email: Valid email address (must be unique)
    - password: Password (minimum 8 characters)
    - name: User's full name
    - background_data: Optional questionnaire responses

    **Response:**
    - access_token: JWT token for authentication
    - token_type: "bearer"
    - user: User profile including background data

    **Errors:**
    - 400 Bad Request: Email already registered
    - 422 Unprocessable Entity: Validation error (invalid email, short password)
    """
    try:
        # Extract background data if provided
        background_data = request.background_data or {}

        # Create user
        user = await create_user(
            db=db,
            email=request.email,
            password=request.password,
            name=request.name,
            ros2_experience=background_data.ros2_experience if request.background_data else "Beginner",
            gpu_model=background_data.gpu_model if request.background_data else None,
            gpu_vram=background_data.gpu_vram if request.background_data else None,
            operating_system=background_data.operating_system if request.background_data else None,
            robotics_knowledge=background_data.robotics_knowledge if request.background_data else "Beginner",
        )

        # Generate JWT token
        access_token = create_access_token(data={"user_id": user.id})

        logger.info(f"New user registered: {user.email} (ID: {user.id})")

        return AuthResponse(
            access_token=access_token,
            token_type="bearer",
            user=UserProfile.from_orm(user),
        )

    except ValueError as e:
        logger.warning(f"Sign-up failed: {str(e)}")
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
    """
    Authenticate a user and return JWT token.

    **Request Body:**
    - email: User's email address
    - password: User's password

    **Response:**
    - access_token: JWT token for authentication
    - token_type: "bearer"
    - user: User profile data

    **Errors:**
    - 401 Unauthorized: Incorrect email or password
    """
    # Authenticate user
    user = await authenticate_user(db, request.email, request.password)

    if not user:
        logger.warning(f"Failed sign-in attempt for email: {request.email}")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect email or password",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Generate JWT token
    access_token = create_access_token(data={"user_id": user.id})

    logger.info(f"User signed in: {user.email} (ID: {user.id})")

    return AuthResponse(
        access_token=access_token,
        token_type="bearer",
        user=UserProfile.from_orm(user),
    )


@router.get("/profile", response_model=UserProfile)
async def get_profile(current_user: User = Depends(get_current_user)):
    """
    Get current user's profile.

    **Authentication Required:** Bearer token in Authorization header

    **Response:**
    - User profile including background questionnaire data

    **Errors:**
    - 401 Unauthorized: Invalid or missing token
    """
    logger.info(f"Profile retrieved for user: {current_user.email} (ID: {current_user.id})")

    return UserProfile.from_orm(current_user)


@router.put("/profile", response_model=UserProfile)
async def update_profile(
    request: UpdateProfileRequest,
    current_user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db),
):
    """
    Update current user's profile background data.

    **Authentication Required:** Bearer token in Authorization header

    **Request Body:**
    - ros2_experience: ROS 2 experience level (optional)
    - gpu_model: GPU model (optional)
    - gpu_vram: GPU VRAM (optional)
    - operating_system: Operating system (optional)
    - robotics_knowledge: Robotics knowledge level (optional)

    **Response:**
    - Updated user profile

    **Errors:**
    - 401 Unauthorized: Invalid or missing token
    """
    try:
        # Update user profile
        updated_user = await update_user_profile(
            db=db,
            user_id=current_user.id,
            ros2_experience=request.ros2_experience,
            gpu_model=request.gpu_model,
            gpu_vram=request.gpu_vram,
            operating_system=request.operating_system,
            robotics_knowledge=request.robotics_knowledge,
        )

        if not updated_user:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="User not found",
            )

        logger.info(f"Profile updated for user: {updated_user.email} (ID: {updated_user.id})")

        return UserProfile.from_orm(updated_user)

    except Exception as e:
        logger.error(f"Profile update error: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while updating profile",
        )
