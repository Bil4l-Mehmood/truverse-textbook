"""
Authentication service for user registration, login, and JWT token management.
"""

from datetime import datetime, timedelta
from typing import Optional
from jose import JWTError, jwt
from passlib.context import CryptContext
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from src.models.user import User
from src.core.config import settings


# Password hashing context
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")


def hash_password(password: str) -> str:
    """
    Hash a password using bcrypt.
    Automatically truncates to 72 bytes (bcrypt limit) to allow any password length.
    """
    # Truncate to 72 bytes for bcrypt compatibility
    # This allows users to enter passwords of any length
    truncated_password = password[:72] if len(password) > 72 else password
    return pwd_context.hash(truncated_password)


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """
    Verify a password against its hash.
    Automatically truncates to 72 bytes to match hash_password behavior.
    """
    # Truncate to 72 bytes to match what was hashed during signup
    truncated_password = plain_password[:72] if len(plain_password) > 72 else plain_password
    return pwd_context.verify(truncated_password, hashed_password)


def create_access_token(data: dict, expires_delta: Optional[timedelta] = None) -> str:
    """
    Create a JWT access token.

    Args:
        data: Dictionary containing claims to encode in the token
        expires_delta: Optional expiration time delta, defaults to 1 hour

    Returns:
        Encoded JWT token string
    """
    to_encode = data.copy()

    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(hours=1)

    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, settings.jwt_secret, algorithm=settings.jwt_algorithm)

    return encoded_jwt


def decode_access_token(token: str) -> Optional[dict]:
    """
    Decode and verify a JWT token.

    Args:
        token: JWT token string

    Returns:
        Decoded token payload or None if invalid
    """
    try:
        payload = jwt.decode(token, settings.jwt_secret, algorithms=[settings.jwt_algorithm])
        return payload
    except JWTError:
        return None


async def create_user(
    db: AsyncSession,
    email: str,
    password: str,
    name: str,
    ros2_experience: str = "Beginner",
    gpu_model: Optional[str] = None,
    gpu_vram: Optional[str] = None,
    operating_system: Optional[str] = None,
    robotics_knowledge: str = "Beginner",
) -> User:
    """
    Create a new user with hashed password and optional background data.

    Args:
        db: Database session
        email: User email (must be unique)
        password: Plain text password (will be hashed)
        name: User's full name
        ros2_experience: ROS 2 experience level
        gpu_model: GPU model (e.g., "NVIDIA RTX 3060")
        gpu_vram: GPU VRAM (e.g., "12GB")
        operating_system: Operating system (Ubuntu/Windows/macOS)
        robotics_knowledge: Robotics knowledge level

    Returns:
        Created User object

    Raises:
        ValueError: If email already exists
    """
    # Check if user already exists
    result = await db.execute(select(User).where(User.email == email))
    existing_user = result.scalar_one_or_none()

    if existing_user:
        raise ValueError(f"User with email {email} already exists")

    # Create new user with hashed password
    hashed_password = hash_password(password)

    new_user = User(
        email=email,
        name=name,
        hashed_password=hashed_password,
        ros2_experience=ros2_experience,
        gpu_model=gpu_model,
        gpu_vram=gpu_vram,
        operating_system=operating_system,
        robotics_knowledge=robotics_knowledge,
    )

    db.add(new_user)
    await db.commit()
    await db.refresh(new_user)

    return new_user


async def authenticate_user(db: AsyncSession, email: str, password: str) -> Optional[User]:
    """
    Authenticate a user by email and password.

    Args:
        db: Database session
        email: User email
        password: Plain text password

    Returns:
        User object if authentication successful, None otherwise
    """
    result = await db.execute(select(User).where(User.email == email))
    user = result.scalar_one_or_none()

    if not user:
        return None

    if not verify_password(password, user.hashed_password):
        return None

    return user


async def get_user_by_id(db: AsyncSession, user_id: int) -> Optional[User]:
    """
    Get a user by their ID.

    Args:
        db: Database session
        user_id: User ID

    Returns:
        User object or None if not found
    """
    result = await db.execute(select(User).where(User.id == user_id))
    return result.scalar_one_or_none()


async def update_user_profile(
    db: AsyncSession,
    user_id: int,
    ros2_experience: Optional[str] = None,
    gpu_model: Optional[str] = None,
    gpu_vram: Optional[str] = None,
    operating_system: Optional[str] = None,
    robotics_knowledge: Optional[str] = None,
) -> Optional[User]:
    """
    Update user's background profile data.

    Args:
        db: Database session
        user_id: User ID
        ros2_experience: ROS 2 experience level
        gpu_model: GPU model
        gpu_vram: GPU VRAM
        operating_system: Operating system
        robotics_knowledge: Robotics knowledge level

    Returns:
        Updated User object or None if user not found
    """
    user = await get_user_by_id(db, user_id)

    if not user:
        return None

    # Update only provided fields
    if ros2_experience is not None:
        user.ros2_experience = ros2_experience
    if gpu_model is not None:
        user.gpu_model = gpu_model
    if gpu_vram is not None:
        user.gpu_vram = gpu_vram
    if operating_system is not None:
        user.operating_system = operating_system
    if robotics_knowledge is not None:
        user.robotics_knowledge = robotics_knowledge

    await db.commit()
    await db.refresh(user)

    return user
