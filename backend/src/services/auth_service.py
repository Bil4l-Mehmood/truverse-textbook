"""Authentication service with bcrypt password hashing and JWT tokens."""

from datetime import datetime, timedelta
from typing import Optional
from passlib.context import CryptContext
from jose import JWTError, jwt
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from src.models.user import User
from src.core.config import settings

# Password hashing context - using bcrypt
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")


def hash_password(password: str) -> str:
    """Hash a password using bcrypt. Password will be truncated to 72 bytes."""
    # Truncate password to 72 bytes (bcrypt limit)
    truncated = password[:72]
    return pwd_context.hash(truncated)


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """Verify a password against its hash."""
    # Truncate to 72 bytes to match hashing
    truncated = plain_password[:72]
    try:
        return pwd_context.verify(truncated, hashed_password)
    except Exception:
        return False


def create_access_token(user_id: int, expires_delta: Optional[timedelta] = None) -> str:
    """Create JWT access token."""
    if expires_delta is None:
        expires_delta = timedelta(hours=settings.jwt_expiration_hours)

    expire = datetime.utcnow() + expires_delta
    to_encode = {"user_id": user_id, "exp": expire}

    encoded_jwt = jwt.encode(
        to_encode,
        settings.jwt_secret,
        algorithm=settings.jwt_algorithm
    )
    return encoded_jwt


def verify_token(token: str) -> Optional[int]:
    """Verify JWT token and return user_id if valid."""
    try:
        payload = jwt.decode(
            token,
            settings.jwt_secret,
            algorithms=[settings.jwt_algorithm]
        )
        user_id: int = payload.get("user_id")
        if user_id is None:
            return None
        return user_id
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
    """Create a new user with hashed password."""
    # Check if user already exists
    result = await db.execute(select(User).where(User.email == email))
    existing_user = result.scalar_one_or_none()

    if existing_user:
        raise ValueError(f"User with email {email} already exists")

    # Create new user
    hashed_password = hash_password(password)
    user = User(
        email=email,
        name=name,
        hashed_password=hashed_password,
        ros2_experience=ros2_experience,
        gpu_model=gpu_model,
        gpu_vram=gpu_vram,
        operating_system=operating_system,
        robotics_knowledge=robotics_knowledge,
    )

    db.add(user)
    await db.commit()
    await db.refresh(user)

    return user


async def authenticate_user(db: AsyncSession, email: str, password: str) -> Optional[User]:
    """Authenticate user by email and password."""
    result = await db.execute(select(User).where(User.email == email))
    user = result.scalar_one_or_none()

    if not user:
        return None

    if not verify_password(password, user.hashed_password):
        return None

    return user


async def get_user_by_id(db: AsyncSession, user_id: int) -> Optional[User]:
    """Get user by ID."""
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
    """Update user profile."""
    user = await get_user_by_id(db, user_id)

    if not user:
        return None

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
