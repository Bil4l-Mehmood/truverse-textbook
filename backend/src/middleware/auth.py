"""
JWT authentication middleware and dependencies for FastAPI.
"""

from typing import Optional
from fastapi import Depends, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from sqlalchemy.ext.asyncio import AsyncSession

from src.services.auth_service import decode_access_token, get_user_by_id
from src.database.postgres import get_db
from src.models.user import User


# Security scheme for JWT bearer token
security = HTTPBearer()


async def get_current_user(
    credentials: HTTPAuthorizationCredentials = Depends(security),
    db: AsyncSession = Depends(get_db),
) -> User:
    """
    Dependency to get the current authenticated user from JWT token.

    Args:
        credentials: HTTP Bearer token credentials
        db: Database session

    Returns:
        Current authenticated User object

    Raises:
        HTTPException: If token is invalid or user not found (401 Unauthorized)
    """
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )

    # Extract token from credentials
    token = credentials.credentials

    # Decode JWT token
    payload = decode_access_token(token)
    if payload is None:
        raise credentials_exception

    # Extract user_id from token payload
    user_id: Optional[int] = payload.get("user_id")
    if user_id is None:
        raise credentials_exception

    # Get user from database
    user = await get_user_by_id(db, user_id)
    if user is None:
        raise credentials_exception

    return user


async def get_optional_current_user(
    credentials: Optional[HTTPAuthorizationCredentials] = Depends(HTTPBearer(auto_error=False)),
    db: AsyncSession = Depends(get_db),
) -> Optional[User]:
    """
    Dependency to optionally get the current authenticated user.

    This is useful for endpoints that can work with or without authentication,
    providing different functionality based on whether the user is logged in.

    Args:
        credentials: Optional HTTP Bearer token credentials
        db: Database session

    Returns:
        Current authenticated User object or None if not authenticated
    """
    if credentials is None:
        return None

    try:
        token = credentials.credentials
        payload = decode_access_token(token)

        if payload is None:
            return None

        user_id: Optional[int] = payload.get("user_id")
        if user_id is None:
            return None

        user = await get_user_by_id(db, user_id)
        return user

    except Exception:
        return None
