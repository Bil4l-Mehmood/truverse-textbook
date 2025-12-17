"""
Pydantic schemas for authentication API requests and responses.
"""

from datetime import datetime
from typing import Optional
from pydantic import BaseModel, EmailStr, Field


class BackgroundData(BaseModel):
    """Background questionnaire data."""

    ros2_experience: str = Field(
        default="Beginner",
        description="ROS 2 experience level: None/Beginner/Intermediate/Advanced",
    )
    gpu_model: Optional[str] = Field(
        default=None,
        description="GPU model (e.g., 'NVIDIA RTX 3060')",
    )
    gpu_vram: Optional[str] = Field(
        default=None,
        description="GPU VRAM (e.g., '12GB')",
    )
    operating_system: Optional[str] = Field(
        default=None,
        description="Operating system: Ubuntu/Windows/macOS",
    )
    robotics_knowledge: str = Field(
        default="Beginner",
        description="Robotics knowledge level: None/Beginner/Intermediate/Advanced",
    )


class SignUpRequest(BaseModel):
    """Request schema for user registration."""

    email: EmailStr = Field(..., description="User email address")
    password: str = Field(..., min_length=8, description="Password (min 8 characters)")
    name: str = Field(..., min_length=1, description="User's full name")
    background_data: Optional[BackgroundData] = Field(
        default=None,
        description="Optional background questionnaire data",
    )


class SignInRequest(BaseModel):
    """Request schema for user login."""

    email: EmailStr = Field(..., description="User email address")
    password: str = Field(..., description="User password")


class UserProfile(BaseModel):
    """User profile response schema."""

    id: int = Field(..., description="User ID")
    email: str = Field(..., description="User email")
    name: str = Field(..., description="User's full name")
    ros2_experience: str = Field(..., description="ROS 2 experience level")
    gpu_model: Optional[str] = Field(None, description="GPU model")
    gpu_vram: Optional[str] = Field(None, description="GPU VRAM")
    operating_system: Optional[str] = Field(None, description="Operating system")
    robotics_knowledge: str = Field(..., description="Robotics knowledge level")
    created_at: datetime = Field(..., description="Account creation timestamp")

    class Config:
        from_attributes = True


class AuthResponse(BaseModel):
    """Response schema for successful authentication."""

    access_token: str = Field(..., description="JWT access token")
    token_type: str = Field(default="bearer", description="Token type")
    user: UserProfile = Field(..., description="User profile data")


class UpdateProfileRequest(BaseModel):
    """Request schema for updating user profile."""

    ros2_experience: Optional[str] = Field(
        None,
        description="ROS 2 experience level: None/Beginner/Intermediate/Advanced",
    )
    gpu_model: Optional[str] = Field(None, description="GPU model")
    gpu_vram: Optional[str] = Field(None, description="GPU VRAM")
    operating_system: Optional[str] = Field(
        None,
        description="Operating system: Ubuntu/Windows/macOS",
    )
    robotics_knowledge: Optional[str] = Field(
        None,
        description="Robotics knowledge level: None/Beginner/Intermediate/Advanced",
    )


class ErrorResponse(BaseModel):
    """Error response schema."""

    detail: str = Field(..., description="Error message")
    code: Optional[str] = Field(None, description="Error code")
