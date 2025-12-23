"""Pydantic schemas for authentication API."""

from datetime import datetime
from typing import Optional
from pydantic import BaseModel, EmailStr, Field


class SignUpRequest(BaseModel):
    """Sign-up request schema."""
    email: EmailStr = Field(..., description="User email address")
    password: str = Field(..., min_length=8, description="Password (min 8 characters)")
    name: str = Field(..., min_length=1, description="User's full name")
    ros2_experience: Optional[str] = Field(default="Beginner", description="ROS 2 experience level")
    gpu_model: Optional[str] = Field(None, description="GPU model")
    gpu_vram: Optional[str] = Field(None, description="GPU VRAM")
    operating_system: Optional[str] = Field(None, description="Operating system")
    robotics_knowledge: Optional[str] = Field(default="Beginner", description="Robotics knowledge level")


class SignInRequest(BaseModel):
    """Sign-in request schema."""
    email: EmailStr = Field(..., description="User email address")
    password: str = Field(..., description="User password")


class UserProfile(BaseModel):
    """User profile schema."""
    id: int
    email: str
    name: str
    ros2_experience: str
    gpu_model: Optional[str]
    gpu_vram: Optional[str]
    operating_system: Optional[str]
    robotics_knowledge: str
    created_at: datetime

    class Config:
        from_attributes = True


class AuthResponse(BaseModel):
    """Auth response schema with token and user."""
    access_token: str
    token_type: str = "bearer"
    user: UserProfile


class UpdateProfileRequest(BaseModel):
    """Update profile request schema."""
    ros2_experience: Optional[str] = None
    gpu_model: Optional[str] = None
    gpu_vram: Optional[str] = None
    operating_system: Optional[str] = None
    robotics_knowledge: Optional[str] = None
