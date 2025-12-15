"""User model for authentication and personalization."""

from sqlalchemy import Column, String, DateTime, Integer
from sqlalchemy.sql import func
from src.database.postgres import Base


class User(Base):
    """User model with background questionnaire fields."""

    __tablename__ = "users"

    id = Column(Integer, primary_key=True, index=True)
    email = Column(String, unique=True, index=True, nullable=False)
    name = Column(String, nullable=False)
    hashed_password = Column(String, nullable=False)

    # Background questionnaire fields
    ros2_experience = Column(String, default="Beginner")  # None/Beginner/Intermediate/Advanced
    gpu_model = Column(String, nullable=True)  # e.g., "NVIDIA RTX 3060"
    gpu_vram = Column(String, nullable=True)  # e.g., "12GB"
    operating_system = Column(String, nullable=True)  # Ubuntu/Windows/macOS
    robotics_knowledge = Column(String, default="Beginner")  # None/Beginner/Intermediate/Advanced

    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

    def __repr__(self):
        return f"<User(id={self.id}, email={self.email}, name={self.name})>"
