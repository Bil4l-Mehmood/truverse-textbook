"""Personalization model for caching chapter personalization results."""

from sqlalchemy import Column, String, DateTime, Integer, Text, ForeignKey
from sqlalchemy.sql import func
from src.database.postgres import Base


class PersonalizationPreference(Base):
    """Model for caching personalized chapter content."""

    __tablename__ = "personalization_preferences"

    id = Column(Integer, primary_key=True, index=True)
    user_id = Column(Integer, ForeignKey("users.id"), nullable=False, index=True)
    chapter_id = Column(String, nullable=False, index=True)
    chapter_title = Column(String, nullable=False)
    personalization_level = Column(String, nullable=False)  # beginner/intermediate/advanced
    original_content = Column(Text, nullable=False)
    personalized_content = Column(Text, nullable=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

    def __repr__(self):
        return f"<PersonalizationPreference(id={self.id}, user_id={self.user_id}, chapter_id={self.chapter_id}, level={self.personalization_level})>"
