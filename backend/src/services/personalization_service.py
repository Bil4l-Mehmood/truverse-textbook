"""
Personalization service for adapting chapter content based on user background.
"""

import json
import logging
from typing import Optional
import os

logger = logging.getLogger(__name__)


class PersonalizationService:
    """Service for personalizing textbook content based on user background."""

    def __init__(self, openai_client=None):
        """Initialize personalization service with optional OpenAI client."""
        self.openai_client = openai_client

    async def personalize_content(
        self,
        chapter_content: str,
        user_background: dict,
        chapter_title: str = "Chapter"
    ) -> dict:
        """
        Personalize chapter content based on user background.

        Args:
            chapter_content: Original Markdown chapter content
            user_background: User's background data (ros2_experience, gpu_model, robotics_knowledge, operating_system)
            chapter_title: Title of the chapter for context

        Returns:
            {
                "success": bool,
                "original_content": str,
                "personalized_content": str,
                "level": str (beginner/intermediate/advanced),
                "message": str
            }
        """
        try:
            # Determine personalization level based on ROS 2 experience
            ros2_experience = user_background.get("ros2_experience", "Beginner").lower()
            robotics_knowledge = user_background.get("robotics_knowledge", "Beginner").lower()

            # Map experience to personalization level
            if ros2_experience in ["none", "beginner"] or robotics_knowledge in ["none", "beginner"]:
                personalization_level = "beginner"
                prompt_instruction = (
                    "Expand introductions and foundational concepts. Simplify technical jargon by adding explanations. "
                    "Add beginner-friendly examples. Include helpful warnings about common mistakes. "
                    "Maintain all original sections but make content more accessible."
                )
            elif ros2_experience in ["advanced"] or robotics_knowledge in ["advanced"]:
                personalization_level = "advanced"
                prompt_instruction = (
                    "Condense basic explanations (assume strong fundamentals). Emphasize advanced topics and complex patterns. "
                    "Add optimization tips and performance considerations. Include references to academic papers or advanced resources. "
                    "Skip over-simplified examples."
                )
            else:  # intermediate
                personalization_level = "intermediate"
                prompt_instruction = (
                    "Maintain balanced coverage of fundamentals and advanced topics. "
                    "Add bridges between basic and advanced concepts. Include practical examples. "
                    "Highlight intermediate best practices."
                )

            # Use Claude or fallback to text-based personalization
            from src.core.config import settings

            if settings.openai_api_key:
                # Use OpenAI GPT-4 for personalization
                try:
                    from openai import AsyncOpenAI
                    client = AsyncOpenAI(api_key=settings.openai_api_key)

                    response = await client.chat.completions.create(
                        model="gpt-4",
                        messages=[
                            {
                                "role": "system",
                                "content": (
                                    "You are an expert technical writer specializing in robotics education. "
                                    "Your task is to adapt textbook chapters for different experience levels while preserving all original information. "
                                    "Always return the response as valid Markdown. Preserve code blocks exactly as they are. "
                                    "Do not add or remove sections, only adjust the depth of explanations and add contextual notes where appropriate."
                                ),
                            },
                            {
                                "role": "user",
                                "content": (
                                    f"Personalize the following chapter for a {personalization_level} level student. "
                                    f"Chapter Title: {chapter_title}\n"
                                    f"Personalization Instructions: {prompt_instruction}\n"
                                    f"User Profile: ROS 2 Experience={ros2_experience}, Robotics Knowledge={robotics_knowledge}\n\n"
                                    f"Original Chapter Content:\n{chapter_content}\n\n"
                                    f"Please personalize this content appropriately. Return ONLY the personalized Markdown content, no explanations."
                                ),
                            },
                        ],
                        temperature=0.7,
                        max_tokens=4000,
                    )

                    personalized_content = response.choices[0].message.content

                    logger.info(
                        f"Successfully personalized chapter '{chapter_title}' for {personalization_level} user"
                    )

                    return {
                        "success": True,
                        "original_content": chapter_content,
                        "personalized_content": personalized_content,
                        "level": personalization_level,
                        "message": f"Content personalized for {personalization_level} level",
                    }

                except Exception as e:
                    logger.error(f"OpenAI API error during personalization: {str(e)}")
                    # Fallback to template-based personalization
                    return await self._template_based_personalization(
                        chapter_content, personalization_level, user_background, chapter_title
                    )
            else:
                # Fallback to template-based personalization
                return await self._template_based_personalization(
                    chapter_content, personalization_level, user_background, chapter_title
                )

        except Exception as e:
            logger.error(f"Personalization error: {str(e)}")
            return {
                "success": False,
                "original_content": chapter_content,
                "personalized_content": chapter_content,
                "level": "error",
                "message": f"Personalization failed: {str(e)}. Showing original content.",
            }

    async def _template_based_personalization(
        self,
        chapter_content: str,
        level: str,
        user_background: dict,
        chapter_title: str
    ) -> dict:
        """
        Fallback template-based personalization when OpenAI is unavailable.
        """
        lines = chapter_content.split('\n')
        personalized_lines = []

        # Add personalization hint at the beginning
        if level == "beginner":
            personalized_lines.append("> 💡 **Beginner Mode**: This content has been simplified for your level.\n")
        elif level == "advanced":
            personalized_lines.append("> 🚀 **Advanced Mode**: Content focuses on advanced patterns and optimization.\n")
        else:
            personalized_lines.append("> 📚 **Intermediate Mode**: Balanced coverage of fundamentals and advanced topics.\n")

        personalized_lines.append("")

        # Process content based on level
        in_code_block = False
        skip_next = False

        for i, line in enumerate(lines):
            # Track code blocks
            if line.strip().startswith("```"):
                in_code_block = not in_code_block

            # Don't modify code blocks
            if in_code_block:
                personalized_lines.append(line)
                continue

            # Beginner: Add explanations for technical terms
            if level == "beginner":
                if "ROS 2" in line and "ROS 2 is" not in line:
                    personalized_lines.append(line)
                    personalized_lines.append("> **Note**: ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software.")
                else:
                    personalized_lines.append(line)

            # Advanced: Remove overly simple explanations
            elif level == "advanced":
                # Skip lines that seem to be basic explanations
                if line.strip().startswith(">") and any(
                    word in line.lower() for word in ["simple", "basic", "easy", "beginner"]
                ):
                    continue
                else:
                    personalized_lines.append(line)
            else:
                personalized_lines.append(line)

        personalized_content = '\n'.join(personalized_lines)

        return {
            "success": True,
            "original_content": chapter_content,
            "personalized_content": personalized_content,
            "level": level,
            "message": f"Content personalized for {level} level (template-based)",
        }
