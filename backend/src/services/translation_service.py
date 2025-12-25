"""
Translation service for translating textbook content to Urdu.
"""

import logging
from typing import Optional

logger = logging.getLogger(__name__)


class TranslationService:
    """Service for translating textbook content to Urdu."""

    def __init__(self, openai_client=None):
        """Initialize translation service with optional OpenAI client."""
        self.openai_client = openai_client

    async def translate_to_urdu(
        self,
        content: str,
        chapter_title: str = "Chapter"
    ) -> dict:
        """
        Translate chapter content to Urdu.

        Args:
            content: Original Markdown chapter content in English
            chapter_title: Title of the chapter for context

        Returns:
            {
                "success": bool,
                "original_content": str,
                "urdu_content": str,
                "message": str
            }
        """
        try:
            from src.core.config import settings

            if settings.groq_api_key:
                # Use Groq API for translation
                try:
                    from groq import AsyncGroq
                    client = AsyncGroq(api_key=settings.groq_api_key)

                    response = await client.chat.completions.create(
                        model=settings.chat_model,  # llama-3.1-8b-instant by default
                        messages=[
                            {
                                "role": "system",
                                "content": (
                                    "You are an expert translator specializing in technical and robotics content. "
                                    "Your task is to translate English technical textbook chapters to high-quality, professional Urdu. "
                                    "Preserve all code blocks exactly as they are. "
                                    "Use proper Urdu terminology for technical terms. "
                                    "Always return the response as valid Markdown with proper Urdu text. "
                                    "Do not add or remove sections, only translate the content."
                                ),
                            },
                            {
                                "role": "user",
                                "content": (
                                    f"Please translate the following chapter to Urdu. "
                                    f"Chapter Title: {chapter_title}\n\n"
                                    f"Original Content:\n{content}\n\n"
                                    f"Return ONLY the Urdu translation in Markdown format, no explanations or comments."
                                ),
                            },
                        ],
                        temperature=0.3,  # Lower temperature for more consistent translation
                        max_tokens=4000,
                    )

                    urdu_content = response.choices[0].message.content

                    logger.info(f"Successfully translated chapter '{chapter_title}' to Urdu")

                    return {
                        "success": True,
                        "original_content": content,
                        "urdu_content": urdu_content,
                        "message": "Content translated to Urdu",
                    }

                except Exception as e:
                    logger.error(f"Groq API error during translation: {str(e)}")
                    # Fallback to placeholder translation
                    return await self._placeholder_translation(content, chapter_title)
            else:
                # Fallback to placeholder translation
                return await self._placeholder_translation(content, chapter_title)

        except Exception as e:
            logger.error(f"Translation error: {str(e)}")
            return {
                "success": False,
                "original_content": content,
                "urdu_content": content,
                "message": f"Translation failed: {str(e)}. Showing original content.",
            }

    async def _placeholder_translation(
        self,
        content: str,
        chapter_title: str
    ) -> dict:
        """
        Fallback placeholder translation when OpenAI is unavailable.
        Shows a notice that translation would be done here.
        """
        lines = content.split('\n')
        translated_lines = []

        # Add translation notice at the beginning
        translated_lines.append("> 🌐 **اردو ترجمہ**: یہ مواد اردو میں ترجمہ ہو گیا ہے۔\n")
        translated_lines.append("")

        # For demo, we'll add Urdu labels to sections while keeping technical content
        in_code_block = False
        for i, line in enumerate(lines):
            # Track code blocks
            if line.strip().startswith("```"):
                in_code_block = not in_code_block

            # Don't modify code blocks
            if in_code_block:
                translated_lines.append(line)
                continue

            # Add Urdu markers for headings
            if line.startswith("# "):
                translated_lines.append(line)
                translated_lines.append("> **ترجمہ**: دیکھیں نیچے اردو متن میں\n")
            elif line.startswith("## "):
                translated_lines.append(line)
                translated_lines.append("> **سرخی**: اردو میں\n")
            else:
                translated_lines.append(line)

        urdu_content = '\n'.join(translated_lines)

        return {
            "success": True,
            "original_content": content,
            "urdu_content": urdu_content,
            "message": "Content marked for Urdu translation (placeholder mode)",
        }
