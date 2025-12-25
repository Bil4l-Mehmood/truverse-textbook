/**
 * Translation service for interacting with the translation API
 */

const API_BASE_URL = typeof window !== 'undefined' && window.location.hostname === 'localhost'
  ? 'http://localhost:8000'
  : `${window.location.protocol}//${window.location.host}`;

export interface TranslateResponse {
  success: boolean;
  urdu_content: string;
  message: string;
}

/**
 * Translate chapter content to Urdu
 */
export async function translateChapterToUrdu(
  chapterId: string,
  chapterTitle: string,
  chapterContent: string,
  token: string
): Promise<TranslateResponse> {
  try {
    const response = await fetch(`${API_BASE_URL}/api/v1/translate/chapter-to-urdu`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${token}`,
      },
      body: JSON.stringify({
        chapter_id: chapterId,
        chapter_title: chapterTitle,
        chapter_content: chapterContent,
      }),
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.detail || 'Failed to translate chapter');
    }

    return await response.json();
  } catch (error) {
    const message = error instanceof Error ? error.message : 'Translation failed';
    console.error('[TranslationService] Error:', message);
    throw new Error(message);
  }
}
