/**
 * Personalization service for interacting with the personalization API
 */

const API_BASE_URL = 'https://truverse-textbook-production.up.railway.app';

export interface PersonalizeResponse {
  success: boolean;
  personalized_content: string;
  level: 'beginner' | 'intermediate' | 'advanced';
  message: string;
}

/**
 * Personalize chapter content based on user's background
 */
export async function personalizeChapter(
  chapterId: string,
  chapterTitle: string,
  chapterContent: string,
  token: string
): Promise<PersonalizeResponse> {
  try {
    const response = await fetch(`${API_BASE_URL}/api/v1/personalize/chapter`, {
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
      throw new Error(error.detail || 'Failed to personalize chapter');
    }

    return await response.json();
  } catch (error) {
    const message = error instanceof Error ? error.message : 'Personalization failed';
    console.error('[PersonalizationService] Error:', message);
    throw new Error(message);
  }
}
