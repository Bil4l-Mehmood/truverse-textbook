import React, { useState } from 'react';
import { useAuthStore } from '../../store/authStore';
import { personalizeChapter } from '../../services/personalizationService';
import styles from './personalization.module.css';

interface PersonalizeButtonProps {
  chapterId: string;
  chapterTitle: string;
  chapterContent: string;
  onPersonalizationComplete?: (personalizedContent: string, level?: 'beginner' | 'intermediate' | 'advanced') => void;
}

export default function PersonalizeButton({
  chapterId,
  chapterTitle,
  chapterContent,
  onPersonalizationComplete,
}: PersonalizeButtonProps) {
  const { isAuthenticated, token } = useAuthStore();
  const [isLoading, setIsLoading] = useState(false);
  const [showMessage, setShowMessage] = useState<{ type: 'success' | 'error'; text: string } | null>(null);

  if (!isAuthenticated) {
    return null; // Don't show button if not authenticated
  }

  const handlePersonalize = async () => {
    if (!token) return;

    setIsLoading(true);
    setShowMessage(null);

    try {
      const result = await personalizeChapter(
        chapterId,
        chapterTitle,
        chapterContent,
        token
      );

      if (result.success) {
        setShowMessage({
          type: 'success',
          text: `✨ Content personalized for your ${result.level} level`,
        });
        if (onPersonalizationComplete) {
          onPersonalizationComplete(result.personalized_content, result.level);
        }
      } else {
        setShowMessage({
          type: 'error',
          text: result.message || 'Personalization failed',
        });
      }
    } catch (error) {
      setShowMessage({
        type: 'error',
        text: `Error: ${error instanceof Error ? error.message : 'Personalization failed'}`,
      });
    } finally {
      setIsLoading(false);
      // Auto-hide message after 4 seconds
      setTimeout(() => setShowMessage(null), 4000);
    }
  };

  return (
    <div className={styles.personalizeContainer}>
      <button
        className={styles.personalizeBtn}
        onClick={handlePersonalize}
        disabled={isLoading}
        title="Personalize this chapter based on your background"
      >
        {isLoading ? (
          <>
            <span className={styles.spinner}></span>
            Personalizing...
          </>
        ) : (
          <>
            🎯 Personalize This Chapter
          </>
        )}
      </button>

      {showMessage && (
        <div className={`${styles.message} ${styles[showMessage.type]}`}>
          {showMessage.text}
        </div>
      )}
    </div>
  );
}
