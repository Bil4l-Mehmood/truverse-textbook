import React, { useState } from 'react';
import { useAuthStore } from '../../store/authStore';
import { translateChapterToUrdu } from '../../services/translationService';
import styles from './translation.module.css';

interface TranslateButtonProps {
  chapterId: string;
  chapterTitle: string;
  chapterContent: string;
  onTranslationComplete?: (urduContent: string) => void;
}

export default function TranslateButton({
  chapterId,
  chapterTitle,
  chapterContent,
  onTranslationComplete,
}: TranslateButtonProps) {
  const { isAuthenticated, token } = useAuthStore();
  const [isLoading, setIsLoading] = useState(false);
  const [showMessage, setShowMessage] = useState<{ type: 'success' | 'error'; text: string } | null>(null);

  if (!isAuthenticated) {
    return null; // Don't show button if not authenticated
  }

  const handleTranslate = async () => {
    if (!token) return;

    setIsLoading(true);
    setShowMessage(null);

    try {
      const result = await translateChapterToUrdu(
        chapterId,
        chapterTitle,
        chapterContent,
        token
      );

      if (result.success) {
        setShowMessage({
          type: 'success',
          text: '✨ Content translated to Urdu',
        });
        if (onTranslationComplete) {
          onTranslationComplete(result.urdu_content);
        }
      } else {
        setShowMessage({
          type: 'error',
          text: result.message || 'Translation failed',
        });
      }
    } catch (error) {
      setShowMessage({
        type: 'error',
        text: `Error: ${error instanceof Error ? error.message : 'Translation failed'}`,
      });
    } finally {
      setIsLoading(false);
      // Auto-hide message after 4 seconds
      setTimeout(() => setShowMessage(null), 4000);
    }
  };

  return (
    <div className={styles.translateContainer}>
      <button
        className={styles.translateBtn}
        onClick={handleTranslate}
        disabled={isLoading}
        title="Translate this chapter to Urdu"
      >
        {isLoading ? (
          <>
            <span className={styles.spinner}></span>
            Translating...
          </>
        ) : (
          <>
            🌐 اردو میں ترجمہ کریں (Translate to Urdu)
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
