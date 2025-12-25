import React, { useState, useEffect } from 'react';
import DocItem from '@theme-original/DocItem';
import type DocItemType from '@theme/DocItem';
import type { WrapperProps } from '@docusaurus/types';
import PersonalizeButton from '../../components/Personalization/PersonalizeButton';
import PersonalizedContent from '../../components/Personalization/PersonalizedContent';
import TranslateButton from '../../components/Translation/TranslateButton';
import UrduContent from '../../components/Translation/UrduContent';
import styles from './docItem.module.css';

type Props = WrapperProps<typeof DocItemType>;

export default function DocItemWrapper(props: Props) {
  const [personalizedContent, setPersonalizedContent] = useState<string | null>(null);
  const [personalizationLevel, setPersonalizationLevel] = useState<'beginner' | 'intermediate' | 'advanced'>('beginner');
  const [showPersonalized, setShowPersonalized] = useState(false);
  const [urduContent, setUrduContent] = useState<string | null>(null);
  const [showUrdu, setShowUrdu] = useState(false);
  const [docMetadata, setDocMetadata] = useState({ id: 'unknown', title: 'Chapter', content: '' });

  // Extract document metadata from the DOM on mount
  useEffect(() => {
    // Only run on client-side
    if (typeof window === 'undefined' || typeof document === 'undefined') {
      return;
    }

    try {
      // Extract chapter title from h1 or page title
      const titleElement = document.querySelector('h1') || document.querySelector('[class*="title"]');
      const title = titleElement?.textContent || 'Chapter';

      // Extract chapter ID from URL or meta tag
      const pathname = window.location.pathname;
      const id = pathname.split('/').filter(Boolean).pop() || 'unknown';

      // Extract content from article body
      const contentDiv = document.querySelector('[itemprop="articleBody"]') || document.querySelector('article') || document.querySelector('main');
      const content = contentDiv?.innerText || '';

      setDocMetadata({ id, title, content });
    } catch (error) {
      console.error('Error extracting document metadata:', error);
    }
  }, []);

  const originalContent = docMetadata.content;
  const chapterId = docMetadata.id;
  const chapterTitle = docMetadata.title;

  const handlePersonalizationComplete = (content: string, level: 'beginner' | 'intermediate' | 'advanced') => {
    setPersonalizedContent(content);
    setPersonalizationLevel(level);
    setShowPersonalized(true);

    // Scroll to the personalized content
    setTimeout(() => {
      const element = document.querySelector(`[data-personalized-content]`);
      if (element) {
        element.scrollIntoView({ behavior: 'smooth', block: 'start' });
      }
    }, 100);
  };

  if (showPersonalized && personalizedContent) {
    // Show personalized content instead of original
    return (
      <div className={styles.docItemWrapper}>
        <PersonalizedContent
          personalizedContent={personalizedContent}
          originalContent={originalContent}
          level={personalizationLevel}
          onShowOriginal={() => setShowPersonalized(false)}
        />

        {/* Translation Button */}
        {chapterId !== 'unknown' && (
          <div className={styles.translateButtonWrapper}>
            <TranslateButton
              chapterId={chapterId}
              chapterTitle={chapterTitle}
              chapterContent={personalizedContent || originalContent || ''}
              onTranslationComplete={(content) => {
                setUrduContent(content);
                setShowUrdu(true);
              }}
            />
          </div>
        )}

        {/* Urdu Content Display */}
        {showUrdu && urduContent && (
          <UrduContent
            urduContent={urduContent}
            originalContent={personalizedContent || originalContent}
            onShowOriginal={() => setShowUrdu(false)}
          />
        )}
      </div>
    );
  }

  if (showUrdu && urduContent) {
    // Show Urdu content instead of original
    return (
      <div className={styles.docItemWrapper}>
        <UrduContent
          urduContent={urduContent}
          originalContent={originalContent}
          onShowOriginal={() => setShowUrdu(false)}
        />
      </div>
    );
  }

  return (
    <div className={styles.docItemWrapper}>
      <DocItem {...props} />

      {/* Personalize Button injected after content */}
      <div className={styles.personalizeButtonWrapper} data-personalized-content>
        {chapterId !== 'unknown' && (
          <PersonalizeButton
            chapterId={chapterId}
            chapterTitle={chapterTitle}
            chapterContent={originalContent || ''}
            onPersonalizationComplete={(content, level) => {
              // The button component handles the API call and returns personalized content with level
              handlePersonalizationComplete(content, level || 'beginner');
            }}
          />
        )}
      </div>

      {/* Translation Button */}
      <div className={styles.translateButtonWrapper} data-translate-content>
        {chapterId !== 'unknown' && (
          <TranslateButton
            chapterId={chapterId}
            chapterTitle={chapterTitle}
            chapterContent={originalContent || ''}
            onTranslationComplete={(content) => {
              setUrduContent(content);
              setShowUrdu(true);
              // Scroll to the translated content
              setTimeout(() => {
                const element = document.querySelector(`[data-translate-content]`);
                if (element) {
                  element.scrollIntoView({ behavior: 'smooth', block: 'start' });
                }
              }, 100);
            }}
          />
        )}
      </div>

      {/* Urdu Content Display */}
      {showUrdu && urduContent && (
        <UrduContent
          urduContent={urduContent}
          originalContent={originalContent}
          onShowOriginal={() => setShowUrdu(false)}
        />
      )}
    </div>
  );
}
