import React, { useState } from 'react';
import DocPageLayout from '@theme-original/DocPage/Layout';
import type DocPageLayoutType from '@theme/DocPage/Layout';
import type { WrapperProps } from '@docusaurus/types';
import PersonalizeButton from '../../../components/Personalization/PersonalizeButton';
import PersonalizedContent from '../../../components/Personalization/PersonalizedContent';
import { useDoc } from '@docusaurus/theme-common/internal';
import styles from './docPageLayout.module.css';

type Props = WrapperProps<typeof DocPageLayoutType>;

export default function DocPageLayoutWrapper(props: Props) {
  const docContext = useDoc();
  const [personalizedContent, setPersonalizedContent] = useState<string | null>(null);
  const [personalizationLevel, setPersonalizationLevel] = useState<'beginner' | 'intermediate' | 'advanced'>('beginner');

  const originalContent = docContext?.frontMatter?.customData?.markdown || docContext?.content || '';

  const handlePersonalizationComplete = (content: string) => {
    setPersonalizedContent(content);
    // Scroll to content
    setTimeout(() => {
      document.querySelector('.main-wrapper')?.scrollIntoView({ behavior: 'smooth' });
    }, 100);
  };

  return (
    <>
      <DocPageLayout {...props} />

      {/* Inject Personalize Button after the page header */}
      <div className={styles.personalizeButtonContainer}>
        {docContext && (
          <>
            <PersonalizeButton
              chapterId={docContext.metadata?.id || 'unknown'}
              chapterTitle={docContext.metadata?.title || 'Chapter'}
              chapterContent={personalizedContent || originalContent}
              onPersonalizationComplete={(content) => {
                setPersonalizedContent(content);
                setPersonalizationLevel('beginner'); // This will be set based on actual user level
                handlePersonalizationComplete(content);
              }}
            />

            {personalizedContent && (
              <PersonalizedContent
                personalizedContent={personalizedContent}
                originalContent={originalContent}
                level={personalizationLevel}
                onShowOriginal={() => setPersonalizedContent(null)}
              />
            )}
          </>
        )}
      </div>
    </>
  );
}
