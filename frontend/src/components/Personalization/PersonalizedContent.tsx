import React, { useState } from 'react';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';
import { Prism as SyntaxHighlighter } from 'react-syntax-highlighter';
import { dracula } from 'react-syntax-highlighter/dist/esm/styles/prism';
import styles from './personalization.module.css';

interface PersonalizedContentProps {
  personalizedContent: string;
  originalContent: string;
  level: 'beginner' | 'intermediate' | 'advanced';
  onShowOriginal: () => void;
}

export default function PersonalizedContent({
  personalizedContent,
  originalContent,
  level,
  onShowOriginal,
}: PersonalizedContentProps) {
  const [isShowingOriginal, setIsShowingOriginal] = useState(false);

  const handleToggleOriginal = () => {
    setIsShowingOriginal(!isShowingOriginal);
  };

  const contentToDisplay = isShowingOriginal ? originalContent : personalizedContent;
  const levelLabel = level.charAt(0).toUpperCase() + level.slice(1);

  return (
    <div className={styles.personalizedContentContainer}>
      <div className={styles.contentHeader}>
        <div className={styles.badge}>
          ✨ Personalized for <strong>{levelLabel}</strong> Level
        </div>
        <button
          className={styles.toggleBtn}
          onClick={handleToggleOriginal}
          title={isShowingOriginal ? 'Show personalized content' : 'Show original content'}
        >
          {isShowingOriginal ? '👁️ Show Personalized' : '📖 Show Original'}
        </button>
      </div>

      <div className={styles.content}>
        <ReactMarkdown
          remarkPlugins={[remarkGfm]}
          components={{
            code({ inline, className, children, ...props }: any) {
              const match = /language-(\w+)/.exec(className || '');
              return !inline && match ? (
                <SyntaxHighlighter
                  style={dracula}
                  language={match[1]}
                  PreTag="div"
                  {...props}
                >
                  {String(children).replace(/\n$/, '')}
                </SyntaxHighlighter>
              ) : (
                <code className={className} {...props}>
                  {children}
                </code>
              );
            },
            a({ node, ...props }: any) {
              return <a {...props} />;
            },
            h1({ node, ...props }: any) {
              return <h1 className={styles.heading1} {...props} />;
            },
            h2({ node, ...props }: any) {
              return <h2 className={styles.heading2} {...props} />;
            },
            h3({ node, ...props }: any) {
              return <h3 className={styles.heading3} {...props} />;
            },
            blockquote({ node, ...props }: any) {
              return <blockquote className={styles.blockquote} {...props} />;
            },
          }}
        >
          {contentToDisplay}
        </ReactMarkdown>
      </div>
    </div>
  );
}
