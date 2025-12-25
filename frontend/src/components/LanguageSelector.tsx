/**
 * Language Selector Component
 * Allows users to switch between English and Urdu
 */

import React, { useState } from 'react';
import { getCurrentLanguage } from '../i18n/translations';

type Language = 'en' | 'ur';

export default function LanguageSelector() {
  const [currentLang, setCurrentLang] = useState<Language>(getCurrentLanguage());

  const handleLanguageChange = (lang: Language) => {
    setCurrentLang(lang);

    // Only proceed if window is available (client-side)
    if (typeof window === 'undefined') {
      return;
    }

    localStorage.setItem('language', lang);

    // Navigate to the appropriate language path
    const pathname = window.location.pathname;
    let newPath = pathname;

    if (lang === 'ur') {
      // Add /ur prefix if not already there
      if (!pathname.startsWith('/ur')) {
        newPath = '/ur' + (pathname === '/' ? '' : pathname);
      }
    } else {
      // Remove /ur prefix
      if (pathname.startsWith('/ur/')) {
        newPath = pathname.substring(2);
      } else if (pathname === '/ur') {
        newPath = '/';
      }
    }

    // Navigate to new path
    window.location.href = newPath;
  };

  return (
    <div style={{ display: 'flex', gap: '8px', marginLeft: '16px' }}>
      <button
        onClick={() => handleLanguageChange('en')}
        style={{
          padding: '4px 12px',
          backgroundColor: currentLang === 'en' ? '#667eea' : 'transparent',
          color: currentLang === 'en' ? 'white' : '#667eea',
          border: `1px solid #667eea`,
          borderRadius: '4px',
          cursor: 'pointer',
          fontSize: '12px',
          fontWeight: currentLang === 'en' ? '600' : '400',
        }}
        title="Switch to English"
      >
        English
      </button>
      <button
        onClick={() => handleLanguageChange('ur')}
        style={{
          padding: '4px 12px',
          backgroundColor: currentLang === 'ur' ? '#667eea' : 'transparent',
          color: currentLang === 'ur' ? 'white' : '#667eea',
          border: `1px solid #667eea`,
          borderRadius: '4px',
          cursor: 'pointer',
          fontSize: '12px',
          fontWeight: currentLang === 'ur' ? '600' : '400',
        }}
        title="اردو میں تبدیل کریں"
      >
        اردو
      </button>
    </div>
  );
}
