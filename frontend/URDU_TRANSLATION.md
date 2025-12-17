# Urdu Translation Implementation Guide

## Overview

The Physical AI & Humanoid Robotics Textbook now includes full Urdu (اردو) language support with right-to-left (RTL) text rendering.

## Features

✅ **Complete Language Support**
- English (English): LTR layout
- Urdu (اردو): RTL layout with proper text direction

✅ **Translated Content**
- Navbar items
- Footer content
- Common UI elements
- Main introduction page
- Sidebar labels

✅ **RTL Styling**
- Automatic direction switching based on language
- Proper text alignment
- Sidebar repositioning for RTL
- Code blocks remain LTR for readability

✅ **Language Persistence**
- Selected language saved to localStorage
- URL-based language routing (/ur/ for Urdu pages)

## File Structure

```
frontend/
├── i18n/
│   └── ur/
│       ├── docusaurus-theme-classic/
│       │   ├── navbar.json          # Navigation translations
│       │   ├── footer.json          # Footer translations
│       │   └── common.json          # Common UI translations
│       └── docusaurus-plugin-content-docs/
│           └── current/
│               ├── intro.md          # Translated intro page
│               └── sidebar.js        # Sidebar translations
├── src/
│   ├── components/
│   │   ├── LanguageSelector.tsx    # Language switcher component
│   │   └── Content/
│   │       └── PersonalizationButton.tsx  # Urdu-aware
│   ├── i18n/
│   │   └── translations.ts         # Frontend translation dictionary
│   └── css/
│       └── rtl.css                 # RTL styling for Urdu
└── docusaurus.config.ts            # Updated with i18n config
```

## How to Use

### For Users

1. **Switch Language**: Use the language selector in the navbar (English / اردو)
2. **Automatic Routing**: Site automatically navigates to /ur/ for Urdu content
3. **Persistent Selection**: Language preference is saved to localStorage

### For Developers

#### Adding Urdu Translation to New Pages

1. Create translated page in `i18n/ur/docusaurus-plugin-content-docs/current/`
2. Add entry to navbar.json or footer.json as needed
3. Import RTL CSS is automatic

#### Adding Urdu Translation to React Components

```typescript
import { getTranslations } from '../i18n/translations';

export function MyComponent() {
  const t = getTranslations();

  return (
    <div>
      <h1>{t.personalizeYourLearning}</h1>
      <button>{t.applyPersonalization}</button>
    </div>
  );
}
```

#### Adding New Translations

1. Add entry to `src/i18n/translations.ts` in both `en` and `ur` objects
2. Use in components with `getTranslations()`

## Translation Keys Available

### Auth
- `signIn`, `signUp`, `signOut`
- `email`, `password`, `fullName`
- `continueButton`, `alreadyHaveAccount`, `dontHaveAccount`

### Personalization
- `personalizeContent`, `personalizeYourLearning`
- `ros2Experience`, `roboticsKnowledge`, `operatingSystem`, `gpuOptimizations`
- `applyPersonalization`, `cancel`, `skipForNow`, `back`

### Levels & Options
- `none`, `beginner`, `intermediate`, `advanced`
- `ubuntu`, `windows`, `macos`

## RTL Styling

RTL styles are automatically applied when viewing Urdu content. The system handles:

- Text direction and alignment
- List and table orientation
- Form element positioning
- Sidebar repositioning
- Code blocks remain LTR
- Navigation and pagination flow

## Language Detection

The site detects language preference in this order:
1. URL path (`/ur/` prefix)
2. localStorage `language` key
3. Defaults to English

## Future Enhancements

- [ ] Add more language locales (Arabic, Persian, etc.)
- [ ] Implement dynamic translation loading
- [ ] Add community translation workflows
- [ ] RTL support for custom components
- [ ] Language selector in mobile navbar

## Deployment

Urdu translations are automatically deployed with the site. No additional configuration needed for Vercel.

## Testing

To test Urdu translation locally:

```bash
cd frontend
npm start

# Navigate to http://localhost:3000/ur/
# or click the اردو button in the navbar
```

## Notes

- All Urdu content is manually translated for accuracy
- Code examples remain in English for clarity
- RTL rendering works in modern browsers
- Mobile responsive RTL layouts included
