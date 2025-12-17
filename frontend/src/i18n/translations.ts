/**
 * Translations for frontend UI components
 * English and Urdu support
 */

export type Language = 'en' | 'ur';

export interface Translations {
  // Auth
  signIn: string;
  signUp: string;
  signOut: string;
  email: string;
  password: string;
  fullName: string;
  continueButton: string;
  alreadyHaveAccount: string;
  dontHaveAccount: string;

  // Personalization
  personalizeContent: string;
  personalizeYourLearning: string;
  ros2Experience: string;
  roboticsKnowledge: string;
  operatingSystem: string;
  gpuOptimizations: string;
  applyPersonalization: string;
  cancel: string;
  skipForNow: string;
  back: string;

  // Levels
  none: string;
  beginner: string;
  intermediate: string;
  advanced: string;

  // OS
  ubuntu: string;
  windows: string;
  macos: string;
}

const en: Translations = {
  signIn: 'Sign In',
  signUp: 'Sign Up',
  signOut: 'Sign Out',
  email: 'Email',
  password: 'Password',
  fullName: 'Full Name',
  continueButton: 'Continue',
  alreadyHaveAccount: 'Already have an account?',
  dontHaveAccount: "Don't have an account?",

  personalizeContent: '🎯 Personalize Content',
  personalizeYourLearning: '📚 Personalize Your Learning',
  ros2Experience: 'ROS 2 Experience Level',
  roboticsKnowledge: 'Robotics Knowledge',
  operatingSystem: 'Operating System',
  gpuOptimizations: 'Show GPU optimization examples',
  applyPersonalization: '✨ Apply Personalization',
  cancel: 'Cancel',
  skipForNow: 'Skip for Now',
  back: '← Back',

  none: 'None',
  beginner: 'Beginner',
  intermediate: 'Intermediate',
  advanced: 'Advanced',

  ubuntu: 'Ubuntu / Linux',
  windows: 'Windows',
  macos: 'macOS',
};

const ur: Translations = {
  signIn: 'سائن ان کریں',
  signUp: 'سائن اپ کریں',
  signOut: 'سائن آؤٹ کریں',
  email: 'ای میل',
  password: 'پاس ورڈ',
  fullName: 'مکمل نام',
  continueButton: 'جاری رکھیں',
  alreadyHaveAccount: 'پہلے سے اکاؤنٹ ہے؟',
  dontHaveAccount: 'اکاؤنٹ نہیں ہے؟',

  personalizeContent: '🎯 مواد کو ذاتی نوعیت دیں',
  personalizeYourLearning: '📚 اپنی سیکھ کو ذاتی نوعیت دیں',
  ros2Experience: 'ROS 2 تجربہ کی سطح',
  roboticsKnowledge: 'روبوٹکس کی علم کی سطح',
  operatingSystem: 'آپریٹنگ سسٹم',
  gpuOptimizations: 'GPU بہتری کی مثالیں دکھائیں',
  applyPersonalization: '✨ ذاتی نوعیت کو لاگو کریں',
  cancel: 'منسوخ کریں',
  skipForNow: 'ابھی کے لیے چھوڑیں',
  back: '← واپس',

  none: 'کوئی نہیں',
  beginner: 'ابتدائی',
  intermediate: 'درمیانی',
  advanced: 'اعلیٰ',

  ubuntu: 'Ubuntu / لینکس',
  windows: 'ونڈوز',
  macos: 'macOS',
};

export const translations: Record<Language, Translations> = {
  en,
  ur,
};

/**
 * Get current language from URL or localStorage
 */
export function getCurrentLanguage(): Language {
  // Check URL path
  if (typeof window !== 'undefined') {
    const path = window.location.pathname;
    if (path.startsWith('/ur')) {
      return 'ur';
    }
    // Check localStorage
    const stored = localStorage.getItem('language');
    if (stored === 'ur' || stored === 'en') {
      return stored;
    }
  }
  return 'en';
}

/**
 * Get translations for current language
 */
export function getTranslations(): Translations {
  const lang = getCurrentLanguage();
  return translations[lang];
}
