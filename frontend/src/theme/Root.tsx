import React, { useEffect, useState } from 'react';
import { useLocation } from '@docusaurus/router';
import ChatWidget from '../../components/ChatWidget';
import { useAuthStore } from '../../store/authStore';
import AuthGuard from '../../components/Auth/AuthGuard';
import '../../css/auth-nav.css';

// Root theme component to inject ChatWidget and Auth links globally
export default function Root({ children }: { children: React.ReactNode }) {
  const { isAuthenticated, user, logout } = useAuthStore();
  const location = useLocation();
  const [isDocsRoute, setIsDocsRoute] = useState(false);

  // Check if current route requires authentication (docs/course content)
  useEffect(() => {
    const pathname = location.pathname;
    const requiresAuth = pathname.startsWith('/docs');
    setIsDocsRoute(requiresAuth);

    console.log('[Root] Route check:', {
      pathname,
      requiresAuth,
      isAuthenticated,
    });
  }, [location.pathname, isAuthenticated]);

  // Wrap children with AuthGuard if on docs route
  const content = isDocsRoute ? (
    <AuthGuard>{children}</AuthGuard>
  ) : (
    children
  );

  return (
    <>
      {content}
      <ChatWidget />

      {/* Auth navigation links - positioned in navbar via CSS */}
      <div id="auth-nav-links" className="auth-nav-container">
        {!isAuthenticated ? (
          <>
            <a href="/signin" className="navbar__link">
              Sign In
            </a>
            <a href="/signup" className="navbar__link navbar__link--active">
              Sign Up
            </a>
          </>
        ) : (
          <>
            <a href="/profile" className="navbar__link">
              {user?.name || 'Profile'}
            </a>
            <button
              onClick={() => {
                logout();
                window.location.href = '/';
              }}
              className="navbar__link auth-logout-btn"
            >
              Sign Out
            </button>
          </>
        )}
      </div>
    </>
  );
}
