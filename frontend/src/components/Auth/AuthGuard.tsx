/**
 * Auth Guard component to protect routes that require authentication
 */

import React, { useEffect } from 'react';
import { useAuthStore } from '../../store/authStore';

interface AuthGuardProps {
  children: React.ReactNode;
  fallbackPath?: string;
}

export default function AuthGuard({ children, fallbackPath = '/signin' }: AuthGuardProps) {
  const { isAuthenticated, user, token } = useAuthStore();

  useEffect(() => {
    // Check if user is authenticated
    if (!isAuthenticated || !user || !token) {
      console.log('[AuthGuard] User not authenticated, redirecting to sign in...');

      // Save the intended destination for redirect after login
      const currentPath = window.location.pathname + window.location.search;
      if (currentPath !== fallbackPath) {
        sessionStorage.setItem('redirectAfterLogin', currentPath);
      }

      // Redirect to sign in
      window.location.href = fallbackPath;
    } else {
      console.log('[AuthGuard] User authenticated:', user.email);
    }
  }, [isAuthenticated, user, token, fallbackPath]);

  // Don't render children if not authenticated
  if (!isAuthenticated || !user || !token) {
    return (
      <div style={{ padding: '2rem', textAlign: 'center' }}>
        <p>Checking authentication...</p>
      </div>
    );
  }

  return <>{children}</>;
}
