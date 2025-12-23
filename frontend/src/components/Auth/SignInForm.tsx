/**
 * Sign-in form component
 * Connects to FastAPI backend
 */

import React, { useState } from 'react';
import { useAuthStore } from '../../store/authStore';
import { signin } from '../../services/authService';

export default function SignInForm() {
  const setAuth = useAuthStore((state) => state.setAuth);

  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    setLoading(true);

    console.log('[SignIn] Attempting sign in for:', email);

    try {
      const response = await signin({ email, password });

      console.log('[SignIn] Sign in successful:', response.user);

      // Store auth state
      const authUser = {
        id: response.user.id,
        email: response.user.email,
        name: response.user.name,
        ros2_experience: response.user.ros2_experience,
        gpu_model: response.user.gpu_model,
        gpu_vram: response.user.gpu_vram,
        operating_system: response.user.operating_system,
        robotics_knowledge: response.user.robotics_knowledge,
      };

      console.log('[SignIn] Storing auth state:', { user: authUser, token: response.access_token });
      setAuth(authUser, response.access_token);

      console.log('[SignIn] Auth state stored, about to redirect');

      // Check for redirect URL from AuthGuard
      const redirectUrl = sessionStorage.getItem('redirectAfterLogin');
      if (redirectUrl) {
        console.log('[SignIn] Redirecting to originally requested page:', redirectUrl);
        sessionStorage.removeItem('redirectAfterLogin');
        window.location.href = redirectUrl;
      } else {
        console.log('[SignIn] Redirecting to home');
        window.location.href = '/';
      }
    } catch (err) {
      console.error('[SignIn] Sign in failed:', err);
      const errorMessage = err instanceof Error ? err.message : 'Sign-in failed. Please try again.';
      setError(errorMessage);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="auth-container">
      <div className="auth-card">
        <h1>Sign In</h1>
        <p className="auth-subtitle">Welcome back! Please sign in to continue</p>

        {error && <div className="auth-error">{error}</div>}

        <form onSubmit={handleSubmit} className="auth-form">
          <div className="form-group">
            <label htmlFor="email">Email</label>
            <input
              type="email"
              id="email"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              placeholder="you@example.com"
              required
              autoComplete="email"
            />
          </div>

          <div className="form-group">
            <label htmlFor="password">Password</label>
            <input
              type="password"
              id="password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              placeholder="Enter your password"
              required
              autoComplete="current-password"
            />
          </div>

          <button type="submit" className="btn-primary" disabled={loading}>
            {loading ? 'Signing in...' : 'Sign In'}
          </button>
        </form>

        <p className="auth-footer">
          Don't have an account? <a href="/signup">Sign up</a>
        </p>
      </div>
    </div>
  );
}
