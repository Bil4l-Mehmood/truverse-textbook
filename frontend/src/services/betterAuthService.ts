/**
 * Better Auth Service
 * Connects to the Node.js Better Auth server on port 5000
 */

const BETTER_AUTH_URL = process.env.NODE_ENV === 'production'
  ? 'https://your-auth-server.railway.app' // Update when deployed
  : 'http://localhost:5000';

export interface SignUpRequest {
  email: string;
  password: string;
  name: string;
  image?: string;
  // Background questionnaire data
  ros2_experience?: string;
  gpu_model?: string;
  gpu_vram?: string;
  operating_system?: string;
  robotics_knowledge?: string;
}

export interface SignInRequest {
  email: string;
  password: string;
}

export interface User {
  id: string;
  email: string;
  name: string;
  image?: string;
  emailVerified: boolean;
  createdAt: string;
  updatedAt: string;
  // Background data
  ros2_experience?: string;
  gpu_model?: string;
  gpu_vram?: string;
  operating_system?: string;
  robotics_knowledge?: string;
}

export interface Session {
  session: {
    token: string;
    expiresAt: string;
    userId: string;
  };
  user: User;
}

/**
 * Sign up a new user with Better Auth
 */
export async function betterAuthSignUp(data: SignUpRequest): Promise<Session> {
  console.log('[BetterAuth] Signing up user:', data.email);

  try {
    const response = await fetch(`${BETTER_AUTH_URL}/api/auth/sign-up/email`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      credentials: 'include',
      body: JSON.stringify(data),
    });

    if (!response.ok) {
      let errorMessage = 'Sign up failed';
      try {
        const error = await response.json();
        errorMessage = error.message || error.error || 'Sign up failed';
      } catch {
        errorMessage = `Sign up failed (${response.status} ${response.statusText})`;
      }
      console.error('[BetterAuth] Signup failed:', errorMessage);
      throw new Error(errorMessage);
    }

    const result = await response.json();
    console.log('[BetterAuth] Signup successful:', result.user);
    return result;
  } catch (error) {
    const message = error instanceof Error ? error.message : 'Sign up failed - Network error';
    console.error('[BetterAuth] Signup error:', message);
    throw new Error(message);
  }
}

/**
 * Sign in an existing user with Better Auth
 */
export async function betterAuthSignIn(data: SignInRequest): Promise<Session> {
  console.log('[BetterAuth] Signing in user:', data.email);

  try {
    console.log('[BetterAuth] Fetching from:', `${BETTER_AUTH_URL}/api/auth/sign-in/email`);

    const response = await fetch(`${BETTER_AUTH_URL}/api/auth/sign-in/email`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      credentials: 'include',
      body: JSON.stringify(data),
    });

    console.log('[BetterAuth] Response status:', response.status, response.statusText);

    if (!response.ok) {
      let errorMessage = 'Sign in failed';
      try {
        const error = await response.json();
        errorMessage = error.message || error.error || 'Sign in failed';
      } catch {
        errorMessage = `Sign in failed (${response.status} ${response.statusText})`;
      }
      console.error('[BetterAuth] Signin failed:', errorMessage);
      throw new Error(errorMessage);
    }

    const result = await response.json();
    console.log('[BetterAuth] Raw response:', JSON.stringify(result, null, 2));
    console.log('[BetterAuth] Signin successful, user:', result.user);
    console.log('[BetterAuth] Session token:', result.session?.token ? 'Present' : 'Missing');
    return result;
  } catch (error) {
    const message = error instanceof Error ? error.message : 'Sign in failed - Network error';
    console.error('[BetterAuth] Signin error:', message);
    throw new Error(message);
  }
}

/**
 * Get current session from Better Auth
 */
export async function betterAuthGetSession(): Promise<Session | null> {
  try {
    const response = await fetch(`${BETTER_AUTH_URL}/api/auth/get-session`, {
      method: 'GET',
      headers: {
        'Content-Type': 'application/json',
      },
      credentials: 'include',
    });

    if (!response.ok) {
      return null;
    }

    const result = await response.json();
    return result;
  } catch (error) {
    console.error('[BetterAuth] Get session failed:', error);
    return null;
  }
}

/**
 * Sign out current user from Better Auth
 */
export async function betterAuthSignOut(): Promise<void> {
  console.log('[BetterAuth] Signing out user');

  try {
    const response = await fetch(`${BETTER_AUTH_URL}/api/auth/sign-out`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      credentials: 'include',
    });

    if (!response.ok) {
      let errorMessage = 'Sign out failed';
      try {
        const error = await response.json();
        errorMessage = error.message || error.error || 'Sign out failed';
      } catch {
        errorMessage = `Sign out failed (${response.status} ${response.statusText})`;
      }
      console.error('[BetterAuth] Signout failed:', errorMessage);
      throw new Error(errorMessage);
    }

    console.log('[BetterAuth] Signout successful');
  } catch (error) {
    const message = error instanceof Error ? error.message : 'Sign out failed - Network error';
    console.error('[BetterAuth] Signout error:', message);
    throw new Error(message);
  }
}
