/**
 * Authentication service for API calls to the backend
 */

import type { User, BackgroundData } from '../store/authStore';

// Get API base URL from environment or default to localhost for development
const API_BASE_URL = process.env.REACT_APP_API_BASE_URL ||
  (typeof window !== 'undefined' && window.location.hostname === 'localhost'
    ? 'http://localhost:8000'
    : `${window.location.protocol}//${window.location.host}`);

export interface SignUpData {
  email: string;
  password: string;
  name: string;
  background_data?: BackgroundData;
}

export interface SignInData {
  email: string;
  password: string;
}

export interface AuthResponse {
  access_token: string;
  token_type: string;
  user: User;
}

export interface UpdateProfileData {
  ros2_experience?: string;
  gpu_model?: string;
  gpu_vram?: string;
  operating_system?: string;
  robotics_knowledge?: string;
}

/**
 * Sign up a new user
 */
export async function signup(data: SignUpData): Promise<AuthResponse> {
  try {
    const response = await fetch(`${API_BASE_URL}/api/v1/auth/signup`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(data),
    });

    if (!response.ok) {
      let errorMessage = 'Sign-up failed';
      try {
        const error = await response.json();
        errorMessage = error.detail || error.message || 'Sign-up failed';
      } catch {
        errorMessage = `Sign-up failed (${response.status} ${response.statusText})`;
      }
      throw new Error(errorMessage);
    }

    return response.json();
  } catch (error) {
    const message = error instanceof Error ? error.message : 'Sign-up failed - Network error';
    console.error('[AuthService] Signup error:', message);
    throw new Error(message);
  }
}

/**
 * Sign in an existing user
 */
export async function signin(data: SignInData): Promise<AuthResponse> {
  try {
    const response = await fetch(`${API_BASE_URL}/api/v1/auth/signin`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(data),
    });

    if (!response.ok) {
      let errorMessage = 'Sign-in failed';
      try {
        const error = await response.json();
        errorMessage = error.detail || error.message || 'Sign-in failed';
      } catch {
        errorMessage = `Sign-in failed (${response.status} ${response.statusText})`;
      }
      throw new Error(errorMessage);
    }

    return response.json();
  } catch (error) {
    const message = error instanceof Error ? error.message : 'Sign-in failed - Network error';
    console.error('[AuthService] Signin error:', message);
    throw new Error(message);
  }
}

/**
 * Get current user profile
 */
export async function getProfile(token: string): Promise<User> {
  try {
    const response = await fetch(`${API_BASE_URL}/api/v1/auth/profile`, {
      method: 'GET',
      headers: {
        'Authorization': `Bearer ${token}`,
        'Content-Type': 'application/json',
      },
    });

    if (!response.ok) {
      let errorMessage = 'Failed to fetch profile';
      try {
        const error = await response.json();
        errorMessage = error.detail || error.message || 'Failed to fetch profile';
      } catch {
        errorMessage = `Failed to fetch profile (${response.status} ${response.statusText})`;
      }
      throw new Error(errorMessage);
    }

    return response.json();
  } catch (error) {
    const message = error instanceof Error ? error.message : 'Failed to fetch profile - Network error';
    console.error('[AuthService] Get profile error:', message);
    throw new Error(message);
  }
}

/**
 * Update user profile
 */
export async function updateProfile(
  token: string,
  data: UpdateProfileData
): Promise<User> {
  try {
    const response = await fetch(`${API_BASE_URL}/api/v1/auth/profile`, {
      method: 'PUT',
      headers: {
        'Authorization': `Bearer ${token}`,
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(data),
    });

    if (!response.ok) {
      let errorMessage = 'Failed to update profile';
      try {
        const error = await response.json();
        errorMessage = error.detail || error.message || 'Failed to update profile';
      } catch {
        errorMessage = `Failed to update profile (${response.status} ${response.statusText})`;
      }
      throw new Error(errorMessage);
    }

    return response.json();
  } catch (error) {
    const message = error instanceof Error ? error.message : 'Failed to update profile - Network error';
    console.error('[AuthService] Update profile error:', message);
    throw new Error(message);
  }
}
