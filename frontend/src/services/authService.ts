/**
 * Authentication service - API client for backend auth endpoints
 */

const API_BASE_URL = typeof window !== 'undefined' && window.location.hostname === 'localhost'
  ? 'http://localhost:8000'
  : `${window.location.protocol}//${window.location.host}`;

export interface User {
  id: number;
  email: string;
  name: string;
  ros2_experience: string;
  gpu_model?: string;
  gpu_vram?: string;
  operating_system?: string;
  robotics_knowledge: string;
  created_at: string;
}

export interface AuthResponse {
  access_token: string;
  token_type: string;
  user: User;
}

export interface SignUpData {
  email: string;
  password: string;
  name: string;
  ros2_experience?: string;
  gpu_model?: string;
  gpu_vram?: string;
  operating_system?: string;
  robotics_knowledge?: string;
}

export interface SignInData {
  email: string;
  password: string;
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
      const error = await response.json();
      throw new Error(error.detail || 'Sign-up failed');
    }

    return await response.json();
  } catch (error) {
    const message = error instanceof Error ? error.message : 'Sign-up failed - Network error';
    console.error('[AuthService] Signup error:', message);
    throw new Error(message);
  }
}

/**
 * Sign in a user
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
      const error = await response.json();
      throw new Error(error.detail || 'Sign-in failed');
    }

    return await response.json();
  } catch (error) {
    const message = error instanceof Error ? error.message : 'Sign-in failed - Network error';
    console.error('[AuthService] Signin error:', message);
    throw new Error(message);
  }
}

/**
 * Get user profile (requires JWT token)
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
      const error = await response.json();
      throw new Error(error.detail || 'Failed to fetch profile');
    }

    return await response.json();
  } catch (error) {
    const message = error instanceof Error ? error.message : 'Failed to fetch profile - Network error';
    console.error('[AuthService] Get profile error:', message);
    throw new Error(message);
  }
}

/**
 * Update user profile (requires JWT token)
 */
export async function updateProfile(token: string, data: UpdateProfileData): Promise<User> {
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
      const error = await response.json();
      throw new Error(error.detail || 'Failed to update profile');
    }

    return await response.json();
  } catch (error) {
    const message = error instanceof Error ? error.message : 'Failed to update profile - Network error';
    console.error('[AuthService] Update profile error:', message);
    throw new Error(message);
  }
}
