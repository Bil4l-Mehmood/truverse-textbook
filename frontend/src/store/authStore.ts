/**
 * Zustand auth store for managing global authentication state
 * Persists token and user data to localStorage
 */

import { create } from 'zustand';
import { User } from '../services/authService';

interface AuthState {
  // State
  user: User | null;
  token: string | null;
  isAuthenticated: boolean;

  // Actions
  setAuth: (token: string, user: User) => void;
  setUser: (user: User) => void;
  logout: () => void;
  loadFromLocalStorage: () => void;
}

export const useAuthStore = create<AuthState>((set) => ({
  // Initial state
  user: null,
  token: null,
  isAuthenticated: false,

  // Action: Set auth (after successful login)
  setAuth: (token: string, user: User) => {
    localStorage.setItem('auth_token', token);
    localStorage.setItem('auth_user', JSON.stringify(user));
    set({
      token,
      user,
      isAuthenticated: true,
    });
  },

  // Action: Update user profile
  setUser: (user: User) => {
    localStorage.setItem('auth_user', JSON.stringify(user));
    set({ user });
  },

  // Action: Logout
  logout: () => {
    localStorage.removeItem('auth_token');
    localStorage.removeItem('auth_user');
    set({
      token: null,
      user: null,
      isAuthenticated: false,
    });
  },

  // Action: Load from localStorage (call on app start)
  loadFromLocalStorage: () => {
    try {
      const token = localStorage.getItem('auth_token');
      const userJson = localStorage.getItem('auth_user');

      if (token && userJson) {
        const user = JSON.parse(userJson);
        set({
          token,
          user,
          isAuthenticated: true,
        });
      }
    } catch (error) {
      console.error('Failed to load auth from localStorage:', error);
      // Clear invalid data
      localStorage.removeItem('auth_token');
      localStorage.removeItem('auth_user');
    }
  },
}));
