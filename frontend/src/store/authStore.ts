/**
 * Zustand auth store for managing global authentication state
 * Persists token and user data to localStorage (client-side only)
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

// Helper to safely access localStorage (only available on client)
const getLocalStorage = () => {
  if (typeof window !== 'undefined' && typeof window.localStorage !== 'undefined') {
    return window.localStorage;
  }
  return null;
};

export const useAuthStore = create<AuthState>((set) => ({
  // Initial state
  user: null,
  token: null,
  isAuthenticated: false,

  // Action: Set auth (after successful login)
  setAuth: (token: string, user: User) => {
    const storage = getLocalStorage();
    if (storage) {
      storage.setItem('auth_token', token);
      storage.setItem('auth_user', JSON.stringify(user));
    }
    set({
      token,
      user,
      isAuthenticated: true,
    });
  },

  // Action: Update user profile
  setUser: (user: User) => {
    const storage = getLocalStorage();
    if (storage) {
      storage.setItem('auth_user', JSON.stringify(user));
    }
    set({ user });
  },

  // Action: Logout
  logout: () => {
    const storage = getLocalStorage();
    if (storage) {
      storage.removeItem('auth_token');
      storage.removeItem('auth_user');
    }
    set({
      token: null,
      user: null,
      isAuthenticated: false,
    });
  },

  // Action: Load from localStorage (call on app start)
  loadFromLocalStorage: () => {
    const storage = getLocalStorage();
    if (!storage) return;

    try {
      const token = storage.getItem('auth_token');
      const userJson = storage.getItem('auth_user');

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
      storage.removeItem('auth_token');
      storage.removeItem('auth_user');
    }
  },
}));
