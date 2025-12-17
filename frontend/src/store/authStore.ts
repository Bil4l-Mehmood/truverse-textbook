/**
 * Authentication state management using Zustand.
 * Manages user authentication state, JWT token, and user profile.
 */

import { create } from 'zustand';
import { persist } from 'zustand/middleware';

export interface BackgroundData {
  ros2_experience: string;
  gpu_model?: string;
  gpu_vram?: string;
  operating_system?: string;
  robotics_knowledge: string;
}

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

interface AuthState {
  user: User | null;
  token: string | null;
  isAuthenticated: boolean;

  // Actions
  setUser: (user: User) => void;
  setToken: (token: string) => void;
  setAuth: (user: User, token: string) => void;
  logout: () => void;
  updateUser: (updates: Partial<User>) => void;
}

export const useAuthStore = create<AuthState>()(
  persist(
    (set) => ({
      user: null,
      token: null,
      isAuthenticated: false,

      setUser: (user) =>
        set({
          user,
          isAuthenticated: true,
        }),

      setToken: (token) =>
        set({
          token,
        }),

      setAuth: (user, token) =>
        set({
          user,
          token,
          isAuthenticated: true,
        }),

      logout: () =>
        set({
          user: null,
          token: null,
          isAuthenticated: false,
        }),

      updateUser: (updates) =>
        set((state) => ({
          user: state.user ? { ...state.user, ...updates } : null,
        })),
    }),
    {
      name: 'auth-storage', // localStorage key
      partialize: (state) => ({
        user: state.user,
        token: state.token,
        isAuthenticated: state.isAuthenticated,
      }),
    }
  )
);
