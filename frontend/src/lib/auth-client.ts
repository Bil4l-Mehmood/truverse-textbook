/**
 * Better Auth Client for React
 * Connects to the Better Auth server (Node.js) on port 5000
 */

import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: process.env.NODE_ENV === "production"
    ? "https://your-auth-server.railway.app"
    : "http://localhost:5000",
  // Better Auth default endpoints (do NOT override - use defaults)
  // Default endpoints:
  // - POST /api/auth/sign-up/email
  // - POST /api/auth/sign-in/email
  // - POST /api/auth/sign-out
  // - GET /api/auth/get-session
});

// Export hooks for use in components
export const {
  useSession,
  signIn,
  signUp,
  signOut,
  useActiveOrganization,
} = authClient;
