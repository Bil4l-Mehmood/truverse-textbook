/**
 * Better Auth Configuration
 * Using the existing PostgreSQL database for user storage
 */

import { betterAuth } from "better-auth";

// Better Auth configuration
export const auth = betterAuth({
  database: {
    provider: "postgres",
    url: process.env.NEXT_PUBLIC_DATABASE_URL || process.env.DATABASE_URL,
  },
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false, // Disable for development
  },
  socialProviders: {
    // Can add GitHub, Google, etc. later
  },
  // Custom user fields for background questionnaire
  user: {
    additionalFields: {
      ros2_experience: {
        type: "string",
        defaultValue: "Beginner",
        input: true,
      },
      gpu_model: {
        type: "string",
        required: false,
        input: true,
      },
      gpu_vram: {
        type: "string",
        required: false,
        input: true,
      },
      operating_system: {
        type: "string",
        required: false,
        input: true,
      },
      robotics_knowledge: {
        type: "string",
        defaultValue: "Beginner",
        input: true,
      },
    },
  },
  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days
    updateAge: 60 * 60 * 24, // 1 day
  },
});

export type Session = typeof auth.$Infer.Session;
