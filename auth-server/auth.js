/**
 * Better Auth Configuration
 * Implements authentication with user background questionnaire
 */

import { betterAuth } from "better-auth";
import { Pool } from "pg";
import * as dotenv from "dotenv";

dotenv.config();

// PostgreSQL connection pool (shared with FastAPI backend)
const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
});

// Better Auth instance with custom user fields for background questionnaire
export const auth = betterAuth({
  database: pool,
  secret: process.env.BETTER_AUTH_SECRET || "09d25e094faa6ca2556c818166b7a9563b93f7099f6f0f4caa6cf63b88e8d3e7",
  trustHost: true, // Trust the host header from Express
  appName: "AI Textbook Platform",
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false, // Disable for development/competition
    minPasswordLength: 8,
  },
  // Custom user fields for hardware/software background
  user: {
    additionalFields: {
      // User background questionnaire fields
      ros2_experience: {
        type: "string",
        required: false,
      },
      gpu_model: {
        type: "string",
        required: false,
      },
      gpu_vram: {
        type: "string",
        required: false,
      },
      operating_system: {
        type: "string",
        required: false,
      },
      robotics_knowledge: {
        type: "string",
        required: false,
      },
    },
  },
});
