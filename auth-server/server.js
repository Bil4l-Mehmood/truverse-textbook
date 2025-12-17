/**
 * Better Auth Server
 * Handles authentication for AI Textbook Platform
 * Runs on Port 5000 (separate from FastAPI backend on 8000)
 */

import express from "express";
import cors from "cors";
import * as dotenv from "dotenv";
import { initDatabase, signUp, signIn, getSession, signOut } from "./simple-auth.js";

dotenv.config();

const app = express();
const PORT = process.env.PORT || 5000;

// Middleware
app.use(cors({
  origin: [
    "http://localhost:3000",
    "http://localhost:8000",
    "https://frontend-flv3y28hx-billals-projects-66089552.vercel.app"
  ],
  credentials: true,
}));

app.use(express.json());

// Set proper headers for Better Auth
app.use((req, res, next) => {
  // Set X-Forwarded-Proto for trustHost to work
  if (!req.headers['x-forwarded-proto']) {
    req.headers['x-forwarded-proto'] = req.protocol || 'http';
  }
  if (!req.headers['x-forwarded-host']) {
    req.headers['x-forwarded-host'] = req.hostname || 'localhost:5000';
  }
  next();
});

// Auth endpoints
app.post("/api/auth/sign-up/email", async (req, res) => {
  console.log(`[Auth] POST /api/auth/sign-up/email`);
  try {
    const { email, password, name } = req.body;
    const result = await signUp(email, password, name);
    res.status(201).json(result);
  } catch (error) {
    console.error(`[Auth] Sign up error:`, error.message);
    res.status(400).json({ error: error.message });
  }
});

app.post("/api/auth/sign-in/email", async (req, res) => {
  console.log(`[Auth] POST /api/auth/sign-in/email`);
  try {
    const { email, password } = req.body;
    const result = await signIn(email, password);
    res.status(200).json(result);
  } catch (error) {
    console.error(`[Auth] Sign in error:`, error.message);
    res.status(401).json({ error: error.message });
  }
});

app.get("/api/auth/get-session", async (req, res) => {
  console.log(`[Auth] GET /api/auth/get-session`);
  try {
    const token = req.headers.authorization?.replace('Bearer ', '');
    if (!token) {
      return res.status(401).json({ error: 'No token provided' });
    }
    const session = await getSession(token);
    if (!session) {
      return res.status(401).json({ error: 'Invalid token' });
    }
    res.status(200).json(session);
  } catch (error) {
    console.error(`[Auth] Get session error:`, error.message);
    res.status(401).json({ error: error.message });
  }
});

app.post("/api/auth/sign-out", async (req, res) => {
  console.log(`[Auth] POST /api/auth/sign-out`);
  try {
    const result = await signOut();
    res.status(200).json(result);
  } catch (error) {
    console.error(`[Auth] Sign out error:`, error.message);
    res.status(400).json({ error: error.message });
  }
});

// Health check endpoint
app.get("/health", (req, res) => {
  res.json({
    status: "healthy",
    service: "Better Auth Server",
    timestamp: new Date().toISOString()
  });
});

// Root endpoint
app.get("/", (req, res) => {
  res.json({
    message: "Better Auth Server for AI Textbook Platform",
    version: "1.0.0",
    endpoints: {
      signup: "POST /api/auth/sign-up/email",
      signin: "POST /api/auth/sign-in/email",
      session: "GET /api/auth/get-session",
      signout: "POST /api/auth/sign-out"
    }
  });
});

// Initialize and start server
async function start() {
  try {
    await initDatabase();

    const server = app.listen(PORT, () => {
      console.log(`✅ Auth Server running on http://localhost:${PORT}`);
      console.log(`📚 Serving AI Textbook Platform authentication`);
      console.log(`🔐 Endpoints available at /api/auth/*`);
      console.log(`🗄️  Database: ${process.env.DATABASE_URL ? '✓ Connected' : '✗ Not configured'}`);
      console.log(`🔑 Auth Secret: ${process.env.BETTER_AUTH_SECRET ? '✓ Configured' : '✗ Not configured'}`);
    });

    server.on('error', (err) => {
      if (err.code === 'EADDRINUSE') {
        console.error(`❌ Port ${PORT} is already in use`);
        process.exit(1);
      } else {
        console.error(`❌ Server error: ${err.message}`);
        process.exit(1);
      }
    });
  } catch (error) {
    console.error('❌ Startup error:', error.message);
    process.exit(1);
  }
}

start();

// Handle uncaught errors
process.on('unhandledRejection', (reason, promise) => {
  console.error(`❌ Unhandled Rejection:`, reason);
});

process.on('uncaughtException', (error) => {
  console.error(`❌ Uncaught Exception:`, error);
  process.exit(1);
});
