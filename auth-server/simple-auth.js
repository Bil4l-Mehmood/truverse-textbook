/**
 * Simple Authentication without Better Auth
 * Direct implementation using bcrypt and JWT
 */

import bcrypt from 'bcrypt';
import jwt from 'jsonwebtoken';
import { Pool } from 'pg';
import * as dotenv from 'dotenv';

dotenv.config();

const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
});

const SECRET = process.env.BETTER_AUTH_SECRET || '09d25e094faa6ca2556c818166b7a9563b93f7099f6f0f4caa6cf63b88e8d3e7';

// Initialize database tables
export async function initDatabase() {
  try {
    await pool.query(`
      CREATE TABLE IF NOT EXISTS "user" (
        id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
        email VARCHAR(255) UNIQUE NOT NULL,
        password VARCHAR(255) NOT NULL,
        name VARCHAR(255),
        image VARCHAR(255),
        "emailVerified" BOOLEAN DEFAULT false,
        "ros2_experience" VARCHAR(255),
        "gpu_model" VARCHAR(255),
        "gpu_vram" VARCHAR(255),
        "operating_system" VARCHAR(255),
        "robotics_knowledge" VARCHAR(255),
        "createdAt" TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
        "updatedAt" TIMESTAMP DEFAULT CURRENT_TIMESTAMP
      );
    `);
    console.log('✅ Database tables initialized');
  } catch (error) {
    console.error('Database init error:', error.message);
  }
}

// Sign up
export async function signUp(email, password, name) {
  try {
    // Check if user exists
    const existing = await pool.query('SELECT id FROM "user" WHERE email = $1', [email]);
    if (existing.rows.length > 0) {
      throw new Error('User already exists');
    }

    // Hash password
    const hashedPassword = await bcrypt.hash(password, 12);

    // Create user
    const result = await pool.query(
      'INSERT INTO "user" (email, password, name) VALUES ($1, $2, $3) RETURNING id, email, name',
      [email, hashedPassword, name]
    );

    const user = result.rows[0];

    // Create token
    const token = jwt.sign({ sub: user.id, email: user.email }, SECRET, { expiresIn: '7d' });

    return {
      session: {
        token,
        expiresAt: new Date(Date.now() + 7 * 24 * 60 * 60 * 1000).toISOString(),
        userId: user.id,
      },
      user: {
        id: user.id,
        email: user.email,
        name: user.name,
        emailVerified: false,
      },
    };
  } catch (error) {
    throw new Error(error.message);
  }
}

// Sign in
export async function signIn(email, password) {
  try {
    const result = await pool.query('SELECT id, email, name, password FROM "user" WHERE email = $1', [email]);

    if (result.rows.length === 0) {
      throw new Error('Invalid credentials');
    }

    const user = result.rows[0];

    // Check password
    const valid = await bcrypt.compare(password, user.password);
    if (!valid) {
      throw new Error('Invalid credentials');
    }

    // Create token
    const token = jwt.sign({ sub: user.id, email: user.email }, SECRET, { expiresIn: '7d' });

    return {
      session: {
        token,
        expiresAt: new Date(Date.now() + 7 * 24 * 60 * 60 * 1000).toISOString(),
        userId: user.id,
      },
      user: {
        id: user.id,
        email: user.email,
        name: user.name,
        emailVerified: false,
      },
    };
  } catch (error) {
    throw new Error(error.message);
  }
}

// Get session
export async function getSession(token) {
  try {
    const payload = jwt.verify(token, SECRET);
    const result = await pool.query('SELECT id, email, name FROM "user" WHERE id = $1', [payload.sub]);

    if (result.rows.length === 0) {
      return null;
    }

    const user = result.rows[0];

    return {
      session: {
        token,
        expiresAt: new Date(Date.now() + 7 * 24 * 60 * 60 * 1000).toISOString(),
        userId: user.id,
      },
      user: {
        id: user.id,
        email: user.email,
        name: user.name,
      },
    };
  } catch (error) {
    return null;
  }
}

// Sign out (just invalidate on client side)
export async function signOut() {
  return { success: true };
}
