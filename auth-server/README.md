# Better Auth Server

**Purpose**: Implements authentication using Better Auth for 50 bonus competition points.

## Architecture

```
Frontend (3000) → Better Auth Server (5000) → PostgreSQL (Neon)
                      ↓
                FastAPI (8000) validates JWT tokens
```

## Endpoints

- `POST /api/auth/sign-up/email` - Register new user with background data
- `POST /api/auth/sign-in/email` - Sign in existing user
- `GET /api/auth/get-session` - Get current user session
- `POST /api/auth/sign-out` - Sign out user

## User Background Fields

The following fields are collected at signup for content personalization:

- `ros2_experience`: ROS 2 experience level (None, Beginner, Intermediate, Advanced)
- `gpu_model`: GPU model (e.g., "NVIDIA RTX 3060")
- `gpu_vram`: GPU VRAM (e.g., "12GB")
- `operating_system`: Operating system (Ubuntu, Windows, macOS)
- `robotics_knowledge`: Robotics knowledge level (None, Beginner, Intermediate, Advanced)

## Running the Server

```bash
cd auth-server
npm install
npm start
```

The server will start on http://localhost:5000

## Database

Shares the same PostgreSQL database (Neon) with the FastAPI backend. Better Auth will automatically create necessary tables on first run.
