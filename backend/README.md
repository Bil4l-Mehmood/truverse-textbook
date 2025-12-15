# AI Textbook Platform - Backend

FastAPI backend for the AI-Native Physical AI & Humanoid Robotics Textbook Platform.

## Features

- **RAG Chatbot**: Retrieval-Augmented Generation using Qdrant vector store and OpenAI GPT-4
- **User Authentication**: JWT-based auth with background questionnaire
- **Content Personalization**: AI-powered content adaptation based on user skill level
- **Multilingual Support**: Urdu translation with RTL rendering
- **AI Skills**: Hardware specs lookup and ROS 2 command generation

## Tech Stack

- **Framework**: FastAPI 0.109+
- **Database**: Neon Serverless Postgres (SQLAlchemy async)
- **Vector Store**: Qdrant Cloud (vector embeddings)
- **AI/ML**: OpenAI GPT-4 and text-embedding-3-small
- **Auth**: JWT tokens with bcrypt password hashing

## Prerequisites

1. **Python 3.10+**
2. **Neon Postgres** account - [Sign up](https://neon.tech/)
3. **Qdrant Cloud** account - [Sign up](https://cloud.qdrant.io/)
4. **OpenAI API** key - [Get API key](https://platform.openai.com/api-keys)

## Setup Instructions

### 1. Install Dependencies

```bash
cd backend
pip install -r requirements.txt
```

### 2. Configure Environment Variables

Copy the example environment file and fill in your credentials:

```bash
cp .env.example .env
```

Edit `.env` with your actual credentials:

```env
# Neon Postgres
NEON_DATABASE_URL=postgresql+asyncpg://username:password@your-host.neon.tech/neondb?sslmode=require

# Qdrant Cloud
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_api_key

# OpenAI
OPENAI_API_KEY=sk-your_api_key

# JWT Secret (generate with: openssl rand -hex 32)
JWT_SECRET=your_secret_key
```

### 3. Initialize Database

The database will be automatically initialized when you start the server. Tables will be created from SQLAlchemy models.

### 4. Run Data Ingestion

Ingest Docusaurus content into Qdrant vector store:

```bash
python scripts/ingest_content.py
```

This will:
- Read all Markdown files from `frontend/docs/`
- Chunk content into 512-token segments
- Generate embeddings using OpenAI
- Upload vectors to Qdrant

Expected output:
```
INFO:__main__:Found 15 Markdown files
INFO:__main__:Processing 15 documents...
INFO:__main__:Total chunks: 487
INFO:__main__:Generating embeddings...
INFO:__main__:Uploading vectors to Qdrant...
INFO:__main__:Ingestion complete!
```

### 5. Start the Server

```bash
# Development mode (with auto-reload)
uvicorn main:app --reload

# Production mode
uvicorn main:app --host 0.0.0.0 --port 8000
```

### 6. Verify Installation

Check the status endpoint to verify all connections:

```bash
curl http://localhost:8000/api/v1/status
```

Expected response:
```json
{
  "api": "online",
  "version": "1.0.0",
  "postgres": "connected",
  "qdrant": {
    "status": "connected",
    "collection": "textbook_embeddings",
    "vectors_count": 487
  },
  "openai": "api_key_configured"
}
```

## Project Structure

```
backend/
├── main.py                 # FastAPI application entry point
├── requirements.txt        # Python dependencies
├── pyproject.toml          # Python project config (Black, mypy, pytest)
├── .env.example            # Environment variables template
│
├── src/
│   ├── api/                # API route handlers
│   ├── core/               # Core utilities
│   │   ├── config.py       # Settings and configuration
│   │   └── exceptions.py   # Custom exception classes
│   ├── database/           # Database connections
│   │   ├── postgres.py     # Neon Postgres connection
│   │   └── qdrant.py       # Qdrant vector store connection
│   ├── models/             # SQLAlchemy ORM models
│   │   ├── user.py         # User model
│   │   └── chat.py         # Chat session and message models
│   ├── schemas/            # Pydantic request/response schemas
│   ├── services/           # Business logic services
│   └── utils/              # Utility functions
│       └── logging.py      # Structured JSON logging
│
├── scripts/                # Utility scripts
│   └── ingest_content.py   # Data ingestion script
│
├── data/                   # Generated data files
│   └── ingestion_manifest.json
│
└── tests/                  # Test files
```

## API Endpoints

### Status & Health

- `GET /` - Root endpoint with API info
- `GET /health` - Health check
- `GET /api/v1/status` - Detailed status with database connections

### Documentation

- `GET /docs` - Interactive API documentation (Swagger UI)
- `GET /redoc` - Alternative API documentation (ReDoc)

## Development

### Code Quality

```bash
# Format code with Black
black src/

# Lint with Flake8
flake8 src/

# Type checking with mypy
mypy src/
```

### Running Tests

```bash
pytest
```

## Deployment

The backend is designed to be deployed on **Railway** (not Vercel Serverless).

### Railway Deployment

1. Create a Railway account at [railway.app](https://railway.app/)
2. Create a new project and connect your GitHub repository
3. Set environment variables in Railway dashboard
4. Deploy using `railway up` or GitHub integration

### Environment Variables for Production

Make sure to set all variables from `.env.example` in your Railway dashboard.

## Troubleshooting

### Connection Errors

If you see "postgres: error" or "qdrant: error" in `/api/v1/status`:

1. **Neon Postgres**: Verify your connection string includes `?sslmode=require`
2. **Qdrant**: Check your cluster URL and API key are correct
3. **OpenAI**: Ensure API key starts with `sk-` and has credits

### Ingestion Failures

If data ingestion fails:

1. Check that `frontend/docs/` directory exists and contains `.md` files
2. Verify OpenAI API key is valid and has sufficient quota
3. Check Qdrant cluster has available storage (free tier = 1GB)

### Rate Limiting

If you hit OpenAI rate limits during ingestion:
- The script includes automatic delays between batches
- Reduce batch size in `ingest_content.py` if needed
- Check your OpenAI tier limits at platform.openai.com

## License

This project is part of the AI Textbook Platform hackathon submission.
