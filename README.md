# AI-Native Physical AI & Humanoid Robotics Textbook

An interactive, AI-powered textbook platform for learning Physical AI and Humanoid Robotics, built with Docusaurus and FastAPI.

## 🎯 Project Overview

This project delivers a **300-point hackathon submission** featuring:
- **100 Core Points**: Interactive Docusaurus textbook + RAG chatbot
- **200 Bonus Points**: Authentication, personalization, Urdu translation, Claude Code skills

### Features

#### Core Features (100 Points)
1. **P1 - Interactive Textbook Foundation** (MVP)
   - Docusaurus-based static site with full course content
   - Responsive design (mobile, tablet, desktop)
   - Professional technical textbook styling
   - Deployed on Vercel

2. **P2 - Intelligent RAG Chatbot**
   - Contextual RAG: Answer questions based on highlighted text
   - General RAG: Retrieve relevant content from full textbook corpus
   - Multi-turn conversations with history
   - Embedded chat widget on all pages

#### Bonus Features (200 Points)

3. **P3 - User Authentication** (50 pts)
   - Sign-up/sign-in with Auth.js
   - Background questionnaire (ROS 2 experience, GPU specs, OS, robotics knowledge)
   - User profile management

4. **P4 - Content Personalization** (50 pts)
   - "Personalize This Chapter" button
   - AI-powered content adaptation based on user skill level
   - Beginner users see expanded explanations, Advanced users see condensed basics

5. **P5 - Urdu Translation** (50 pts)
   - "Translate to Urdu" button on all chapters
   - LLM-powered translation with RTL rendering
   - Preserves code blocks in original English

6. **P6 - Claude Code Skills** (50 pts)
   - Hardware_Spec_Lookup: Component specifications (Jetson, GPUs, etc.)
   - ROS2_Command_Generator: Generate ROS 2 commands for tasks
   - Accessible via chat widget (`/hardware`, `/ros2` syntax)

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Vercel (Frontend)                     │
│  ┌─────────────────────────────────────────────────┐   │
│  │          Docusaurus (React + TypeScript)        │   │
│  │  - Course content (Markdown chapters)           │   │
│  │  - Chat Widget (custom React component)        │   │
│  │  - Auth UI (sign-up, sign-in, profile)         │   │
│  │  - Personalization & Translation UI             │   │
│  └─────────────────────────────────────────────────┘   │
└──────────────────────┬──────────────────────────────────┘
                        │ HTTPS/REST API
┌──────────────────────▼──────────────────────────────────┐
│                  Railway (Backend)                       │
│  ┌─────────────────────────────────────────────────┐   │
│  │            FastAPI (Python + Pydantic)          │   │
│  │  - RAG Service (contextual + general queries)  │   │
│  │  - Auth Service (JWT, password hashing)        │   │
│  │  - Personalization Service (GPT-4)             │   │
│  │  - Translation Service (GPT-4)                 │   │
│  │  - Skills (Hardware Lookup, ROS2 Generator)    │   │
│  └─────────────────────────────────────────────────┘   │
└──────┬───────────────────┬──────────────────┬───────────┘
       │                    │                   │
       ▼                    ▼                   ▼
┌──────────────┐  ┌──────────────────┐  ┌──────────────┐
│ Neon Postgres│  │  Qdrant Cloud    │  │  OpenAI API  │
│              │  │                  │  │              │
│ - Users      │  │ - Vector         │  │ - Embeddings │
│ - Chat       │  │   embeddings     │  │ - Chat (GPT-4)│
│ - Prefs      │  │ - Textbook       │  │ - Translation│
│ - Cache      │  │   chunks         │  │              │
└──────────────┘  └──────────────────┘  └──────────────┘
```

## 🛠️ Tech Stack

### Frontend
- **Framework**: Docusaurus 3.x (React 18 + TypeScript)
- **State Management**: Zustand
- **Styling**: Custom CSS + Docusaurus Infima
- **Deployment**: Vercel
- **Key Libraries**: react-markdown, axios

### Backend
- **Framework**: FastAPI (Python 3.10+)
- **Database ORM**: SQLAlchemy (async) + Alembic
- **Authentication**: Auth.js + Prisma
- **Validation**: Pydantic 2.x
- **Deployment**: Railway
- **Key Libraries**: qdrant-client, openai, python-jose, passlib

### Data & AI
- **Relational DB**: Neon Serverless Postgres (free tier)
- **Vector Store**: Qdrant Cloud (free tier, 1GB)
- **LLM**: OpenAI GPT-4 (chat, personalization, translation)
- **Embeddings**: OpenAI text-embedding-3-small (1536 dimensions)

## 🚀 Quick Start

### Prerequisites

- Node.js 18+ (for Docusaurus)
- Python 3.10+ (for FastAPI backend)
- OpenAI API key
- Neon Postgres database URL
- Qdrant Cloud URL + API key

### Environment Variables

#### Frontend (`.env` in `frontend/`)
```env
REACT_APP_API_BASE_URL=https://your-railway-backend.up.railway.app
```

#### Backend (`.env` in `backend/`)
```env
NEON_DATABASE_URL=postgresql://user:password@ep-xxx.neon.tech/dbname
QDRANT_URL=https://xyz.qdrant.io:6333
QDRANT_API_KEY=your-qdrant-api-key
OPENAI_API_KEY=sk-xxx
JWT_SECRET=your-secret-key-here
```

### Local Development

#### Frontend
```bash
cd frontend
npm install
npm start
# Docusaurus dev server runs on http://localhost:3000
```

#### Backend
```bash
cd backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
alembic upgrade head  # Run database migrations
python backend/scripts/ingest_content.py  # Load textbook into Qdrant
uvicorn backend.main:app --reload
# FastAPI dev server runs on http://localhost:8000
```

### Deployment

#### Frontend (Vercel)
1. Push to GitHub
2. Connect repository to Vercel
3. Set `REACT_APP_API_BASE_URL` environment variable
4. Deploy (auto-deploys on push to main)

#### Backend (Railway)
1. Push to GitHub
2. Connect repository to Railway
3. Set all environment variables (NEON_DATABASE_URL, QDRANT_URL, etc.)
4. Deploy (auto-deploys on push to main)

## 📋 Development Workflow

This project follows **Spec-Driven Development (SDD)** methodology:

1. **Specification** ([spec.md](specs/001-ai-textbook-platform/spec.md))
   - 6 prioritized user stories (P1-P6)
   - 42 functional requirements
   - 10 measurable success criteria

2. **Planning** ([plan.md](specs/001-ai-textbook-platform/plan.md))
   - 5-phase implementation plan
   - Technology research & decisions
   - Risk analysis & mitigation

3. **Tasks** ([tasks.md](specs/001-ai-textbook-platform/tasks.md))
   - 104 actionable tasks organized by user story
   - Clear dependencies and parallelization opportunities
   - MVP-first strategy (P1+P2 = 100 points)

4. **Implementation**
   - Execute tasks in order (Setup → Foundational → User Stories → Polish)
   - Each user story independently testable
   - Incremental delivery (deploy after each story)

## 📊 Project Progress

Current Status: **[Implementation Phase]**

- ✅ Phase 0: Research & Validation (8 technology decisions)
- 🚧 Phase 1: Docusaurus Setup (in progress)
- ⏳ Phase 2: RAG Backend Infrastructure
- ⏳ Phase 3: RAG & Chat Integration
- ⏳ Phase 4: Authentication & Personalization
- ⏳ Phase 5: Translation & Skills

## 🧪 Testing

### Manual Testing Checklist

**P1 (Textbook)**:
- [ ] Navigate to deployed Vercel URL
- [ ] Click through all chapters in sidebar
- [ ] Verify responsive design on mobile/tablet/desktop
- [ ] Run Lighthouse audit (target: >90 accessibility)

**P2 (RAG Chatbot)**:
- [ ] Highlight text in any chapter
- [ ] Ask "What does this mean?" in chat widget
- [ ] Verify response uses only highlighted text
- [ ] Ask general question without highlight
- [ ] Verify retrieval from full textbook corpus
- [ ] Check conversation history maintained

**P3 (Authentication)**:
- [ ] Sign up with email/password
- [ ] Complete background questionnaire
- [ ] Sign in with credentials
- [ ] View profile, verify all data stored

**P4 (Personalization)**:
- [ ] Create account with "Beginner" ROS 2 experience
- [ ] Personalize a chapter, note content
- [ ] Create second account with "Advanced" experience
- [ ] Personalize same chapter, verify 30% variation

**P5 (Translation)**:
- [ ] Sign in
- [ ] Click "Translate to Urdu" on any chapter
- [ ] Verify Urdu content with RTL rendering
- [ ] Verify code blocks remain in English
- [ ] Toggle back to English

**P6 (Skills)**:
- [ ] Open chat widget
- [ ] Type `/hardware Jetson Orin Nano`
- [ ] Verify specs displayed in card format
- [ ] Type `/ros2 launch a LiDAR sensor node`
- [ ] Verify ROS 2 command generated

## 📈 Performance Targets

- **Page Load**: <3 seconds (p95)
- **Chatbot Latency**: <2 seconds (p95)
- **Translation**: <5 seconds (chapters up to 5000 words)
- **Concurrent Users**: 100 without degradation
- **Build Time**: <5 minutes (Vercel deployment)

## 🔒 Security

- Passwords hashed with bcrypt (12+ rounds)
- JWT tokens with 1-hour expiration
- Rate limiting (100 req/min per user)
- CORS restricted to Vercel frontend domain
- Input validation on all API endpoints (Pydantic)
- XSS prevention (React auto-escaping + CSP headers)
- No secrets in code (environment variables only)

## 📝 License

This project is created for a hackathon submission.

## 🙏 Acknowledgments

- Docusaurus team for excellent documentation framework
- OpenAI for GPT-4 and embedding APIs
- Neon for serverless Postgres
- Qdrant for vector search
- Vercel and Railway for deployment platforms

---

**Built with Claude Code using Spec-Driven Development** 🤖
