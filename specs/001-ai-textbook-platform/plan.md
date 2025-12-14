# Implementation Plan: AI-Native Physical AI & Humanoid Robotics Textbook Platform

**Branch**: `001-ai-textbook-platform` | **Date**: 2025-12-14 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/001-ai-textbook-platform/spec.md`

**Note**: This plan follows the 5-phase approach specified by the user and aligns with the project constitution.

## Summary

Building an AI-native Docusaurus textbook platform with integrated RAG chatbot, user authentication, content personalization, and multilingual support. The system enables students to learn Physical AI & Humanoid Robotics through an interactive textbook with contextual AI assistance that answers questions based on highlighted text, personalizes content based on user background, and translates to Urdu. The platform targets 300 total hackathon points (100 core + 200 bonus) through systematic delivery of 6 prioritized features.

**Technical Approach**:
- Frontend: Docusaurus (React) with OpenAI ChatKit SDK for chat widget
- Backend: FastAPI (Python) with OpenAI Agents SDK for RAG orchestration
- Data Layer: Neon Serverless Postgres (relational data) + Qdrant Cloud (vector embeddings)
- Auth: Better-Auth with background questionnaire integration
- Deployment: Vercel (frontend + serverless backend functions)

## Technical Context

**Language/Version**:
- Frontend: TypeScript 5.x, Node.js 18.x (Docusaurus 3.x requirement)
- Backend: Python 3.10+ (for modern type hints and async/await)

**Primary Dependencies**:
- Frontend: Docusaurus 3.x, React 18.x, OpenAI ChatKit SDK, Better-Auth (client), Zustand (state management), Axios (HTTP client)
- Backend: FastAPI 0.109+, OpenAI Agents SDK, Pydantic 2.x, SQLAlchemy 2.x (async), Alembic (migrations), python-jose (JWT), passlib+bcrypt (password hashing), qdrant-client, psycopg2-binary (Neon Postgres)

**Storage**:
- Relational: Neon Serverless Postgres (users, chat sessions, messages, personalization preferences, translation cache)
- Vector: Qdrant Cloud free tier (textbook content embeddings for RAG)
- Static Assets: Vercel CDN (Docusaurus build output)

**Testing**:
- Frontend: Jest + React Testing Library (component tests), Playwright (E2E tests)
- Backend: pytest + pytest-asyncio (async tests), httpx (async TestClient for FastAPI)
- RAG Validation: Custom embedding quality tests, retrieval precision/recall metrics

**Target Platform**:
- Frontend: Modern browsers (Chrome 90+, Firefox 88+, Safari 14+, Edge 90+)
- Backend: Vercel Serverless Functions (Node.js 18.x runtime for Python via custom build)
- Database: Neon Serverless Postgres (cloud-hosted, no local server)
- Vector Store: Qdrant Cloud (cloud-hosted, no local server)

**Project Type**: Web application (frontend + backend)

**Performance Goals**:
- Page Load: <3 seconds for chapter pages (p95)
- Chatbot Latency: <2 seconds for RAG queries (p95)
- Translation: <5 seconds for chapters up to 5000 words
- Concurrent Users: 100 users without degradation
- Build Time: <5 minutes for Vercel deployment

**Constraints**:
- Free Tier Limits: Neon (0.5GB storage, 100 hours compute/month), Qdrant (1GB cluster)
- Vercel Limits: 100GB bandwidth/month, 100 serverless function invocations/day (Hobby tier)
- OpenAI API: Rate limits apply (3,500 RPM for GPT-4, 10,000 RPM for embeddings on Tier 1)
- Deployment Target: Vercel ONLY (GitHub Pages explicitly forbidden per constitution)
- Authentication: Must use Better-Auth (no Firebase, Auth0, or custom JWT solution)

**Scale/Scope**:
- Content: ~50,000-100,000 words across 10-15 chapters
- Users: 100-500 during hackathon evaluation period
- Embeddings: ~500-1000 chunks (512 tokens each) in Qdrant
- Chat Sessions: ~1000-5000 messages total during evaluation
- Codebase: Estimated 5,000-8,000 LOC (3,000 frontend TypeScript, 2,500 backend Python, 2,500 docs/config)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Technology Stack Adherence (Constitution Section II)

- ✅ **Frontend**: Docusaurus (React-based) ✓ TypeScript/JavaScript ✓ OpenAI ChatKit SDK ✓
- ✅ **Backend**: FastAPI ✓ Python 3.10+ ✓ OpenAI Agents SDK ✓
- ✅ **Database**: Neon Serverless Postgres ✓
- ✅ **Vector Store**: Qdrant Cloud (free tier) ✓
- ✅ **Authentication**: Better-Auth ✓
- ✅ **Deployment**: Vercel ✓ (GitHub Pages excluded ✓)

**Status**: PASS - All stack requirements met per constitution mandate.

### Feature Completeness (Constitution Section III)

- ✅ **Core Features**: Docusaurus textbook ✓ RAG chatbot ✓ User authentication ✓ Personalization ✓
- ✅ **50-Point Bonuses**:
  - Better-Auth with questionnaire ✓ (P3)
  - Content personalization ✓ (P4)
  - Urdu translation ✓ (P5)
  - Claude Code subagents/skills ✓ (P6)

**Status**: PASS - All features planned for implementation.

### Code Quality Standards (Constitution Section IV)

- ✅ **Frontend**: TypeScript strict mode in plan ✓ ESLint + Prettier (standard Docusaurus setup) ✓ React best practices ✓ Error boundaries + loading states ✓ WCAG 2.1 AA accessibility ✓
- ✅ **Backend**: Type hints ✓ PEP 8 (Black + Flake8) ✓ Async/await for I/O ✓ Custom exceptions ✓ Structured logging (JSON) ✓
- ✅ **General**: No secrets in code (.env) ✓ Separation of concerns ✓ Self-documenting code ✓ DRY principle ✓

**Status**: PASS - Quality tooling and practices planned.

### Integration Architecture (Constitution Section V)

- ✅ **API Contract**: RESTful endpoints (`/api/chat`, `/api/auth`, `/api/personalize`, `/api/translate`) ✓ OpenAPI docs (FastAPI auto-generation) ✓ CORS for Vercel ✓ JWT headers ✓
- ✅ **State Management**: React Context/Zustand for frontend ✓ Postgres for backend sessions ✓ Qdrant for vectors ✓
- ✅ **Error Handling**: Unified format `{error, code, details}` ✓ User-friendly frontend messages ✓ Backend stack traces logged ✓

**Status**: PASS - Integration contracts defined.

### Testing & Validation (Constitution Section VI)

- ✅ **Frontend**: Jest/Vitest unit tests ✓ React Testing Library component tests ✓ Playwright E2E ✓
- ✅ **Backend**: pytest unit tests ✓ httpx integration tests ✓ Contract tests (OpenAPI validation) ✓
- ✅ **RAG**: Embedding quality ✓ Retrieval accuracy ✓ Response evaluation ✓
- ✅ **Deployment**: Smoke tests in staging ✓ Performance benchmarks (p95 <2s) ✓

**Status**: PASS - Testing strategy defined.

### Deployment & DevOps (Constitution Section VII)

- ✅ **Build**: Docusaurus static export ✓ FastAPI as Vercel serverless ✓ Env vars in Vercel dashboard ✓ Build <5 min ✓
- ✅ **Monitoring**: Vercel Analytics ✓ Backend JSON logging ✓ Error tracking (optional Sentry) ✓
- ✅ **Rollback**: Git-tagged deployments ✓ Vercel instant rollback ✓ Reversible migrations ✓

**Status**: PASS - Deployment strategy Vercel-optimized.

### Security & Privacy (Constitution Section VIII)

- ✅ **Auth Security**: bcrypt (12+ rounds) ✓ JWT (1-hour expiry) ✓ HTTPS ✓ CSRF protection ✓
- ✅ **Data Privacy**: Row-level user isolation ✓ No PII in logs ✓ GDPR export/deletion ✓
- ✅ **API Security**: Rate limiting (100 req/min) ✓ Pydantic input validation ✓ Parameterized queries ✓ React XSS prevention + CSP ✓

**Status**: PASS - Security measures planned.

### Simplicity & Pragmatism (Constitution Section IX)

- ✅ **YAGNI**: Build only hackathon requirements ✓ No speculative features ✓ No premature abstractions ✓
- ✅ **Smallest Change**: Focused refactoring ✓ No unrelated improvements ✓ Single-story PRs ✓
- ✅ **Tech Minimalism**: Docusaurus built-ins ✓ Monolithic backend ✓ No microservices ✓

**Status**: PASS - Complexity minimized.

### Overall Constitution Compliance

**✅ ALL GATES PASSED** - No violations. No complexity tracking required.

## Project Structure

### Documentation (this feature)

```text
specs/001-ai-textbook-platform/
├── spec.md              # Feature specification (completed)
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0: Technology research and decisions
├── data-model.md        # Phase 1: Database schema and entities
├── quickstart.md        # Phase 1: Developer setup guide
├── contracts/           # Phase 1: API contracts (OpenAPI specs)
│   ├── auth.openapi.yaml
│   ├── chat.openapi.yaml
│   ├── personalization.openapi.yaml
│   └── translation.openapi.yaml
├── checklists/
│   └── requirements.md  # Specification quality checklist (completed)
└── tasks.md             # Phase 2: Implementation tasks (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Web application structure (frontend + backend)

# === FRONTEND (Docusaurus) ===
frontend/
├── docusaurus.config.js         # Docusaurus configuration
├── sidebars.js                  # Sidebar navigation structure
├── package.json                 # Node.js dependencies
├── tsconfig.json                # TypeScript configuration (strict mode)
├── .eslintrc.js                 # ESLint rules
├── .prettierrc                  # Prettier formatting
├── docs/                        # Markdown textbook content
│   ├── intro.md
│   ├── quarter-overview.md
│   ├── week-01-02/
│   │   ├── index.md
│   │   └── module-details.md
│   ├── hardware-requirements.md
│   └── ...                      # Additional chapters
├── src/
│   ├── components/              # React components
│   │   ├── ChatWidget/          # RAG chatbot UI
│   │   │   ├── ChatWidget.tsx
│   │   │   ├── MessageList.tsx
│   │   │   ├── InputBox.tsx
│   │   │   └── HighlightDetector.tsx
│   │   ├── Auth/                # Authentication UI
│   │   │   ├── SignUpForm.tsx
│   │   │   ├── SignInForm.tsx
│   │   │   ├── BackgroundQuestionnaire.tsx
│   │   │   └── ProfileView.tsx
│   │   ├── Personalization/     # Content personalization
│   │   │   ├── PersonalizeButton.tsx
│   │   │   └── PersonalizedContent.tsx
│   │   ├── Translation/         # Urdu translation
│   │   │   ├── TranslateButton.tsx
│   │   │   └── UrduContent.tsx
│   │   └── Skills/              # Claude Code skills integration
│   │       ├── HardwareSpecLookup.tsx
│   │       └── ROS2CommandGenerator.tsx
│   ├── services/                # API client services
│   │   ├── chatService.ts       # RAG API calls
│   │   ├── authService.ts       # Auth API calls
│   │   ├── personalizationService.ts
│   │   └── translationService.ts
│   ├── store/                   # Zustand state management
│   │   ├── authStore.ts         # User auth state
│   │   ├── chatStore.ts         # Chat session state
│   │   └── uiStore.ts           # UI preferences
│   ├── types/                   # TypeScript type definitions
│   │   ├── api.ts
│   │   ├── user.ts
│   │   └── chat.ts
│   └── theme/                   # Custom Docusaurus theme
│       └── Root.tsx             # Global providers (auth, chat)
├── static/                      # Static assets
│   └── img/
└── tests/
    ├── unit/                    # Jest unit tests
    ├── component/               # React Testing Library
    └── e2e/                     # Playwright E2E tests

# === BACKEND (FastAPI) ===
backend/
├── main.py                      # FastAPI app entry point
├── requirements.txt             # Python dependencies
├── pyproject.toml               # Poetry/pip configuration
├── alembic.ini                  # Database migration config
├── .env.example                 # Environment variable template
├── src/
│   ├── api/                     # API endpoints
│   │   ├── __init__.py
│   │   ├── auth.py              # POST /api/auth/signup, /api/auth/signin
│   │   ├── chat.py              # POST /api/chat/query, GET /api/chat/history
│   │   ├── personalization.py  # POST /api/personalize/chapter
│   │   ├── translation.py       # POST /api/translate/chapter
│   │   └── skills.py            # POST /api/skills/hardware, /api/skills/ros2
│   ├── models/                  # SQLAlchemy ORM models
│   │   ├── __init__.py
│   │   ├── user.py              # User entity
│   │   ├── chat.py              # ChatSession, ChatMessage
│   │   ├── personalization.py  # PersonalizationPreference
│   │   └── translation.py       # TranslationCache
│   ├── services/                # Business logic
│   │   ├── __init__.py
│   │   ├── rag_service.py       # RAG orchestration (OpenAI Agents SDK)
│   │   ├── embedding_service.py # Vector embedding generation
│   │   ├── auth_service.py      # Better-Auth integration
│   │   ├── personalization_service.py # Content adaptation logic
│   │   └── translation_service.py # Urdu translation (OpenAI API)
│   ├── database/                # Database utilities
│   │   ├── __init__.py
│   │   ├── postgres.py          # Neon Postgres connection (async)
│   │   ├── qdrant.py            # Qdrant client wrapper
│   │   └── session.py           # SQLAlchemy session management
│   ├── schemas/                 # Pydantic request/response models
│   │   ├── __init__.py
│   │   ├── auth.py              # SignUpRequest, SignInResponse, etc.
│   │   ├── chat.py              # ChatQueryRequest, ChatResponse, etc.
│   │   ├── personalization.py
│   │   └── translation.py
│   ├── core/                    # Core configuration
│   │   ├── __init__.py
│   │   ├── config.py            # Settings (env vars, secrets)
│   │   ├── security.py          # JWT, password hashing
│   │   └── exceptions.py        # Custom exception classes
│   └── utils/                   # Utility functions
│       ├── __init__.py
│       ├── logging.py           # Structured JSON logging
│       └── validation.py        # Input sanitization
├── alembic/                     # Database migrations
│   └── versions/
├── scripts/                     # Utility scripts
│   ├── ingest_content.py        # Load Docusaurus content into Qdrant
│   └── create_admin.py          # Admin user creation
└── tests/
    ├── unit/                    # pytest unit tests
    ├── integration/             # API endpoint integration tests
    └── contract/                # OpenAPI contract validation tests

# === DEPLOYMENT (Vercel) ===
vercel.json                      # Vercel deployment configuration
.vercelignore                    # Files to exclude from deployment

# === ROOT ===
.gitignore
README.md
LICENSE
```

**Structure Decision**: Selected **Option 2: Web application** structure with separate `frontend/` (Docusaurus) and `backend/` (FastAPI) directories. This separation enables:
1. Independent frontend and backend deployments to Vercel
2. Clear technology boundaries (TypeScript vs Python)
3. Parallel development of P1-P2 (frontend) and P2-P6 (backend with frontend integration)
4. Easier testing (frontend tests don't require backend, backend tests don't require Docusaurus build)

## Complexity Tracking

**No violations** - All constitution checks passed. No complexity tracking required.

## Implementation Phases

### Phase 0: Research & Technology Validation

**Objective**: Resolve all technical unknowns and validate technology integration feasibility before design.

**Research Tasks**:
1. **Vercel Serverless for FastAPI**: Investigate deployment patterns for FastAPI on Vercel (Python runtime support, serverless function limitations, cold start times)
2. **Better-Auth + Neon Integration**: Research Better-Auth adapter patterns for PostgreSQL, validate custom background questionnaire during sign-up flow
3. **OpenAI ChatKit SDK**: Evaluate ChatKit SDK vs custom React chat component (bundle size, customization flexibility, highlight detection support)
4. **OpenAI Agents SDK**: Research RAG orchestration patterns with Agents SDK (context management, multi-turn conversations, retrieval strategies)
5. **Qdrant Cloud Setup**: Validate free tier limits (1GB cluster), connection patterns from Vercel serverless functions, embedding dimensionality (OpenAI text-embedding-3-small: 1536 dimensions)
6. **Docusaurus Theme Customization**: Research swizzling patterns for embedding persistent chat widget, authentication state providers
7. **Urdu Translation**: Validate OpenAI GPT-4 translation quality for technical content, RTL rendering in Docusaurus/React
8. **Claude Code Skills**: Research Claude Code SDK patterns for custom agent skills (if SDK available) or document as standalone callable functions

**Output**: [research.md](research.md) with decisions, rationale, and alternatives for each research task.

**Completion Criteria**: All "NEEDS CLARIFICATION" items resolved with concrete technical decisions.

### Phase 1: Docusaurus & Content Foundation (Frontend)

**Objective**: Establish the textbook platform foundation (P1 - MVP).

**Tasks**:
1. **Scaffold Docusaurus Project**:
   - Run `npx create-docusaurus@latest frontend classic --typescript`
   - Configure `docusaurus.config.js` for Vercel deployment (base URL, static hosting)
   - Set up TypeScript strict mode in `tsconfig.json`
   - Configure ESLint + Prettier

2. **Integrate Course Content**:
   - Convert "Physical AI & Humanoid Robotics Course Details" to Markdown files in `frontend/docs/`
   - Structure content hierarchy: Quarter Overview, Weeks 1-10, Hardware Requirements, Learning Outcomes
   - Configure `sidebars.js` to match course outline structure
   - Add module metadata (estimated reading time, difficulty level)

3. **Theme Customization**:
   - Swizzle Docusaurus theme for custom layout (add chat widget placeholder)
   - Implement responsive design for mobile (320px), tablet (768px), desktop (1920px+)
   - Apply professional technical textbook styling (clean, minimal, readable typography)
   - Add accessibility features (semantic HTML, ARIA labels, keyboard navigation)

4. **Vercel Deployment Config**:
   - Create `vercel.json` with build commands (`docusaurus build`)
   - Configure environment variables (API base URL for backend)
   - Set up preview deployments for branches
   - Test deployment with static content

**Artifacts**:
- Working Docusaurus site deployed to Vercel
- All course content accessible via sidebar navigation
- Responsive design validated on 3 device sizes
- Lighthouse accessibility score >90

**Dependencies**: None (foundation phase)

**Estimated Effort**: 8-12 hours

### Phase 2: RAG Backend Infrastructure

**Objective**: Set up backend services and data ingestion for RAG (P2 core feature).

**Tasks**:
1. **FastAPI Project Setup**:
   - Initialize FastAPI project in `backend/`
   - Configure async SQLAlchemy for Neon Postgres (connection pooling, async sessions)
   - Set up Alembic for database migrations
   - Implement structured JSON logging (Python `logging` module with JSON formatter)

2. **Database Schema**:
   - Create SQLAlchemy models: `User`, `ChatSession`, `ChatMessage`, `PersonalizationPreference`, `TranslationCache`
   - Define relationships (User → ChatSessions → ChatMessages, User → PersonalizationPreferences)
   - Add indexes for common queries (user_id, session_id, chapter_id)
   - Generate and apply initial migration

3. **Qdrant Integration**:
   - Set up Qdrant Cloud cluster (free tier)
   - Configure qdrant-client in backend
   - Define collection schema for textbook embeddings (vector dimension: 1536, metadata: chapter_id, section, position)
   - Implement connection pooling for serverless environment

4. **Data Ingestion Script**:
   - Create `scripts/ingest_content.py` to:
     - Read all Markdown files from `frontend/docs/`
     - Chunk content into 512-token segments (with 50-token overlap)
     - Generate embeddings using OpenAI `text-embedding-3-small`
     - Upload to Qdrant with metadata (chapter ID, title, section heading)
   - Add progress logging and error handling
   - Store ingestion manifest (chunk count, total tokens, timestamp)

5. **Environment Configuration**:
   - Create `.env.example` with all required variables:
     - `NEON_DATABASE_URL` (Postgres connection string)
     - `QDRANT_URL`, `QDRANT_API_KEY`
     - `OPENAI_API_KEY`
     - `JWT_SECRET`
   - Document setup process in `backend/README.md`

**Artifacts**:
- FastAPI app running locally with Neon + Qdrant connections
- Database schema created with all tables
- Textbook content ingested into Qdrant (500-1000 embeddings)
- Ingestion script runnable via `python scripts/ingest_content.py`

**Dependencies**: Phase 1 (content must exist in Docusaurus)

**Estimated Effort**: 10-14 hours

### Phase 3: RAG & Chatbot Integration

**Objective**: Implement core RAG functionality and frontend chat widget (P2 core feature).

**Tasks**:
1. **RAG Service Implementation** (Backend):
   - Implement `rag_service.py` using OpenAI Agents SDK:
     - Contextual query: Use highlighted text as exclusive context (no vector search)
     - General query: Embed query → retrieve top-5 chunks from Qdrant → pass to LLM
     - Multi-turn: Maintain conversation history in memory (store in `ChatMessage` table)
   - Add retrieval strategy (semantic search with similarity threshold >0.7)
   - Implement response streaming (SSE for real-time answers)
   - Add fallback handling (no relevant content found → "Topic not covered in textbook")

2. **Chat API Endpoints** (Backend):
   - `POST /api/chat/query`: Accept `{query, highlighted_text?, session_id?}` → return `{answer, sources[], latency_ms}`
   - `GET /api/chat/history?session_id={id}`: Return conversation history
   - `POST /api/chat/session`: Create new chat session
   - Add request validation (Pydantic schemas)
   - Implement rate limiting (100 req/min per IP using slowapi)

3. **Chat Widget Frontend** (Docusaurus):
   - Install OpenAI ChatKit SDK (or build custom if SDK unsuitable per research)
   - Implement `ChatWidget.tsx`:
     - Persistent chat panel (collapsible, bottom-right corner)
     - Message list with user/assistant bubbles
     - Input box with send button
     - Loading states (typing indicator)
   - Implement `HighlightDetector.tsx`:
     - Listen for `window.getSelection()` events
     - Capture highlighted text on mouseup
     - Display highlighted context in chat input (e.g., "Ask about: [first 50 chars]...")
   - Add chat state management (Zustand store for messages, session ID)

4. **API Integration**:
   - Implement `chatService.ts`:
     - `sendMessage(query, highlightedText?, sessionId?)` → POST to `/api/chat/query`
     - `getChatHistory(sessionId)` → GET from `/api/chat/history`
     - `createSession()` → POST to `/api/chat/session`
   - Handle CORS (backend allows Vercel frontend domain)
   - Add error handling (network errors, API failures → user-friendly messages)

5. **Testing**:
   - Unit tests for `rag_service.py` (mock Qdrant, validate retrieval logic)
   - Integration tests for chat API endpoints (pytest with TestClient)
   - Component tests for `ChatWidget.tsx` (React Testing Library)
   - E2E test: Highlight text → ask question → verify response uses highlighted context

**Artifacts**:
- Working RAG chatbot on all Docusaurus pages
- Contextual queries respond using only highlighted text
- General queries retrieve relevant textbook content
- Chat history persisted in database
- P95 latency <2 seconds (measure with backend logging)

**Dependencies**: Phase 2 (RAG infrastructure + content ingestion)

**Estimated Effort**: 16-20 hours

### Phase 4: Authentication & Personalization Features

**Objective**: Implement user authentication with background questionnaire (P3) and content personalization (P4).

**Tasks**:
1. **Better-Auth Integration** (Backend + Frontend):
   - Install Better-Auth in both frontend and backend
   - Configure Better-Auth provider for Neon Postgres
   - Implement custom sign-up flow:
     - Step 1: Email, password, full name (standard Better-Auth)
     - Step 2: Background questionnaire (custom form)
       - ROS 2 experience (None/Beginner/Intermediate/Advanced) - dropdown
       - GPU model + VRAM (text input, e.g., "NVIDIA RTX 3060, 12GB") - free text
       - Operating system (Ubuntu/Windows/macOS) - dropdown
       - Robotics knowledge (None/Beginner/Intermediate/Advanced) - dropdown
   - Store questionnaire responses in `User` table (extend Better-Auth user model)

2. **Auth API Endpoints** (Backend):
   - `POST /api/auth/signup`: Create user + store background data → return JWT
   - `POST /api/auth/signin`: Authenticate → return JWT
   - `GET /api/auth/profile`: Return user profile + background data (JWT required)
   - `PUT /api/auth/profile`: Update background data (JWT required)
   - Implement JWT middleware for protected endpoints

3. **Auth UI Components** (Frontend):
   - `SignUpForm.tsx`: Email, password, name fields → submit → redirect to questionnaire
   - `BackgroundQuestionnaire.tsx`: 4 fields (ROS 2 experience, GPU, OS, robotics knowledge) → submit → redirect to dashboard
   - `SignInForm.tsx`: Email, password → submit → store JWT in localStorage → redirect to last page
   - `ProfileView.tsx`: Display user info + background data → edit button → update via API
   - Add auth state to Zustand (`authStore.ts`: user, token, isAuthenticated)

4. **Personalization Service** (Backend):
   - Implement `personalization_service.py`:
     - Accept `{user_id, chapter_content, personalization_level}` (beginner/standard/advanced)
     - Use OpenAI GPT-4 to rewrite content:
       - Beginner: Expand introductions, simplify jargon, add beginner examples
       - Advanced: Condense basics, emphasize advanced topics, add complex examples
     - Return personalized content as Markdown
   - Add caching in `PersonalizationPreference` table (avoid redundant LLM calls)

5. **Personalization API** (Backend):
   - `POST /api/personalize/chapter`: Accept `{chapter_id}` (JWT required) → return personalized content
   - Retrieve user background from JWT → determine personalization level (ROS 2 experience maps to beginner/advanced)
   - Fetch chapter content from Docusaurus (or pre-cached in database)
   - Call personalization service → return result

6. **Personalization UI** (Frontend):
   - Add `PersonalizeButton.tsx` to each chapter page (only visible when authenticated)
   - On click: Call `/api/personalize/chapter` → replace chapter content with personalized version
   - Add loading state ("Personalizing chapter...")
   - Store personalization preference in backend (`PersonalizationPreference` table)
   - On page load: Check if personalized version exists → auto-load if yes

7. **Testing**:
   - Unit tests for `personalization_service.py` (mock OpenAI, validate content transformation)
   - Integration tests for auth endpoints (signup → signin → profile)
   - E2E test: Sign up with "Beginner" ROS 2 experience → personalize chapter → verify expanded content

**Artifacts**:
- Working sign-up/sign-in flows with background questionnaire
- JWT-based authentication protecting personalization features
- "Personalize This Chapter" button on authenticated pages
- Personalized content differs for Beginner vs Advanced users (validate 30% variation)
- Profile view allows editing background data

**Dependencies**: Phase 3 (chat widget integrated, can test with authenticated users)

**Estimated Effort**: 14-18 hours

### Phase 5: Urdu Translation & Final Touches

**Objective**: Implement Urdu translation (P5), Claude Code skills (P6), and deploy to Vercel.

**Tasks**:
1. **Translation Service** (Backend):
   - Implement `translation_service.py`:
     - Accept `{chapter_content, target_language='ur'}` (Urdu)
     - Use OpenAI GPT-4 for translation (prompt: "Translate the following technical content to Urdu. Preserve code blocks as-is. Transliterate technical terms.")
     - Return translated content with RTL markers
   - Add caching in `TranslationCache` table (avoid redundant translations)

2. **Translation API** (Backend):
   - `POST /api/translate/chapter`: Accept `{chapter_id, target_language}` (JWT required) → return translated content
   - Fetch chapter content → call translation service → return result
   - Add error handling (translation API failures → keep English, show error message)

3. **Translation UI** (Frontend):
   - Add `TranslateButton.tsx` to each chapter page (only visible when authenticated)
   - On click: Call `/api/translate/chapter` with `target_language='ur'` → replace content with Urdu version
   - Implement RTL rendering (CSS: `direction: rtl`, `text-align: right`)
   - Preserve code blocks in original form (no translation)
   - Add "Show Original English" toggle button

4. **Claude Code Skills Implementation** (P6):
   - Research Claude Code SDK (if available) or implement as standalone callable functions
   - Implement `Hardware_Spec_Lookup` skill:
     - Accept component name (e.g., "NVIDIA Jetson Orin Nano")
     - Return specs (CPU, GPU, RAM, power, price, availability)
     - Use hardcoded database or external API (e.g., PCPartPicker if accessible)
   - Implement `ROS2_Command_Generator` skill:
     - Accept task description (e.g., "launch a LiDAR sensor node")
     - Generate ROS 2 command (e.g., `ros2 launch sensor_pkg lidar.launch.py`)
     - Use template-based generation or LLM call
   - Add API endpoints: `POST /api/skills/hardware`, `POST /api/skills/ros2`
   - Integrate with chat widget (special syntax like `/hardware Jetson Orin` or `/ros2 launch lidar`)

5. **Skills UI Integration** (Frontend):
   - Add skill invocation buttons in chat widget (e.g., "🔧 Hardware Specs", "🤖 ROS2 Commands")
   - On click: Send special query to chat API → backend routes to appropriate skill
   - Display skill results in chat (formatted as structured data, not plain text)
   - Document skills in textbook (add "Using AI Skills" page)

6. **Vercel Deployment Finalization**:
   - Configure `vercel.json` for both frontend and backend:
     - Frontend: Docusaurus build output from `frontend/build/`
     - Backend: Serverless functions from `backend/` (if Vercel supports Python serverless, otherwise deploy to separate Vercel project or Railway/Render and configure CORS)
   - Set up environment variables in Vercel dashboard (all secrets, API keys, database URLs)
   - Configure custom domain (if available) or use Vercel subdomain
   - Test full deployment: Frontend + Backend + Database + Vector Store connections

7. **Final Testing & Polish**:
   - Run full E2E test suite (Playwright):
     - P1: Navigate chapters, verify responsive design
     - P2: Chat with RAG, verify contextual queries
     - P3: Sign up, sign in, view profile
     - P4: Personalize chapter, verify content differs
     - P5: Translate to Urdu, verify RTL rendering
     - P6: Invoke skills, verify results
   - Performance testing: Load test with 100 concurrent users (Artillery or k6)
   - Accessibility audit: Run Lighthouse, fix issues to meet WCAG 2.1 AA
   - Security audit: Test rate limiting, SQL injection prevention, XSS prevention
   - Documentation: Update README with setup instructions, architecture diagram, API docs

8. **Hackathon Submission Prep**:
   - Record demo video showing all features (P1-P6)
   - Create presentation deck (problem, solution, tech stack, demo, impact)
   - Document point breakdown (100 core + 200 bonus = 300 total)
   - Prepare code walkthrough for judges

**Artifacts**:
- Urdu translation functional on all chapters
- "Translate to Urdu" button with RTL rendering
- Hardware_Spec_Lookup and ROS2_Command_Generator skills working
- Fully deployed to Vercel (frontend + backend)
- All features tested end-to-end
- Demo video and presentation ready

**Dependencies**: Phase 4 (authentication required for translation)

**Estimated Effort**: 16-20 hours

## Total Estimated Effort

- Phase 0: Research & Validation: 4-6 hours
- Phase 1: Docusaurus Foundation: 8-12 hours
- Phase 2: RAG Backend: 10-14 hours
- Phase 3: RAG & Chat Integration: 16-20 hours
- Phase 4: Auth & Personalization: 14-18 hours
- Phase 5: Translation & Deployment: 16-20 hours

**Total: 68-90 hours** (approximately 9-12 full working days for a single developer, or 5-7 days for a 2-person team working in parallel)

**Critical Path**: Phase 1 → Phase 2 → Phase 3 (MVP) [~34-46 hours]
**Parallelizable**: Phase 4, Phase 5 features can be developed in parallel after Phase 3 completes

## Risk Analysis & Mitigation

### High-Risk Items

1. **Vercel Serverless Python Support**:
   - **Risk**: Vercel may not fully support Python serverless functions (cold starts, limited runtime)
   - **Mitigation**: Research alternative: Deploy FastAPI to Railway/Render/Fly.io, configure CORS for Vercel frontend. Test deployment early in Phase 2.
   - **Fallback**: Use Vercel Edge Functions (Node.js) as proxy to external FastAPI deployment

2. **Better-Auth + Neon Integration**:
   - **Risk**: Better-Auth may not have official Neon Postgres adapter, requiring custom implementation
   - **Mitigation**: Research Better-Auth database adapter patterns in Phase 0. If complex, consider alternative: NextAuth.js or Auth.js (similar libraries with Postgres support)
   - **Fallback**: Implement custom JWT authentication without Better-Auth library (violates constitution, requires approval)

3. **OpenAI ChatKit SDK Compatibility**:
   - **Risk**: ChatKit SDK may not support highlight detection or custom UI requirements
   - **Mitigation**: Evaluate SDK in Phase 0. If unsuitable, build custom React chat component (increases effort by ~4 hours)
   - **Fallback**: Use lightweight chat library (react-chat-widget) + custom highlight logic

4. **Free Tier Limits (Neon, Qdrant, OpenAI)**:
   - **Risk**: Exceeding free tier limits during hackathon evaluation (100-500 users)
   - **Mitigation**: Monitor usage closely. Implement aggressive caching (translation, personalization). Set rate limits to prevent abuse.
   - **Fallback**: Upgrade to paid tier if necessary (budget $20-50 for hackathon period)

### Medium-Risk Items

5. **Urdu Translation Quality**:
   - **Risk**: OpenAI GPT-4 may produce poor quality Urdu translations for technical content
   - **Mitigation**: Test translation early in Phase 5 with sample chapters. Iterate prompts to improve quality.
   - **Fallback**: Use Google Translate API as backup (lower quality but functional)

6. **Performance (p95 <2s chatbot latency)**:
   - **Risk**: RAG queries may exceed 2-second latency due to embedding generation, Qdrant search, LLM inference
   - **Mitigation**: Optimize retrieval (reduce top-k chunks, use faster embedding model). Cache frequent queries.
   - **Fallback**: Relax success criteria to p95 <3s (document rationale)

### Low-Risk Items

7. **Content Conversion to Markdown**:
   - **Risk**: Provided content may not be in suitable format for Docusaurus
   - **Mitigation**: Manual conversion in Phase 1 (8-12 hours budgeted). Use tools like Pandoc for bulk conversion.
   - **Fallback**: Simplify content structure if conversion is too complex

8. **Claude Code Skills SDK Availability**:
   - **Risk**: Claude Code SDK may not be publicly available or documented
   - **Mitigation**: Implement skills as standalone API endpoints callable from chat widget (simpler than SDK integration)
   - **Fallback**: Document skills as "demonstrable" in code/documentation rather than runtime-invocable

## Next Steps

1. **Execute Phase 0**: Generate [research.md](research.md) by researching all 8 technology unknowns
2. **Execute Phase 1 Design**: Generate [data-model.md](data-model.md), [contracts/](contracts/), [quickstart.md](quickstart.md)
3. **Re-validate Constitution Check** after Phase 1 design decisions
4. **Proceed to `/sp.tasks`**: Generate actionable task list organized by priority (P1-P6) with parallelization opportunities

**Current Status**: ✅ Plan complete, ready for Phase 0 research.
