# Tasks: AI-Native Physical AI & Humanoid Robotics Textbook Platform

**Input**: Design documents from `/specs/001-ai-textbook-platform/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md (decisions)

**Tests**: Tests are OPTIONAL for this project unless explicitly requested. Focus on functional delivery for hackathon.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `frontend/` (Docusaurus), `backend/` (FastAPI)
- Frontend paths: `frontend/src/`, `frontend/docs/`
- Backend paths: `backend/src/`, `backend/scripts/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Initialize Git repository with `.gitignore` (exclude `node_modules/`, `__pycache__/`, `.env`, `build/`)
- [ ] T002 Create root project structure: `frontend/`, `backend/`, `vercel.json`, `README.md`
- [ ] T003 [P] Scaffold Docusaurus project: Run `npx create-docusaurus@latest frontend classic --typescript`
- [ ] T004 [P] Initialize FastAPI project structure in `backend/` (create `main.py`, `requirements.txt`, `src/`, `tests/`)
- [ ] T005 [P] Configure TypeScript strict mode in `frontend/tsconfig.json`
- [ ] T006 [P] Set up ESLint and Prettier for frontend in `frontend/.eslintrc.js` and `frontend/.prettierrc`
- [ ] T007 [P] Create Python project config `backend/pyproject.toml` with Black, Flake8, mypy settings
- [ ] T008 [P] Create `.env.example` files for frontend (`frontend/.env.example`) and backend (`backend/.env.example`)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T009 Configure Neon Serverless Postgres connection in `backend/src/database/postgres.py` (async SQLAlchemy engine)
- [ ] T010 [P] Configure Qdrant Cloud connection in `backend/src/database/qdrant.py` (qdrant-client with API key)
- [ ] T011 [P] Create Alembic migration setup in `backend/alembic.ini` and `backend/alembic/env.py`
- [ ] T012 Create database models in `backend/src/models/`:
  - `backend/src/models/user.py` (User entity with background fields)
  - `backend/src/models/chat.py` (ChatSession, ChatMessage)
  - `backend/src/models/personalization.py` (PersonalizationPreference)
  - `backend/src/models/translation.py` (TranslationCache)
- [ ] T013 Generate initial Alembic migration: `alembic revision --autogenerate -m "Initial schema"`
- [ ] T014 Apply database migration: `alembic upgrade head`
- [ ] T015 [P] Create Pydantic schemas in `backend/src/schemas/` for request/response validation:
  - `backend/src/schemas/auth.py` (SignUpRequest, SignInRequest, UserProfile)
  - `backend/src/schemas/chat.py` (ChatQueryRequest, ChatResponse, ChatHistory)
  - `backend/src/schemas/personalization.py` (PersonalizeRequest, PersonalizeResponse)
  - `backend/src/schemas/translation.py` (TranslateRequest, TranslateResponse)
- [ ] T016 [P] Implement core utilities in `backend/src/core/`:
  - `backend/src/core/config.py` (load environment variables, settings management)
  - `backend/src/core/security.py` (JWT token generation, password hashing with bcrypt)
  - `backend/src/core/exceptions.py` (custom exception classes: AuthenticationError, ValidationError, etc.)
- [ ] T017 [P] Set up structured JSON logging in `backend/src/utils/logging.py`
- [ ] T018 [P] Configure CORS middleware in `backend/main.py` (allow Vercel frontend domain)
- [ ] T019 Create Railway account and project for backend deployment (note: use Railway, not Vercel Serverless per research.md)
- [ ] T020 [P] Create Vercel account and project for frontend deployment

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Interactive Textbook Foundation (Priority: P1) 🎯 MVP

**Goal**: Deploy a fully functional Docusaurus textbook with all Physical AI & Humanoid Robotics course content, accessible on Vercel

**Independent Test**: Visit deployed URL → navigate sidebar → verify all chapters load with proper formatting on desktop/mobile

### Implementation for User Story 1

- [ ] T021 [P] [US1] Convert Physical AI & Humanoid Robotics course content to Markdown files in `frontend/docs/`:
  - `frontend/docs/intro.md` (course introduction)
  - `frontend/docs/quarter-overview.md`
  - `frontend/docs/weeks-01-02/index.md` (and module-details.md)
  - `frontend/docs/weeks-03-04/index.md`
  - `frontend/docs/weeks-05-06/index.md`
  - `frontend/docs/weeks-07-08/index.md`
  - `frontend/docs/weeks-09-10/index.md`
  - `frontend/docs/hardware-requirements.md`
  - `frontend/docs/learning-outcomes.md`
  - (Continue for all course modules - estimate 10-15 total chapters)
- [ ] T022 [US1] Configure sidebar navigation in `frontend/sidebars.js` to match course structure with collapsible sections
- [ ] T023 [P] [US1] Customize Docusaurus theme in `frontend/docusaurus.config.js`:
  - Set site title: "Physical AI & Humanoid Robotics Textbook"
  - Configure theme colors for professional technical aesthetic
  - Set base URL for Vercel deployment
  - Enable dark mode toggle
- [ ] T024 [P] [US1] Implement responsive design CSS in `frontend/src/css/custom.css`:
  - Mobile breakpoint (320px-768px): optimize sidebar, increase font size
  - Tablet breakpoint (768px-1024px): adjust content width
  - Desktop (1024px+): full layout with sidebar
- [ ] T025 [P] [US1] Add accessibility features to Docusaurus pages:
  - ARIA labels for navigation elements
  - Semantic HTML headings (h1, h2, h3)
  - Keyboard navigation support (tab indexing)
  - Alt text for any images in content
- [ ] T026 [US1] Create `vercel.json` for frontend deployment:
  - Build command: `cd frontend && npm run build`
  - Output directory: `frontend/build`
  - Environment variables: `REACT_APP_API_BASE_URL` (Railway backend URL)
- [ ] T027 [US1] Deploy Docusaurus site to Vercel: `cd frontend && vercel --prod`
- [ ] T028 [US1] Test deployment on 3 device sizes:
  - Mobile (320px width via Chrome DevTools)
  - Tablet (768px width)
  - Desktop (1920px width)
  - Verify sidebar navigation, content readability, responsive design
- [ ] T029 [US1] Run Lighthouse audit on deployed site, fix issues to achieve >90 accessibility score

**Checkpoint**: At this point, User Story 1 (P1) should be fully functional and testable independently as a static textbook

---

## Phase 4: User Story 2 - Intelligent RAG Chatbot (Priority: P2) 🎯 Core Feature

**Goal**: Functional RAG chatbot embedded on all pages that answers questions using highlighted text context or full corpus retrieval

**Independent Test**: Highlight text → ask "What does this mean?" → verify response uses only highlighted context. Ask general question → verify retrieval from full textbook.

### Implementation for User Story 2

- [ ] T030 [P] [US2] Create data ingestion script `backend/scripts/ingest_content.py`:
  - Read all Markdown files from `frontend/docs/`
  - Chunk content into 512-token segments with 50-token overlap (use `tiktoken` library)
  - Generate embeddings using OpenAI `text-embedding-3-small` API
  - Upload vectors to Qdrant with metadata (chapter_id, title, section, position)
  - Store ingestion manifest in `backend/data/ingestion_log.json`
- [ ] T031 [US2] Run ingestion script: `python backend/scripts/ingest_content.py` (estimate 500-1000 chunks)
- [ ] T032 [P] [US2] Implement RAG service in `backend/src/services/rag_service.py`:
  - `contextual_query(highlighted_text, query)`: Use highlighted text as exclusive context, pass to GPT-4
  - `general_query(query)`: Embed query → search Qdrant (top-5, similarity >0.7) → pass chunks to GPT-4
  - `get_conversation_history(session_id)`: Retrieve last 5 messages from database
  - Handle multi-turn: Include conversation history in GPT-4 context
- [ ] T033 [P] [US2] Implement embedding service in `backend/src/services/embedding_service.py`:
  - `generate_embedding(text)`: Call OpenAI embedding API, return 1536-dim vector
  - `chunk_text(content, chunk_size=512, overlap=50)`: Split text into chunks
- [ ] T034 [US2] Create chat API endpoints in `backend/src/api/chat.py`:
  - `POST /api/chat/query`: Accept `{query, highlighted_text?, session_id?}` → return `{answer, sources[], latency_ms}`
  - `GET /api/chat/history?session_id={id}`: Return conversation history
  - `POST /api/chat/session`: Create new chat session → return `{session_id}`
  - Add Pydantic request/response validation
  - Implement rate limiting using `slowapi` (100 req/min per IP)
- [ ] T035 [US2] Add chat routes to FastAPI app in `backend/main.py`: `app.include_router(chat.router, prefix="/api/chat")`
- [ ] T036 [P] [US2] Build custom chat widget component `frontend/src/components/ChatWidget/ChatWidget.tsx`:
  - Collapsible panel (bottom-right corner, fixed position)
  - Header with "AI Tutor" title + minimize button
  - Message list area with scroll
  - Input textarea + send button
  - Loading indicator (typing animation)
- [ ] T037 [P] [US2] Implement message list component `frontend/src/components/ChatWidget/MessageList.tsx`:
  - Display user messages (right-aligned, blue bubbles)
  - Display assistant messages (left-aligned, gray bubbles)
  - Render markdown in assistant responses (use `react-markdown`)
  - Show timestamps
- [ ] T038 [P] [US2] Implement highlight detector `frontend/src/components/ChatWidget/HighlightDetector.tsx`:
  - Listen to `mouseup` events on document
  - Capture `window.getSelection().toString()`
  - If text selected, display in chat widget: "Ask about: [first 50 chars]..."
  - Store highlighted text in Zustand state
- [ ] T039 [P] [US2] Create Zustand chat store `frontend/src/store/chatStore.ts`:
  - State: `messages: Message[]`, `sessionId: string | null`, `isLoading: boolean`, `highlightedText: string | null`
  - Actions: `addMessage()`, `setHighlightedText()`, `sendQuery()`, `createSession()`
- [ ] T040 [P] [US2] Implement chat service API client `frontend/src/services/chatService.ts`:
  - `sendMessage(query, highlightedText?, sessionId?)`: POST to `/api/chat/query`
  - `getChatHistory(sessionId)`: GET from `/api/chat/history`
  - `createSession()`: POST to `/api/chat/session`
  - Handle CORS, authentication headers (JWT token from auth store)
  - Error handling: Network errors → display "Unable to connect to chatbot"
- [ ] T041 [US2] Swizzle Docusaurus `Root` component to inject global chat widget:
  - Run: `npm run swizzle @docusaurus/theme-classic Root -- --eject`
  - Modify `frontend/src/theme/Root.tsx` to render `<ChatWidget />` globally
  - Wrap in error boundary
- [ ] T042 [US2] Deploy backend to Railway:
  - Create `Procfile`: `web: uvicorn backend.main:app --host 0.0.0.0 --port $PORT`
  - Set environment variables in Railway dashboard (NEON_DATABASE_URL, QDRANT_URL, OPENAI_API_KEY, JWT_SECRET)
  - Deploy via `railway up` or GitHub integration
- [ ] T043 [US2] Update frontend `vercel.json` to set `REACT_APP_API_BASE_URL` to Railway backend URL
- [ ] T044 [US2] Redeploy frontend to Vercel: `vercel --prod`
- [ ] T045 [US2] Test chatbot end-to-end:
  - Open any chapter → highlight text → ask question → verify response uses highlighted text only
  - Ask general question without highlight → verify retrieval from full corpus
  - Ask follow-up question → verify conversation history maintained
  - Verify p95 latency <2 seconds (check backend logs for `latency_ms`)

**Checkpoint**: At this point, User Story 2 should be fully functional - chatbot works on all pages with contextual and general RAG

---

## Phase 5: User Story 3 - Authentication with Background Questionnaire (Priority: P3) 💎 Bonus

**Goal**: Functional sign-up/sign-in flows with background questionnaire collecting ROS 2 experience, GPU specs, OS, robotics knowledge

**Independent Test**: Sign up → fill questionnaire → sign in → view profile → verify all background data stored correctly

### Implementation for User Story 3

- [ ] T046 [P] [US3] Install Auth.js (NextAuth.js) and Prisma in frontend and backend:
  - Frontend: `npm install next-auth @auth/core`
  - Backend: `pip install prisma-client-py python-jose[cryptography] passlib[bcrypt]`
- [ ] T047 [US3] Create Prisma schema `backend/prisma/schema.prisma` extending User model with background fields:
  - `ros2_experience: String` (None/Beginner/Intermediate/Advanced)
  - `gpu_model: String?` (nullable, free text)
  - `gpu_vram: String?` (nullable, e.g., "12GB")
  - `operating_system: String?` (Ubuntu/Windows/macOS)
  - `robotics_knowledge: String?` (None/Beginner/Intermediate/Advanced)
- [ ] T048 [US3] Generate Prisma client: `cd backend && prisma generate`
- [ ] T049 [P] [US3] Implement auth service `backend/src/services/auth_service.py`:
  - `create_user(email, password, name)`: Hash password with bcrypt (12 rounds), create user in database
  - `authenticate_user(email, password)`: Verify password, return user object
  - `generate_jwt_token(user_id)`: Create JWT with 1-hour expiration, include user_id in payload
  - `verify_jwt_token(token)`: Decode and validate JWT, return user_id
- [ ] T050 [US3] Create auth API endpoints in `backend/src/api/auth.py`:
  - `POST /api/auth/signup`: Accept `{email, password, name, background_data}` → create user → return JWT + user profile
  - `POST /api/auth/signin`: Accept `{email, password}` → authenticate → return JWT + user profile
  - `GET /api/auth/profile`: Require JWT → return user profile including background data
  - `PUT /api/auth/profile`: Require JWT → accept `{background_data}` → update user record
- [ ] T051 [US3] Implement JWT middleware in `backend/src/middleware/auth.py`:
  - Decorator `@require_auth` for protected endpoints
  - Extract JWT from `Authorization: Bearer <token>` header
  - Validate token, attach `user_id` to request context
- [ ] T052 [US3] Add auth routes to FastAPI app in `backend/main.py`: `app.include_router(auth.router, prefix="/api/auth")`
- [ ] T053 [P] [US3] Build sign-up form component `frontend/src/components/Auth/SignUpForm.tsx`:
  - Fields: email (email input), password (password input, min 8 chars), name (text input)
  - Validation: Email format, password strength, required fields
  - Submit → call `authService.signup()` → redirect to background questionnaire
- [ ] T054 [P] [US3] Build background questionnaire component `frontend/src/components/Auth/BackgroundQuestionnaire.tsx`:
  - Dropdown: ROS 2 experience (None/Beginner/Intermediate/Advanced)
  - Text input: GPU model + VRAM (placeholder: "NVIDIA RTX 3060, 12GB")
  - Dropdown: Operating system (Ubuntu/Windows/macOS)
  - Dropdown: Robotics knowledge (None/Beginner/Intermediate/Advanced)
  - Submit → call `authService.updateProfile()` → redirect to homepage
  - Skip button → allow completion later (use default values: Beginner, Unknown GPU, Unknown OS)
- [ ] T055 [P] [US3] Build sign-in form component `frontend/src/components/Auth/SignInForm.tsx`:
  - Fields: email, password
  - Submit → call `authService.signin()` → store JWT in localStorage → redirect to last visited page
  - Error handling: Invalid credentials → display "Incorrect email or password"
- [ ] T056 [P] [US3] Build profile view component `frontend/src/components/Auth/ProfileView.tsx`:
  - Display user info: name, email (read-only)
  - Display background data: ROS 2 experience, GPU, OS, robotics knowledge
  - Edit button → open editable form with same fields as questionnaire
  - Save button → call `authService.updateProfile()` → update display
- [ ] T057 [P] [US3] Create Zustand auth store `frontend/src/store/authStore.ts`:
  - State: `user: User | null`, `token: string | null`, `isAuthenticated: boolean`
  - Actions: `setUser()`, `setToken()`, `logout()`, `loadFromLocalStorage()`
  - Persist token in localStorage, auto-load on app initialization
- [ ] T058 [P] [US3] Implement auth service API client `frontend/src/services/authService.ts`:
  - `signup(email, password, name, backgroundData)`: POST to `/api/auth/signup`
  - `signin(email, password)`: POST to `/api/auth/signin`
  - `getProfile()`: GET from `/api/auth/profile` (include JWT header)
  - `updateProfile(backgroundData)`: PUT to `/api/auth/profile` (include JWT header)
  - Store returned JWT in authStore
- [ ] T059 [US3] Add auth routes to Docusaurus (create custom pages):
  - `frontend/src/pages/signup.tsx`: Render `<SignUpForm />`
  - `frontend/src/pages/signin.tsx`: Render `<SignInForm />`
  - `frontend/src/pages/profile.tsx`: Render `<ProfileView />` (protected: redirect to signin if not authenticated)
  - `frontend/src/pages/questionnaire.tsx`: Render `<BackgroundQuestionnaire />` (accessible after signup)
- [ ] T060 [US3] Add navigation links in Docusaurus navbar (`frontend/docusaurus.config.js`):
  - "Sign Up" → `/signup` (show only if not authenticated)
  - "Sign In" → `/signin` (show only if not authenticated)
  - "Profile" → `/profile` (show only if authenticated)
  - "Logout" button (show only if authenticated, call `authStore.logout()`)
- [ ] T061 [US3] Test authentication flow end-to-end:
  - Sign up with email/password/name → complete questionnaire → verify redirected to homepage
  - Sign out → sign in with same credentials → verify authenticated
  - View profile → verify all background data displayed correctly
  - Edit profile → update GPU model → save → verify update persisted

**Checkpoint**: At this point, User Story 3 should be fully functional - auth works, background data stored

---

## Phase 6: User Story 4 - Content Personalization (Priority: P4) 💎 Bonus

**Goal**: "Personalize This Chapter" button on authenticated pages that re-renders content based on user's ROS 2 experience level

**Independent Test**: Create 2 accounts (Beginner vs Advanced ROS 2 experience) → personalize same chapter → verify content differs by 30%

### Implementation for User Story 4

- [ ] T062 [P] [US4] Implement personalization service `backend/src/services/personalization_service.py`:
  - `personalize_content(chapter_content, user_background)`: Determine personalization level from ROS 2 experience → call GPT-4 with custom prompt
  - GPT-4 prompt for Beginner: "Expand introductions, simplify jargon, add beginner examples. Return as Markdown."
  - GPT-4 prompt for Advanced: "Condense basics, emphasize advanced topics, add complex examples. Return as Markdown."
  - `get_cached_personalization(user_id, chapter_id)`: Check PersonalizationPreference table for cached version
  - `cache_personalization(user_id, chapter_id, personalized_content)`: Store in database with timestamp
- [ ] T063 [US4] Create personalization API endpoint in `backend/src/api/personalization.py`:
  - `POST /api/personalize/chapter`: Require JWT → Accept `{chapter_id}` → retrieve chapter content from database/files → call personalization service → return `{personalized_content}`
  - Check cache first (PersonalizationPreference table) → if exists and <7 days old, return cached version
  - If cache miss: Generate personalization → cache result → return personalized content
  - Error handling: GPT-4 API failures → return original content with warning message
- [ ] T064 [US4] Add personalization routes to FastAPI app in `backend/main.py`: `app.include_router(personalization.router, prefix="/api/personalize")`
- [ ] T065 [P] [US4] Build "Personalize This Chapter" button component `frontend/src/components/Personalization/PersonalizeButton.tsx`:
  - Display only if user is authenticated (check `authStore.isAuthenticated`)
  - Button text: "🎯 Personalize This Chapter"
  - On click: Call `personalizationService.personalizeChapter(chapter_id)` → show loading spinner
  - On success: Replace current chapter content with personalized version
  - On error: Display toast notification "Personalization failed, showing original content"
- [ ] T066 [P] [US4] Implement personalized content renderer `frontend/src/components/Personalization/PersonalizedContent.tsx`:
  - Accept `content` prop (Markdown string)
  - Render using `react-markdown` with syntax highlighting
  - Add badge "✨ Personalized for your level" at top
  - Include "Show Original" toggle button → restore original chapter content
- [ ] T067 [P] [US4] Implement personalization service API client `frontend/src/services/personalizationService.ts`:
  - `personalizeChapter(chapterId)`: POST to `/api/personalize/chapter` with JWT header
  - Return personalized content (Markdown string)
- [ ] T068 [US4] Swizzle Docusaurus `DocPage/Layout` to inject PersonalizeButton:
  - Run: `npm run swizzle @docusaurus/theme-classic DocPage/Layout -- --eject`
  - Modify `frontend/src/theme/DocPage/Layout/index.tsx` to render `<PersonalizeButton />` in chapter header (next to title)
  - Pass current chapter ID as prop (extract from Docusaurus page metadata)
- [ ] T069 [US4] Store chapter content in database for personalization:
  - Create `Chapter` table in Alembic migration (if not exists): `chapter_id`, `title`, `slug`, `markdown_content`
  - Seed database with all Docusaurus Markdown files: Run `python backend/scripts/seed_chapters.py`
  - Alternatively: Read Markdown files directly from `frontend/docs/` in personalization API (simpler, no database)
- [ ] T070 [US4] Test personalization with 2 test accounts:
  - Account A: ROS 2 experience = "Beginner" → personalize Chapter "ROS 2 Fundamentals" → verify expanded explanations
  - Account B: ROS 2 experience = "Advanced" → personalize same chapter → verify condensed basics
  - Compare word counts and technical depth (target: 30% variation)

**Checkpoint**: At this point, User Story 4 should be fully functional - personalization works for authenticated users

---

## Phase 7: User Story 5 - Urdu Translation (Priority: P5) 💎 Bonus

**Goal**: "Translate to Urdu" button on authenticated pages that translates chapter to Urdu with RTL rendering

**Independent Test**: Sign in → navigate to any chapter → click "Translate to Urdu" → verify content displayed in Urdu with RTL layout → toggle back to English

### Implementation for User Story 5

- [ ] T071 [P] [US5] Implement translation service `backend/src/services/translation_service.py`:
  - `translate_to_urdu(chapter_content)`: Call GPT-4 with translation prompt (preserve code blocks, transliterate technical terms)
  - GPT-4 prompt: "Translate to Urdu. Preserve code blocks. Transliterate: GPU → جی پی یو, ROS 2 → آر او ایس 2. Return Markdown."
  - `get_cached_translation(chapter_id, target_language)`: Check TranslationCache table
  - `cache_translation(chapter_id, target_language, translated_content)`: Store in database with 7-day expiry
- [ ] T072 [US5] Create translation API endpoint in `backend/src/api/translation.py`:
  - `POST /api/translate/chapter`: Require JWT → Accept `{chapter_id, target_language}` → retrieve chapter → call translation service → return `{translated_content}`
  - Check cache first → if exists, return cached version
  - If cache miss: Generate translation → cache result → return translated content
  - Error handling: GPT-4 API failures → return original English content with error message "Translation service unavailable"
- [ ] T073 [US5] Add translation routes to FastAPI app in `backend/main.py`: `app.include_router(translation.router, prefix="/api/translate")`
- [ ] T074 [P] [US5] Build "Translate to Urdu" button component `frontend/src/components/Translation/TranslateButton.tsx`:
  - Display only if user is authenticated
  - Button text: "🌐 Translate to Urdu"
  - On click: Call `translationService.translateChapter(chapter_id, 'ur')` → show loading spinner
  - On success: Replace chapter content with Urdu translation + apply RTL CSS
  - Toggle button text to "Show Original English" when in Urdu mode
- [ ] T075 [P] [US5] Implement Urdu content renderer `frontend/src/components/Translation/UrduContent.tsx`:
  - Accept `content` prop (Urdu Markdown string)
  - Render with RTL CSS: `direction: rtl`, `text-align: right`
  - Preserve code blocks in LTR (left-to-right, original English code)
  - Add badge "🌐 Translated to Urdu" at top
  - Include "Show Original English" button
- [ ] T076 [P] [US5] Implement translation service API client `frontend/src/services/translationService.ts`:
  - `translateChapter(chapterId, targetLanguage)`: POST to `/api/translate/chapter` with JWT header
  - Return translated content (Markdown string)
- [ ] T077 [US5] Add TranslateButton to Docusaurus chapters (same swizzled `DocPage/Layout` as personalization):
  - Modify `frontend/src/theme/DocPage/Layout/index.tsx` to render `<TranslateButton />` next to PersonalizeButton
  - Pass current chapter ID as prop
- [ ] T078 [US5] Implement RTL CSS in `frontend/src/css/custom.css`:
  - `.urdu-content { direction: rtl; text-align: right; font-family: 'Noto Nastaliq Urdu', serif; }`
  - `.urdu-content pre, .urdu-content code { direction: ltr; text-align: left; }` (preserve code blocks)
  - Load Urdu web font from Google Fonts in `frontend/docusaurus.config.js`
- [ ] T079 [US5] Test translation:
  - Sign in → navigate to "Hardware Requirements" chapter
  - Click "Translate to Urdu" → verify content translated, RTL rendering applied
  - Verify code blocks remain in English with LTR layout
  - Click "Show Original English" → verify toggles back to English
  - Test translation quality: Check technical term transliteration (GPU, ROS 2, etc.)

**Checkpoint**: At this point, User Story 5 should be fully functional - Urdu translation works with proper RTL rendering

---

## Phase 8: User Story 6 - Claude Code Skills (Priority: P6) 💎 Bonus

**Goal**: Two demonstrable Claude Code-inspired skills: Hardware_Spec_Lookup and ROS2_Command_Generator, accessible via chat widget

**Independent Test**: Invoke skills from chat: `/hardware Jetson Orin` → verify specs returned, `/ros2 launch lidar` → verify ROS 2 command generated

### Implementation for User Story 6

- [ ] T080 [P] [US6] Create hardware specs database `backend/data/hardware_specs.json` with 20-30 common robotics components:
  - Include: Jetson Orin Nano, Jetson AGX Orin, NVIDIA RTX 3060/3080/3090, Intel NUC, Raspberry Pi 4, etc.
  - Fields for each: `name`, `cpu`, `gpu`, `ram`, `power`, `price_range`, `availability`, `use_cases`
- [ ] T081 [P] [US6] Implement Hardware_Spec_Lookup skill service `backend/src/services/skills/hardware_lookup.py`:
  - `lookup_hardware(component_name)`: Load specs from JSON file, fuzzy match component name (e.g., "Jetson Orin" matches "Jetson Orin Nano")
  - Return structured response: `{component, cpu, gpu, ram, power, price_range, availability}`
  - If not found: Return `{error: "Component specifications not found in database"}`
- [ ] T082 [P] [US6] Implement ROS2_Command_Generator skill service `backend/src/services/skills/ros2_generator.py`:
  - `generate_ros2_command(task_description)`: Parse task keywords (launch, run, topic, node, etc.)
  - Template-based generation:
    - "launch {node}" → `ros2 launch {package}_pkg {node}.launch.py`
    - "list topics" → `ros2 topic list`
    - "echo {topic}" → `ros2 topic echo /{topic}`
  - Return: `{task, command, explanation, parameters[]}`
  - If vague: Return `{clarification_needed: "Please specify which ROS 2 distribution (Humble/Iron/Rolling)"}`
- [ ] T083 [US6] Create skills API endpoints in `backend/src/api/skills.py`:
  - `POST /api/skills/hardware`: Accept `{component}` → call hardware_lookup service → return specs
  - `POST /api/skills/ros2`: Accept `{task}` → call ros2_generator service → return command
  - Both endpoints are public (no JWT required for demo purposes)
- [ ] T084 [US6] Add skills routes to FastAPI app in `backend/main.py`: `app.include_router(skills.router, prefix="/api/skills")`
- [ ] T085 [P] [US6] Extend chat widget to detect skill invocation syntax in `frontend/src/components/ChatWidget/ChatWidget.tsx`:
  - Detect `/hardware <component>` in user input → route to hardware skill
  - Detect `/ros2 <task>` in user input → route to ROS2 skill
  - Display skill results as structured cards (not plain text bubbles)
- [ ] T086 [P] [US6] Implement hardware skill result card `frontend/src/components/Skills/HardwareSpecCard.tsx`:
  - Display component name, CPU, GPU, RAM, power, price, availability in formatted card
  - Styling: Icon for hardware (🖥️), color-coded price range (green=budget, yellow=mid, red=high-end)
- [ ] T087 [P] [US6] Implement ROS2 skill result card `frontend/src/components/Skills/ROS2CommandCard.tsx`:
  - Display task description, generated command in code block, explanation, optional parameters
  - Styling: Icon for ROS2 (🤖), copy-to-clipboard button for command
  - If clarification needed: Display question with suggested options
- [ ] T088 [P] [US6] Add skill invocation buttons to chat widget UI:
  - "🔧 Hardware Lookup" button → insert `/hardware ` into input box, focus input
  - "🤖 ROS2 Commands" button → insert `/ros2 ` into input box, focus input
  - Position buttons above input box (toolbar style)
- [ ] T089 [P] [US6] Implement skills service API client `frontend/src/services/skillsService.ts`:
  - `lookupHardware(component)`: POST to `/api/skills/hardware`
  - `generateROS2Command(task)`: POST to `/api/skills/ros2`
- [ ] T090 [US6] Document skills in textbook (create new chapter `frontend/docs/using-ai-skills.md`):
  - Explain purpose of Hardware_Spec_Lookup and ROS2_Command_Generator
  - Provide examples: `/hardware Jetson Orin Nano`, `/ros2 launch a LiDAR sensor node`
  - Add to sidebar navigation under "Getting Started" section
- [ ] T091 [US6] Test skills:
  - Chat widget: Type `/hardware Jetson Orin` → verify specs card displayed with correct data
  - Chat widget: Type `/ros2 launch lidar` → verify ROS2 command card shows `ros2 launch sensor_pkg lidar.launch.py`
  - Test unknown component: `/hardware QuantumComputer9000` → verify error message "Component not found"
  - Test vague ROS2 task: `/ros2 do something` → verify clarification request

**Checkpoint**: At this point, User Story 6 should be fully functional - both skills demonstrable via chat widget

---

## Phase 9: Final Deployment & Polish

**Purpose**: Deploy complete system, end-to-end testing, hackathon submission prep

- [ ] T092 [P] Create comprehensive README.md in repository root:
  - Project overview and hackathon goals (300 points breakdown)
  - Architecture diagram (frontend → backend → databases)
  - Setup instructions (prerequisites, environment variables, local development)
  - Deployment guide (Railway + Vercel)
  - Feature showcase (6 user stories with screenshots)
- [ ] T093 [P] Document API in OpenAPI format:
  - FastAPI auto-generates OpenAPI docs at `/docs`
  - Customize with descriptions, examples, schemas
  - Export to `backend/openapi.yaml` for reference
- [ ] T094 [P] Create environment variable checklist:
  - Frontend `.env`: `REACT_APP_API_BASE_URL`
  - Backend `.env`: `NEON_DATABASE_URL`, `QDRANT_URL`, `QDRANT_API_KEY`, `OPENAI_API_KEY`, `JWT_SECRET`
  - Document setup steps in README.md
- [ ] T095 Verify Railway backend deployment:
  - Check all environment variables set correctly
  - Test API health endpoint: `GET https://<railway-url>/health` → 200 OK
  - Verify database connection, Qdrant connection, OpenAI API key valid
- [ ] T096 Verify Vercel frontend deployment:
  - Check `REACT_APP_API_BASE_URL` points to Railway backend
  - Test homepage loads, sidebar navigation works
  - Verify responsive design on mobile/tablet/desktop
- [ ] T097 Run end-to-end test suite covering all 6 user stories:
  - **P1 (US1)**: Navigate chapters → verify content loads, responsive design
  - **P2 (US2)**: Highlight text → ask question → verify contextual RAG, ask general question → verify corpus retrieval
  - **P3 (US3)**: Sign up → fill questionnaire → sign in → view profile → verify auth flow
  - **P4 (US4)**: Personalize chapter → verify content differs for Beginner vs Advanced
  - **P5 (US5)**: Translate to Urdu → verify RTL rendering, toggle back to English
  - **P6 (US6)**: Invoke hardware skill → verify specs, invoke ROS2 skill → verify command
- [ ] T098 Performance testing:
  - Load test with 100 concurrent users using Artillery or k6
  - Verify chatbot p95 latency <2 seconds
  - Verify page load <3 seconds
  - Check Railway backend metrics (CPU, memory usage)
- [ ] T099 Accessibility audit:
  - Run Lighthouse on 3 pages (homepage, chapter, profile)
  - Fix issues to achieve >90 accessibility score
  - Test keyboard navigation (tab through all interactive elements)
  - Verify screen reader compatibility (test with NVDA or JAWS)
- [ ] T100 Security audit:
  - Test rate limiting: Send 150 requests in 1 minute → verify 429 rate limit error
  - Test SQL injection prevention: Try malicious input in auth fields → verify rejected
  - Test XSS prevention: Try `<script>alert('XSS')</script>` in chat → verify escaped
  - Verify JWT expiration: Wait 1 hour → verify token invalid, redirected to signin
  - Check CORS: Try API request from unauthorized origin → verify blocked
- [ ] T101 Create demo video (5-7 minutes):
  - Introduction: Problem statement (static textbooks lack interactivity)
  - Solution: AI-native textbook with 6 features
  - Demo walkthrough:
    - P1: Navigate textbook on desktop/mobile
    - P2: Use contextual RAG (highlight + ask) and general RAG
    - P3: Sign up with background questionnaire
    - P4: Personalize chapter (show Beginner vs Advanced side-by-side)
    - P5: Translate to Urdu with RTL rendering
    - P6: Invoke hardware and ROS2 skills
  - Technical architecture overview (diagram)
  - Impact: Personalized learning, multilingual access, AI-assisted education
- [ ] T102 Create presentation deck (10-15 slides):
  - Slide 1: Title + Team
  - Slide 2-3: Problem + Opportunity
  - Slide 4-5: Solution Overview (6 features, 300 points)
  - Slide 6-11: Feature demos (1 slide per P1-P6)
  - Slide 12: Technical Architecture
  - Slide 13: Tech Stack (Docusaurus, FastAPI, Railway, Vercel, OpenAI, Neon, Qdrant)
  - Slide 14: Future Vision (scalability, additional languages, instructor tools)
  - Slide 15: Thank You + Q&A
- [ ] T103 Prepare code walkthrough for judges:
  - Highlight clean architecture (frontend/backend separation)
  - Show Spec-Driven Development artifacts (spec.md, plan.md, tasks.md)
  - Demonstrate code quality (TypeScript strict mode, Python type hints, linting)
  - Point out constitution compliance (all gates passed)
- [ ] T104 Final smoke test before submission:
  - Complete a full user journey from sign-up to using all 6 features
  - Verify deployed URLs work (Vercel frontend + Railway backend)
  - Check all environment variables secured (no secrets in code)
  - Verify Git repository clean (no `.env` files committed)
  - Test on fresh browser (incognito mode) to simulate judge experience

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phases 3-8)**: All depend on Foundational phase completion
  - User Story 1 (P1): Independent - can start after Foundational
  - User Story 2 (P2): Depends on P1 (needs textbook content in Qdrant) - can start after P1 completes
  - User Story 3 (P3): Independent after Foundational - can run parallel to P1/P2
  - User Story 4 (P4): Depends on P3 (needs authentication) - can start after P3 completes
  - User Story 5 (P5): Depends on P3 (needs authentication) - can run parallel to P4
  - User Story 6 (P6): Independent after Foundational - can run parallel to P1-P5
- **Final Deployment (Phase 9)**: Depends on all desired user stories being complete

### Critical Path (MVP - P1 + P2)

1. Phase 1 (Setup): T001-T008
2. Phase 2 (Foundational): T009-T020
3. Phase 3 (US1 - Docusaurus): T021-T029
4. Phase 4 (US2 - RAG Chatbot): T030-T045

**Estimated Time for MVP**: 34-46 hours (Phases 1-4)

### Parallel Development Opportunities

After **Foundational Phase (T020)** completes:

**Parallel Track A** (Frontend Developer):
- T021-T029 (US1: Docusaurus setup) → 8-12 hours
- T036-T041 (US2: Chat widget UI) → 6-8 hours
- T053-T060 (US3: Auth UI) → 6-8 hours
- T065-T068 (US4: Personalization UI) → 4-6 hours
- T074-T078 (US5: Translation UI) → 4-6 hours
- T085-T088 (US6: Skills UI) → 4-6 hours

**Parallel Track B** (Backend Developer):
- T030-T035 (US2: RAG backend) → 10-14 hours
- T046-T052 (US3: Auth backend) → 8-10 hours
- T062-T064 (US4: Personalization backend) → 6-8 hours
- T071-T073 (US5: Translation backend) → 4-6 hours
- T080-T084 (US6: Skills backend) → 6-8 hours

**Parallel Track C** (DevOps/Testing):
- T026-T027, T042-T044 (Deployments) → ongoing
- T097-T100 (Testing & audits) → final phase
- T101-T104 (Demo prep) → final phase

**Team Speedup**: With 2 developers (1 frontend, 1 backend), estimated timeline reduces from 68-90 hours to **40-50 hours** (5-7 working days).

---

## Implementation Strategy

### MVP First (User Story 1 + 2 Only)

1. Complete Phase 1: Setup (T001-T008) → ~4 hours
2. Complete Phase 2: Foundational (T009-T020) → ~10 hours
3. Complete Phase 3: US1 - Textbook (T021-T029) → ~8-12 hours
4. Complete Phase 4: US2 - RAG Chatbot (T030-T045) → ~16-20 hours
5. **STOP and VALIDATE**: Test MVP independently
6. Deploy/demo MVP (100 points: textbook + RAG chatbot)

**MVP Completion**: ~38-46 hours

### Incremental Delivery (Add Bonuses Sequentially)

1. MVP (P1+P2) → Deploy → 100 points ✅
2. Add P3 (Auth) → Deploy → 150 points ✅
3. Add P4 (Personalization) → Deploy → 200 points ✅
4. Add P5 (Translation) → Deploy → 250 points ✅
5. Add P6 (Skills) → Deploy → 300 points ✅

Each increment is independently testable and deployable.

### Parallel Team Strategy (2 Developers)

With multiple developers:

1. **Both**: Complete Setup + Foundational together (~14 hours)
2. **Once Foundational done**:
   - **Developer A (Frontend)**: US1 → US2 frontend → US3 frontend → US4-US6 frontends
   - **Developer B (Backend)**: US2 backend → US3 backend → US4-US6 backends
3. Integration points:
   - After US1: Developer B starts US2 RAG backend (needs content from US1)
   - After US2 backend: Developer A integrates chat widget
   - After US3 backend: Developer A integrates auth UI
   - Repeat for US4-US6

**Parallel Timeline**: ~40-50 hours total (5-7 days)

---

## Notes

- **[P] tasks** = different files, no dependencies (can run in parallel)
- **[Story] label** = maps task to specific user story for traceability (e.g., [US1], [US2])
- Each user story should be independently completable and testable
- Tests are OPTIONAL unless explicitly requested (hackathon prioritizes functional delivery)
- Commit after each completed task or logical group
- Stop at any checkpoint to validate story independently before continuing
- Avoid: vague tasks, same-file conflicts, cross-story dependencies that break independence

---

## Total Task Count

- **Setup**: 8 tasks (T001-T008)
- **Foundational**: 12 tasks (T009-T020)
- **US1 (P1 - Textbook)**: 9 tasks (T021-T029)
- **US2 (P2 - RAG Chatbot)**: 16 tasks (T030-T045)
- **US3 (P3 - Authentication)**: 16 tasks (T046-T061)
- **US4 (P4 - Personalization)**: 9 tasks (T062-T070)
- **US5 (P5 - Translation)**: 9 tasks (T071-T079)
- **US6 (P6 - Skills)**: 12 tasks (T080-T091)
- **Final Deployment**: 13 tasks (T092-T104)

**Total: 104 tasks**

**Parallel Opportunities**: 42 tasks marked [P] (40% parallelizable)

**Independent Test Criteria**:
- ✅ US1: Visit deployed URL, navigate chapters, verify responsive design
- ✅ US2: Highlight text + ask question, verify contextual RAG + general RAG
- ✅ US3: Sign up → questionnaire → sign in → profile → verify data stored
- ✅ US4: Personalize chapter with 2 accounts (Beginner vs Advanced) → verify 30% variation
- ✅ US5: Translate to Urdu → verify RTL rendering → toggle English
- ✅ US6: Invoke `/hardware` and `/ros2` skills → verify results

**Suggested MVP Scope**: User Story 1 (P1) + User Story 2 (P2) = 25 tasks, ~38-46 hours, delivers 100 core points

---

## Format Validation

✅ **ALL tasks follow checklist format**:
- Checkbox: `- [ ]`
- Task ID: T001-T104 (sequential)
- [P] marker: Present for 42 parallelizable tasks
- [Story] label: Present for user story tasks (US1-US6)
- Description: Clear action with file paths

✅ **Task organization by user story**: Phases 3-8 map to spec.md priorities P1-P6

✅ **Dependencies documented**: Critical path, parallel tracks, integration points defined

✅ **Independent test criteria**: Each user story has clear pass/fail validation

**Tasks are immediately executable** - each task is specific enough for implementation without additional context.
