<!--
Sync Impact Report:
Version: 1.0.0 → 1.0.0 (Initial constitution for new project)
Modified Principles: N/A (new creation)
Added Sections: All sections created from template
Removed Sections: None
Templates Status:
  ✅ plan-template.md - Constitution Check section ready for validation gates
  ✅ spec-template.md - Requirements alignment configured
  ✅ tasks-template.md - Task categorization aligned with principles
Follow-up TODOs: None - all placeholders resolved
-->

# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. AI-Native Development (NON-NEGOTIABLE)

All development MUST leverage AI-first tools and workflows:
- Claude Code is the primary development agent for all implementation work
- Spec-Kit Plus workflow (specify → plan → tasks → implement) MUST be followed for all features
- Every feature begins with `/sp.specify`, proceeds through `/sp.plan` and `/sp.tasks` before implementation
- Human-as-tool strategy: Invoke user for clarification when requirements are ambiguous, dependencies are unforeseen, or architectural decisions have significant tradeoffs
- Prompt History Records (PHRs) MUST be created for every significant interaction under `history/prompts/`

**Rationale**: Ensures systematic, traceable, and high-quality development aligned with AI-assisted workflows and Spec-Driven Development principles.

### II. Technology Stack Adherence (NON-NEGOTIABLE)

The following technology stack is MANDATORY and MUST NOT be substituted:

**Frontend/Book Platform**:
- Docusaurus (React-based static site generator)
- TypeScript/JavaScript (idiomatic, type-safe code required)
- OpenAI ChatKit SDK for frontend chatbot integration

**Backend/RAG System**:
- FastAPI (Python web framework)
- Python 3.10+ (idiomatic, type-hinted code required)
- OpenAI Agents SDK for backend RAG orchestration
- Neon Serverless Postgres (database - free tier)
- Qdrant Cloud (vector store - free tier)

**Authentication**:
- Better-Auth (signup/signin implementation)

**Deployment**:
- Vercel (primary and only deployment target)
- GitHub Pages is EXPLICITLY FORBIDDEN

**Rationale**: Stack choices are constrained by hackathon requirements and free-tier availability. Deviations risk disqualification or infrastructure costs.

### III. Feature Completeness & Bonus Points (NON-NEGOTIABLE)

ALL features required for maximum hackathon points MUST be implemented:

**Core Features (Required)**:
- Complete AI-native textbook content for Physical AI & Humanoid Robotics
- Functional RAG chatbot integrated into Docusaurus site
- User authentication (signup/signin) via Better-Auth
- Personalization features (user-specific chat history, bookmarks, progress tracking)

**50-Point Bonus Features (ALL REQUIRED)**:
- Advanced RAG capabilities (multi-turn conversations, context retention)
- Responsive design (mobile, tablet, desktop)
- Search functionality across textbook content
- Interactive code examples and demonstrations
- Analytics integration for user engagement tracking

No feature may be deferred, simplified, or marked as "future work" without explicit user approval.

**Rationale**: Hackathon success requires maximum point accumulation; partial implementation reduces competitive standing.

### IV. Code Quality Standards (NON-NEGOTIABLE)

All code MUST meet professional production standards:

**Frontend (TypeScript/JavaScript)**:
- Strict TypeScript mode enabled (`strict: true` in tsconfig.json)
- ESLint and Prettier configured and enforced
- Component-based architecture following React best practices
- Proper error boundaries and loading states
- Accessible UI (WCAG 2.1 AA minimum)

**Backend (Python)**:
- Type hints required for all functions (mypy validation passing)
- PEP 8 compliance (Black formatter, Flake8 linter)
- Async/await patterns for I/O operations
- Proper exception handling with custom exception classes
- Structured logging (JSON format for production)

**General**:
- No hardcoded secrets (environment variables via `.env`)
- Clear separation of concerns (no God objects or 1000-line files)
- Self-documenting code (meaningful names over excessive comments)
- DRY principle: no copy-paste code duplication

**Rationale**: High code quality is a stated hackathon requirement and reflects professional engineering standards.

### V. Integration Architecture

The Docusaurus frontend and FastAPI backend MUST integrate seamlessly:

**API Contract**:
- RESTful endpoints for RAG chat interactions (`/api/chat`, `/api/history`, etc.)
- OpenAPI/Swagger documentation auto-generated from FastAPI
- CORS properly configured for Vercel deployment
- Authentication tokens (JWT) passed via headers

**State Management**:
- Frontend uses React Context or Zustand for global state
- Backend maintains session state in Postgres
- Vector embeddings stored in Qdrant with metadata linking to Postgres records

**Error Handling**:
- Unified error response format: `{ "error": string, "code": string, "details": object }`
- Frontend displays user-friendly error messages
- Backend logs detailed stack traces for debugging

**Rationale**: Clear contracts prevent integration bugs and enable parallel frontend/backend development.

### VI. Testing & Validation

Quality assurance through systematic testing:

**Frontend Testing**:
- Unit tests for utility functions and hooks (Jest/Vitest)
- Component tests for interactive elements (React Testing Library)
- E2E tests for critical user journeys (Playwright)

**Backend Testing**:
- Unit tests for business logic (pytest)
- Integration tests for API endpoints (pytest + TestClient)
- Contract tests verifying OpenAPI spec compliance

**RAG Testing**:
- Embedding quality validation (cosine similarity thresholds)
- Retrieval accuracy tests (precision/recall on known queries)
- Response quality evaluation (manual review + automated checks)

**Deployment Testing**:
- Smoke tests in staging environment before production
- Performance benchmarks (p95 latency < 2s for RAG queries)

**Rationale**: Systematic testing prevents regressions and ensures hackathon demo reliability.

### VII. Deployment & DevOps

Vercel-optimized deployment strategy:

**Build Configuration**:
- Docusaurus optimized for static export (`docusaurus build`)
- FastAPI deployed as Vercel serverless functions
- Environment variables configured in Vercel dashboard
- Build time < 5 minutes (enforced via CI)

**Monitoring**:
- Vercel Analytics enabled for frontend metrics
- Backend logging via structured JSON to stdout (captured by Vercel)
- Error tracking via Sentry or similar (optional but recommended)

**Rollback Strategy**:
- Git-based deployments (every commit tagged)
- Vercel instant rollback to previous deployment
- Database migrations reversible (separate schema versions)

**Rationale**: Vercel is the mandated platform; optimizing for its constraints ensures reliable deployments.

### VIII. Security & Privacy

Protect user data and prevent vulnerabilities:

**Authentication Security**:
- Passwords hashed with bcrypt (12+ rounds)
- JWT tokens with 1-hour expiration, refresh tokens rotated
- HTTPS enforced (Vercel default)
- CSRF protection for state-changing operations

**Data Privacy**:
- User chat history isolated per user (row-level security)
- No PII in logs or error messages
- GDPR-compliant data export/deletion endpoints

**API Security**:
- Rate limiting (100 req/min per user)
- Input validation on all endpoints (Pydantic models)
- SQL injection prevention (parameterized queries via ORM)
- XSS prevention (React automatic escaping + CSP headers)

**Rationale**: Security vulnerabilities disqualify projects; privacy compliance is legally required.

### IX. Simplicity & Pragmatism

Avoid over-engineering; deliver working features:

**YAGNI Principle**:
- Build only what's explicitly required for hackathon criteria
- No speculative features ("might need later")
- No premature abstractions (3 duplications before extracting)

**Smallest Viable Change**:
- Refactor only code being modified for current feature
- No "while we're here" improvements unrelated to task
- Keep PRs focused on single user story

**Technology Minimalism**:
- Use native platform features before adding libraries
- Prefer Docusaurus built-ins over custom React components
- Avoid microservices (monolithic FastAPI backend sufficient)

**Rationale**: Hackathon time constraints demand ruthless prioritization; complexity is the enemy of shipping.

## Development Workflow

### Spec-Driven Development (SDD) Process

Every feature MUST follow this workflow:

1. **Specify** (`/sp.specify`): Capture requirements, user stories, acceptance criteria
2. **Plan** (`/sp.plan`): Research, design architecture, define contracts
3. **Tasks** (`/sp.tasks`): Break down into implementable, testable tasks
4. **Implement** (`/sp.implement`): Execute tasks with TDD discipline
5. **Review**: Create PHR, suggest ADR if architecturally significant

### Prompt History Records (PHR)

**Mandatory Creation**:
- After every feature specification, plan, or implementation session
- Routed to `history/prompts/constitution/` (constitution changes)
- Routed to `history/prompts/<feature-name>/` (feature work)
- Routed to `history/prompts/general/` (general queries)

**Required Fields**:
- Full PROMPT_TEXT (verbatim user input, not truncated)
- Concise RESPONSE_TEXT (summary of assistant output)
- Stage label (spec, plan, tasks, red, green, refactor, misc, general)
- Files modified/created, tests run/added

### Architecture Decision Records (ADR)

**When to Create**:
- Detect significance: Long-term impact + Multiple alternatives + Cross-cutting scope
- Suggest to user: "📋 Architectural decision detected: [brief]. Document? Run `/sp.adr [title]`"
- Wait for user consent; NEVER auto-create

**Example Triggers**:
- Choosing Qdrant over Pinecone for vector store
- Deciding on JWT vs. session-based auth
- Selecting Docusaurus plugin architecture for chatbot integration

## Quality Gates

### Constitution Check (from plan-template.md)

Every implementation plan MUST verify compliance:

- ✅ Technology stack matches Section II requirements
- ✅ All bonus features from Section III included
- ✅ Code quality standards (Section IV) tooling configured
- ✅ Integration contracts (Section V) documented
- ✅ Testing strategy (Section VI) defined
- ✅ Deployment plan (Section VII) Vercel-compatible
- ✅ Security measures (Section VIII) implemented
- ✅ No over-engineering (Section IX) violations

### Definition of Done

A feature is complete when:

- [ ] All acceptance criteria from spec.md passing
- [ ] Code quality checks passing (linters, formatters, type checkers)
- [ ] Tests written and passing (unit, integration, E2E as applicable)
- [ ] API contracts documented (OpenAPI spec updated)
- [ ] Deployed to staging and smoke-tested
- [ ] PHR created documenting the work
- [ ] ADR created if architecturally significant decision made

## Governance

**Authority**: This constitution supersedes all other project documentation and coding practices.

**Amendments**:
- Require explicit user approval via `/sp.constitution` command
- Must include version bump rationale (MAJOR/MINOR/PATCH)
- Must propagate changes to affected templates (plan, spec, tasks)
- Must create ADR documenting rationale for significant governance changes

**Compliance**:
- All pull requests/commits MUST reference compliance with relevant principles
- Violations require justification in plan.md "Complexity Tracking" section
- Repeated violations without justification block merges

**Enforcement**:
- Claude Code agent validates compliance during `/sp.plan` execution
- Human reviewers verify adherence during PR reviews
- CI/CD pipelines enforce automated quality gates (linters, tests, build)

**Conflict Resolution**:
- User intent always overrides constitution where explicit
- Constitution provides defaults when user intent ambiguous
- Ambiguities resolved via Human-as-Tool clarifying questions

**Version**: 1.0.0 | **Ratified**: 2025-12-14 | **Last Amended**: 2025-12-14
