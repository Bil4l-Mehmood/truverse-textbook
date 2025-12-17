# Research Findings: Better Auth Implementation

**Date**: 2025-12-17 | **Status**: ✅ Complete

## Summary

This research document consolidates findings on implementing Better Auth as a complete authentication system replacement. All clarifications from the planning phase have been resolved.

## 1. Better Auth Library Setup and Configuration

### Decision
Use Better Auth 1.4.7 with a standalone Node.js Express server (auth-server) that connects to the same PostgreSQL database as FastAPI.

### Rationale
- Better Auth is a TypeScript-first authentication framework with automatic schema generation
- The standalone server pattern separates auth concerns from the main FastAPI backend
- Provides built-in email/password support with minimal configuration
- Uses cookie-based session management, perfect for SPAs like Docusaurus
- Already configured in your existing `auth-server/auth.js`

### Alternatives Considered
- **FastAPI-only JWT**: Rejected - requires manual password hashing, session management, schema
- **Auth0/Supabase**: Rejected - third-party vendor lock-in and potential costs
- **NextAuth.js**: Rejected - designed for Next.js, Better Auth is framework-agnostic

### Current Status
✅ Your auth-server already has Better Auth configured with PostgreSQL pool and custom user fields

---

## 2. Session Persistence Patterns for Better Auth

### Decision
Use Better Auth's cookie-based session management with JWT encoding strategy (HS256) for FastAPI interoperability.

### Architecture
```
Session Lifecycle:
├── Cookie Cache (5 minute TTL)
├── Database Session (7 days expiration)
└── Auto-refresh when 80% of maxAge reached
```

### Configuration
```javascript
session: {
  expiresIn: 60 * 60 * 24 * 7,  // 7 days
  updateAge: 60 * 60 * 24,      // Refresh every 1 day
  cookieCache: {
    enabled: true,
    maxAge: 60 * 5,              // 5-minute cache
    encoding: "jwt"              // HS256 JWT for FastAPI
  }
}
```

### Rationale
- JWT encoding allows FastAPI to validate tokens without additional decryption
- Cookie caching reduces database queries while maintaining security
- 7-day expiration balances UX (stay logged in) with security
- Automatic refresh prevents token expiration during active usage

### Alternatives Considered
- **Compact encoding**: Rejected - proprietary format, harder for FastAPI to validate
- **JWE encoding**: Rejected - encrypted, impossible for FastAPI to read
- **Database-only sessions**: Rejected - performance impact on every request
- **localStorage only**: Rejected - vulnerable to XSS attacks

---

## 3. React/Docusaurus Frontend Integration

### Decision
Use Better Auth's React client (`createAuthClient`) with Zustand + persist middleware for global state management.

### Architecture
```
Frontend Auth Flow:
├── Better Auth Client (useSession hook)
├── Zustand Store (with persist middleware)
├── localStorage Sync (auth state persistence)
└── Docusaurus Root Component (global provider)
```

### Rationale
- Better Auth provides first-class React support with type-safe hooks
- Zustand is lightweight (3kb) with minimal boilerplate
- Persist middleware automatically syncs to localStorage
- Docusaurus Root component (swizzle) wraps entire React tree
- Dual pattern: Zustand for local state, useSession for server validation

### Implementation Pattern
```typescript
// useAuthStore with Zustand persist
const useAuthStore = create<AuthState>()(
  persist(
    (set) => ({
      user: null,
      token: null,
      isAuthenticated: false,
      setAuth: (user, token) => set({ user, token, isAuthenticated: !!token }),
      logout: () => set({ user: null, token: null, isAuthenticated: false })
    }),
    {
      name: 'auth-store',
      partialize: (state) => ({
        user: state.user,
        token: state.token,
        isAuthenticated: state.isAuthenticated
      })
    }
  )
);
```

### Alternatives Considered
- **React Context only**: Rejected - unnecessary re-renders of all consumers
- **Redux**: Rejected - too heavy for simple auth state
- **Better Auth hooks only**: Rejected - no localStorage persistence by default
- **localStorage only**: Rejected - manual sync required, no reactivity

---

## 4. FastAPI Backend Token Validation

### Decision
Implement FastAPI middleware/dependency that validates Better Auth JWT tokens using the same `BETTER_AUTH_SECRET` key.

### Architecture
```
Request Flow:
├── Extract Bearer token from Authorization header
├── Validate signature using BETTER_AUTH_SECRET (HS256)
├── Decode payload to get user_id
├── Fetch user from shared PostgreSQL database
└── Pass to route handler as dependency
```

### Implementation
```python
# backend/src/middleware/auth.py
from fastapi import Depends, HTTPException
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from jose import jwt, JWTError

security = HTTPBearer()

async def get_current_user(
    credentials: HTTPAuthorizationCredentials = Depends(security)
) -> dict:
    """Validate Better Auth JWT token."""
    try:
        payload = jwt.decode(
            credentials.credentials,
            settings.better_auth_secret,
            algorithms=["HS256"]
        )
        user_id = payload.get("sub")
        if not user_id:
            raise HTTPException(status_code=401)
        return payload
    except JWTError:
        raise HTTPException(status_code=401)
```

### Rationale
- Same secret key in both Better Auth and FastAPI ensures trust
- Dependency injection allows reuse across protected routes
- Per-route application (not middleware) allows public endpoints
- JWT tokens contain user_id for database lookups

### Alternatives Considered
- **Middleware-only**: Rejected - applies to all routes, can't have public endpoints
- **Separate auth databases**: Rejected - data synchronization nightmare
- **Token forwarding**: Rejected - adds latency on every request

---

## 5. Two-Step Signup Flow Implementation

### Decision
Implement two-step flow: Step 1 creates account, Step 2 collects background questionnaire data.

### Flow Diagram
```
Step 1: Email/Password Signup
├── User enters: email, password, name
├── Frontend validates: password 8-72 bytes
├── Calls: POST /api/auth/sign-up/email (auth-server)
├── Returns: Session token + user object
└── Redirect to Step 2

Step 2: Background Questionnaire
├── User answers: ROS2 experience, GPU model, VRAM, OS, robotics knowledge
├── Calls: PATCH /api/v1/auth/profile (backend) with Bearer token
├── Backend updates user record with questionnaire data
└── User fully onboarded, redirect to dashboard
```

### Rationale
- Better Auth's `additionalFields` config stores custom fields in same user table
- Two-step approach provides better UX (quick signup, optional enrichment)
- Questionnaire data enables content personalization
- No email verification required (speeds up onboarding for competition)

### Configuration in auth.js
```javascript
additionalFields: {
  ros2_experience: {
    type: "string",
    input: false
  },
  gpu_model: {
    type: "string",
    input: false
  },
  gpu_vram: {
    type: "string",
    input: false
  },
  operating_system: {
    type: "string",
    input: false
  },
  robotics_knowledge: {
    type: "string",
    input: false
  }
}
```

### Alternatives Considered
- **Single-step with all required**: Rejected - high abandonment rates
- **Email OTP verification**: Rejected - adds complexity, requires email service
- **Three-step with email verification**: Rejected - over-engineered for timeline

---

## 6. React State Management Best Practices

### Decision
Use Zustand with persist middleware + Better Auth's useSession hook for dual-layer approach.

### Architecture
```
State Management Layers:
├── Server State (useSession from Better Auth)
│   ├── Real-time session validation
│   ├── Loading states
│   └── Cache invalidation
└── Local State (Zustand + persist)
    ├── localStorage persistence
    ├── Cross-tab sync
    └── Offline capability
```

### Implementation Patterns

**Selective Subscription** (performance optimization)
```typescript
// Only re-render when isAuthenticated changes
const SignUpButton = () => {
  const isAuthenticated = useAuthStore(state => state.isAuthenticated);
  return isAuthenticated ? <Profile /> : <SignUp />;
};
```

**Hydration Handling** (SSR/SSG support)
```typescript
// Zustand persist middleware handles hydration automatically
const useAuthStore = create<AuthState>()(
  persist(..., {
    name: 'auth-store',
    onRehydrateStorage: () => (state) => {
      // Called after localStorage is loaded
      if (state && state.token) {
        validateTokenWithServer(state.token);
      }
    }
  })
);
```

### Rationale
- Lightweight (3kb) vs Redux (70kb+) boilerplate
- No unnecessary re-renders of other components
- localStorage persistence survives page refreshes
- Server-side session validation catches expired tokens

### Alternatives Considered
- **React Context only**: Rejected - re-renders entire consumer tree
- **Redux**: Rejected - excessive boilerplate for simple auth state
- **localStorage only**: Rejected - no reactivity to state changes
- **localStorage + useState**: Rejected - manual sync required

---

## 7. Password Length Validation and Bcrypt

### Decision
Validate passwords on frontend (8-72 bytes in UTF-8) and backend before bcrypt hashing.

### Why 72 Bytes?
Bcrypt has a hard limit of 72 bytes due to its internal design (Blowfish cipher limitation). If passwords exceed 72 bytes, they are silently truncated, creating a security vulnerability where users think their password is longer than it actually is.

### UTF-8 Byte Calculation
```
1-byte chars (ASCII):  A-Z, a-z, 0-9 → 1 byte each
2-byte chars (Latin):  à, é, ñ → 2 bytes each
3-byte chars (CJK):    中, 日, 韓 → 3 bytes each
4-byte chars (Emoji):  😀, 🎉 → 4 bytes each
```

### Frontend Validation
```typescript
function validatePasswordBytes(password: string): {
  valid: boolean;
  error?: string;
} {
  const byteLength = new TextEncoder().encode(password).length;

  if (byteLength < 8) {
    return { valid: false, error: "Password must be at least 8 characters" };
  }

  if (byteLength > 72) {
    return {
      valid: false,
      error: "Password too long (max 72 bytes). Reduce special characters."
    };
  }

  return { valid: true };
}
```

### Backend Validation
```python
def validate_password_length(password: str) -> None:
    """Validate password byte length before bcrypt hashing."""
    password_bytes = password.encode('utf-8')

    if len(password_bytes) < 8:
        raise ValueError("Password must be at least 8 characters")

    if len(password_bytes) > 72:
        raise ValueError(
            "Password too long (max 72 bytes in UTF-8). "
            "Use fewer special characters or shorter password."
        )
```

### Rationale
- OWASP recommends minimum 8 characters (no maximum for user convenience)
- Byte-length validation prevents silent truncation
- Users can use emoji and special characters safely
- Clear error messages guide users to compliant passwords

### Alternatives Considered
- **No maximum**: Rejected - bcrypt silently truncates, security vulnerability
- **Character-based limit (72 chars)**: Rejected - doesn't account for multi-byte UTF-8
- **Pre-hash with SHA-256**: Rejected - doesn't improve security, adds complexity
- **Switch to Argon2**: Considered but not needed for timeline (bcrypt acceptable)

### Current Status
✅ Your `betterAuthService.ts` already implements frontend validation
⚠️ Backend needs byte-length validation before bcrypt in FastAPI

---

## 8. Content Protection & Routing

### Decision
Implement ProtectedRoute component that checks authentication status before rendering protected content.

### Architecture
```
Route Protection Flow:
├── User clicks "Course Content"
├── ProtectedRoute component checks isAuthenticated
├── If false: Redirect to /signin
├── If true: Render course content page
└── Course content includes RAG chat widget
```

### Implementation Pattern
```typescript
// frontend/src/components/Auth/ProtectedRoute.tsx
import { useAuthStore } from '@site/src/store/authStore';
import { Navigate } from 'react-router-dom';

export function ProtectedRoute({ children }: { children: React.ReactNode }) {
  const isAuthenticated = useAuthStore(state => state.isAuthenticated);

  if (!isAuthenticated) {
    return <Navigate to="/signin" replace />;
  }

  return <>{children}</>;
}
```

### Backend Protection
```python
# backend/src/api/routes/search.py
from fastapi import Depends
from src.middleware.auth import get_current_user

@router.post("/api/v1/chat")
async def chat(
    request: ChatRequest,
    current_user: dict = Depends(get_current_user)  # Requires auth
):
    """Protected endpoint - only authenticated users can chat."""
    return process_chat(request, user_id=current_user["sub"])
```

### Rationale
- Simple, declarative component wrapping
- Works with Docusaurus page routing
- Backend enforces auth regardless of frontend (defense in depth)
- Graceful redirect to sign-in instead of error pages

---

## 9. Current Implementation Status

### ✅ Already Configured
- `auth-server/auth.js` - Better Auth server with PostgreSQL pool
- `frontend/src/lib/auth-client.ts` - Better Auth client initialization
- `frontend/src/store/authStore.ts` - Zustand store with persist
- `backend/.env` - Database credentials and API keys configured

### ⚠️ Needs Fixes
- `frontend/src/services/betterAuthService.ts` - Error handling (FIXED in previous task)
- `frontend/src/services/api.ts` - Error handling (FIXED in previous task)
- `frontend/src/theme/Navbar/index.tsx` - Sign In/Sign Up buttons removed (need to restore)

### ❌ Needs Implementation
- Two-step signup pages (Step 1 identity + Step 2 questionnaire)
- QuestionnaireForm component
- Profile update endpoint in backend
- ProtectedRoute component
- Content guarding on homepage

---

## Technology Stack Summary

| Component | Technology | Version | Status |
|-----------|-----------|---------|--------|
| Frontend | React + Docusaurus | 3.x | ✅ Running |
| Auth Frontend Client | better-auth | 1.4.7 | ✅ Configured |
| State Management | Zustand | 4.x | ✅ Configured |
| Auth Server | Node.js + Better Auth | 18+ | ✅ Configured |
| Backend | FastAPI | 0.100+ | ✅ Running |
| Token Validation | python-jose | 3.x | ⚠️ Needs config |
| Database | Neon PostgreSQL | Latest | ✅ Connected |
| Vector DB | Qdrant | Cloud | ✅ Connected |

---

## Key Takeaways

1. **Better Auth is production-ready** - No need for custom auth implementation
2. **Session persistence is automatic** - JWT + cookie caching handles 7-day sessions
3. **Frontend/backend integration is simple** - JWT validation with same secret key
4. **Two-step signup improves UX** - Quick account creation + optional enrichment
5. **Password validation must handle UTF-8** - Not just character count
6. **State management with Zustand is optimal** - Light, powerful, flexible
7. **Content protection is straightforward** - ProtectedRoute component pattern

---

## Next Steps

1. **Cleanup phase**: Remove broken custom auth code
2. **Error handling**: ✅ Already fixed in betterAuthService.ts and api.ts
3. **Restore navbar**: Show Sign In/Sign Up buttons
4. **Implement signup flow**: Two-step process with questionnaire
5. **Add content guarding**: ProtectedRoute + backend middleware
6. **Backend profile endpoint**: PATCH /api/v1/auth/profile
7. **Testing & deployment**: E2E testing on Vercel

---

**Research Status**: ✅ Complete - All clarifications resolved
**Ready for**: Task generation via `/sp.tasks`
