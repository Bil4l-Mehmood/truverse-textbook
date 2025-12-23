/**
 * API service for connecting to the RAG backend
 */

// Get API base URL from environment or default intelligently
const getAPIBaseUrl = (): string => {
  // Check for environment variable (only in build-time, replaced by webpack)
  try {
    if (typeof process !== 'undefined' && process.env && process.env.REACT_APP_API_BASE_URL) {
      return process.env.REACT_APP_API_BASE_URL;
    }
  } catch (e) {
    // process might not be defined in browser context
  }

  // Check if we're in browser
  if (typeof window === 'undefined') {
    return 'http://localhost:8000'; // SSR fallback
  }

  // If localhost, use localhost backend
  if (window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1') {
    return 'http://localhost:8000';
  }

  // For production/Vercel, try to detect backend URL
  // Option 1: Check for window.__API_BASE_URL__ if set globally
  if (typeof (window as any).__API_BASE_URL__ !== 'undefined') {
    return (window as any).__API_BASE_URL__;
  }

  // Option 2: Use same domain (assumes API is served from same domain or via proxy)
  return `${window.location.protocol}//${window.location.host}`;
};

const API_BASE_URL = getAPIBaseUrl();

export interface SearchResult {
  id: number;
  score: number;
  chapter_id: string;
  title: string;
  text: string;
  chunk_index: number;
  total_chunks: number;
}

export interface ChatMessage {
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  sources?: SearchResult[];
}

export interface ChatResponse {
  answer: string;
  session_id: string;
  sources: SearchResult[];
  latency_ms: number;
}

export interface SearchResponse {
  query: string;
  results: SearchResult[];
  count: number;
}

/**
 * Send a chat message to the RAG backend
 */
export async function sendChatMessage(
  question: string,
  sessionId?: string
): Promise<ChatResponse> {
  try {
    const response = await fetch(`${API_BASE_URL}/api/v1/chat`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        question,
        session_id: sessionId,
        use_history: true,
        top_k: 3,
      }),
    });

    if (!response.ok) {
      let errorMessage = 'Failed to get response';
      try {
        const error = await response.json();
        errorMessage = error.detail || error.message || 'Failed to get response';
      } catch {
        errorMessage = `Chat request failed (${response.status} ${response.statusText})`;
      }
      throw new Error(errorMessage);
    }

    return response.json();
  } catch (error) {
    const message = error instanceof Error ? error.message : 'Chat request failed - Network error';
    console.error('[API] Chat error:', message);
    throw new Error(message);
  }
}

/**
 * Search the textbook content
 */
export async function searchTextbook(
  query: string,
  limit: number = 5
): Promise<SearchResponse> {
  try {
    const response = await fetch(`${API_BASE_URL}/api/v1/search`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        query,
        limit,
        score_threshold: 0.3,
      }),
    });

    if (!response.ok) {
      let errorMessage = 'Search failed';
      try {
        const error = await response.json();
        errorMessage = error.detail || error.message || 'Search failed';
      } catch {
        errorMessage = `Search failed (${response.status} ${response.statusText})`;
      }
      throw new Error(errorMessage);
    }

    return response.json();
  } catch (error) {
    const message = error instanceof Error ? error.message : 'Search failed - Network error';
    console.error('[API] Search error:', message);
    throw new Error(message);
  }
}

/**
 * Check backend status
 */
export async function checkBackendStatus(): Promise<boolean> {
  try {
    const response = await fetch(`${API_BASE_URL}/api/v1/status`);
    return response.ok;
  } catch {
    return false;
  }
}
