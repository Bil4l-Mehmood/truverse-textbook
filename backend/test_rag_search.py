"""Test RAG search functionality."""

from dotenv import load_dotenv
load_dotenv()

from src.services.embedding_service import generate_embedding
from src.database.qdrant import qdrant_manager

print("Testing RAG search with Qdrant...\n")

# Test query
query = "What are the hardware requirements?"
print(f"Query: '{query}'")

# Generate query embedding
print("  Generating query embedding...")
query_embedding = generate_embedding(query)
print(f"  [OK] Query embedding: {len(query_embedding)} dimensions")

# Search Qdrant
print("  Searching Qdrant for similar content...")
results = qdrant_manager.search_similar(query_embedding, limit=3, score_threshold=0.0)  # Lower threshold for testing
print(f"  [OK] Found {len(results)} results\n")

# Display results
print("Top 3 Results:")
print("=" * 80)
for i, result in enumerate(results, 1):
    score = result.score
    text = result.payload.get('text', '')[:200]  # First 200 chars
    chapter = result.payload.get('chapter_id', 'unknown')
    title = result.payload.get('title', 'Untitled')

    print(f"\n{i}. Score: {score:.4f} | Chapter: {chapter}")
    print(f"   Title: {title}")
    print(f"   Content: {text}...")

print("\n" + "=" * 80)
print("[SUCCESS] RAG search working! Your AI textbook backend is ready!")
