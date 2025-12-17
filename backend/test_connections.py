"""Quick test script to verify API connections."""

import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from openai import OpenAI

load_dotenv()

print("Testing API connections...\n")

# Test Qdrant
print("1. Testing Qdrant Cloud...")
try:
    qdrant = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY"),
    )
    collections = qdrant.get_collections()
    print(f"   [OK] Qdrant connected! Collections: {[c.name for c in collections.collections]}")
except Exception as e:
    print(f"   [ERROR] Qdrant error: {e}")

# Test OpenAI
print("\n2. Testing OpenAI API...")
try:
    client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
    # Test with a simple embedding
    response = client.embeddings.create(
        model="text-embedding-3-small",
        input="test",
    )
    print(f"   [OK] OpenAI connected! Embedding dimensions: {len(response.data[0].embedding)}")
except Exception as e:
    print(f"   [ERROR] OpenAI error: {e}")

print("\nConnection test complete!")
