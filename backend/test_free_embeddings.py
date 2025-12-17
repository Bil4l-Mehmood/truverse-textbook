"""Test free embedding service."""

import sys
from pathlib import Path
from dotenv import load_dotenv

sys.path.insert(0, str(Path(__file__).parent))

# Load environment variables
load_dotenv()

from src.services.embedding_service import generate_embedding, generate_embeddings_batch

print("Testing FREE Cohere embedding service...\n")

# Test single embedding
print("1. Testing single embedding...")
text = "This is a test of the free embedding service."
embedding = generate_embedding(text)
print(f"   [OK] Generated embedding with {len(embedding)} dimensions")

# Test batch embeddings
print("\n2. Testing batch embeddings...")
texts = [
    "Physical AI and robotics",
    "ROS 2 fundamentals",
    "Computer vision with YOLO",
]
embeddings = generate_embeddings_batch(texts)
print(f"   [OK] Generated {len(embeddings)} embeddings in batch")

print("\n[SUCCESS] Free embeddings working! Ready for data ingestion!")
