"""
Data ingestion script for Docusaurus content.

This script:
1. Reads all Markdown files from frontend/docs/
2. Chunks content into 512-token segments with 50-token overlap
3. Generates embeddings using OpenAI text-embedding-3-small
4. Uploads vectors to Qdrant with metadata

Usage:
    python backend/scripts/ingest_content.py
"""

import os
import sys
import asyncio
import json
from pathlib import Path
from typing import List, Dict, Any
import logging

# Add backend to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from dotenv import load_dotenv
import tiktoken

# Load environment variables FIRST (before importing settings)
load_dotenv()

from src.core.config import settings
from src.database.qdrant import qdrant_manager
from src.services.embedding_service import generate_embeddings_batch

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize tokenizer
encoding = tiktoken.encoding_for_model("gpt-4")


def chunk_text(text: str, chunk_size: int = 512, overlap: int = 50) -> List[str]:
    """
    Chunk text into segments with overlap using token-based chunking.

    Args:
        text: Input text to chunk
        chunk_size: Maximum tokens per chunk
        overlap: Number of overlapping tokens between chunks

    Returns:
        List of text chunks
    """
    tokens = encoding.encode(text)
    chunks = []

    start = 0
    while start < len(tokens):
        end = start + chunk_size
        chunk_tokens = tokens[start:end]
        chunk_text = encoding.decode(chunk_tokens)
        chunks.append(chunk_text)

        # Move start position with overlap
        start = end - overlap if end < len(tokens) else end

    return chunks


def read_markdown_files(docs_dir: str) -> List[Dict[str, Any]]:
    """
    Read all Markdown files from the docs directory.

    Args:
        docs_dir: Path to the docs directory

    Returns:
        List of documents with content and metadata
    """
    documents = []
    docs_path = Path(docs_dir)

    if not docs_path.exists():
        logger.error(f"Docs directory not found: {docs_dir}")
        return documents

    # Find all .md files
    md_files = list(docs_path.rglob("*.md"))
    logger.info(f"Found {len(md_files)} Markdown files")

    for md_file in md_files:
        try:
            # Try UTF-8 first, fallback to latin-1 if needed
            try:
                with open(md_file, "r", encoding="utf-8") as f:
                    content = f.read()
            except UnicodeDecodeError:
                with open(md_file, "r", encoding="latin-1") as f:
                    content = f.read()

            # Extract relative path for metadata
            relative_path = md_file.relative_to(docs_path)
            chapter_id = str(relative_path).replace("\\", "/").replace(".md", "")

            # Extract title (first # heading)
            title = "Untitled"
            lines = content.split("\n")
            for line in lines:
                if line.startswith("# "):
                    title = line.replace("# ", "").strip()
                    break

            documents.append({
                "path": str(md_file),
                "relative_path": str(relative_path),
                "chapter_id": chapter_id,
                "title": title,
                "content": content,
            })

        except Exception as e:
            logger.error(f"Error reading {md_file}: {e}")

    return documents


# Removed generate_embedding - using batch embeddings from embedding_service


async def ingest_documents(documents: List[Dict[str, Any]]):
    """
    Process documents and upload to Qdrant.

    Args:
        documents: List of documents to ingest
    """
    all_chunks = []
    all_metadata = []

    logger.info(f"Processing {len(documents)} documents...")

    for doc in documents:
        # Chunk the content
        chunks = chunk_text(
            doc["content"],
            chunk_size=settings.chunk_size,
            overlap=settings.chunk_overlap,
        )

        logger.info(f"  {doc['chapter_id']}: {len(chunks)} chunks")

        for i, chunk in enumerate(chunks):
            all_chunks.append(chunk)
            all_metadata.append({
                "chapter_id": doc["chapter_id"],
                "title": doc["title"],
                "chunk_index": i,
                "total_chunks": len(chunks),
                "text": chunk,  # Store original text for retrieval
            })

    logger.info(f"Total chunks: {len(all_chunks)}")

    # Generate embeddings using Sentence Transformers (FREE, fast batch processing)
    logger.info("Generating embeddings with Sentence Transformers (local, no API calls)...")

    # Process all at once - no API limits!
    all_embeddings = generate_embeddings_batch(all_chunks)

    logger.info(f"Generated {len(all_embeddings)} embeddings (384 dimensions each)")

    # Upload to Qdrant
    logger.info("Uploading vectors to Qdrant...")

    # Create collection if it doesn't exist
    await qdrant_manager.create_collection()

    # Upload in batches
    batch_size = 100
    for i in range(0, len(all_embeddings), batch_size):
        batch_embeddings = all_embeddings[i:i + batch_size]
        batch_metadata = all_metadata[i:i + batch_size]
        batch_ids = list(range(i, i + len(batch_embeddings)))

        qdrant_manager.upsert_vectors(
            vectors=batch_embeddings,
            payloads=batch_metadata,
            ids=batch_ids,
        )

        logger.info(f"  Uploaded batch {i//batch_size + 1}/{(len(all_embeddings) + batch_size - 1)//batch_size}")

    logger.info("Ingestion complete!")

    # Save ingestion manifest
    manifest = {
        "total_documents": len(documents),
        "total_chunks": len(all_chunks),
        "embedding_model": settings.embedding_model,
        "chunk_size": settings.chunk_size,
        "chunk_overlap": settings.chunk_overlap,
        "documents": [
            {
                "chapter_id": doc["chapter_id"],
                "title": doc["title"],
                "chunks": len([c for c in all_metadata if c["chapter_id"] == doc["chapter_id"]]),
            }
            for doc in documents
        ],
    }

    manifest_path = Path(__file__).parent.parent / "data" / "ingestion_manifest.json"
    manifest_path.parent.mkdir(parents=True, exist_ok=True)

    with open(manifest_path, "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2)

    logger.info(f"Manifest saved to: {manifest_path}")


async def main():
    """Main ingestion workflow."""
    # Path to frontend docs directory
    docs_dir = Path(__file__).parent.parent.parent / "frontend" / "docs"

    logger.info(f"Reading Markdown files from: {docs_dir}")

    # Read all documents
    documents = read_markdown_files(str(docs_dir))

    if not documents:
        logger.error("No documents found. Exiting.")
        return

    # Ingest documents
    await ingest_documents(documents)

    # Verify collection
    collection_info = qdrant_manager.get_collection_info()
    logger.info(f"Collection status: {collection_info}")


if __name__ == "__main__":
    asyncio.run(main())
