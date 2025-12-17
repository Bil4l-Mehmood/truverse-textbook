# Quick Setup: Free Embeddings with Cohere

## Why Cohere?

After trying multiple free embedding solutions:
- ❌ **OpenAI**: Quota exceeded, needs billing
- ❌ **Sentence Transformers**: Requires VC++ Redistributable on Windows
- ❌ **Hugging Face**: API endpoint deprecated (410 Gone)
- ✅ **Cohere**: Works immediately, free tier, no system dependencies

## Get Your Free API Key (1 minute)

1. Go to: https://dashboard.cohere.com/api-keys
2. Sign up with email (no credit card required)
3. Copy your API key (starts with `co_...`)
4. Add to `backend/.env`:
   ```
   COHERE_API_KEY=co_your_key_here
   ```

## Free Tier Limits

- **100 requests/minute** (very generous)
- **96 texts per batch** (efficient for data ingestion)
- **384 dimensions** (perfect for RAG)
- **Model**: embed-english-light-v3.0 (fast & accurate)

## Test the Setup

```bash
cd backend
python test_free_embeddings.py
```

Expected output:
```
Testing FREE Cohere embedding service...

1. Testing single embedding...
   [OK] Generated embedding with 384 dimensions

2. Testing batch embeddings...
   [OK] Generated 3 embeddings in batch

[SUCCESS] Free embeddings working! Ready for data ingestion!
```

## Next Step: Data Ingestion

Once embeddings work, run:
```bash
python backend/scripts/ingest_content.py
```

This will:
1. Read all Markdown files from `frontend/docs/`
2. Chunk content (512 tokens, 50 overlap)
3. Generate embeddings via Cohere (FREE)
4. Upload to Qdrant

## Why This Works

- **No billing**: Cohere's free tier is generous
- **No dependencies**: Just HTTP requests, no PyTorch/VC++
- **Production-ready**: Cohere is used by major companies
- **Great quality**: Specifically designed for semantic search

## Cost Comparison

| Provider | Cost | Rate Limit | Setup |
|----------|------|------------|-------|
| OpenAI | $0.00002/1K tokens | 3500/min | Needs billing |
| Cohere | $0 (FREE) | 100/min | Just API key |
| Sentence Transformers | $0 (local) | None | Needs VC++ |

---

**You're one API key away from unlimited free embeddings!** 🚀
