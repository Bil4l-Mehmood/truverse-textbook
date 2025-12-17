"""Test RAG API endpoints."""

import requests
import json
import time

BASE_URL = "http://localhost:8000"

print("Testing RAG API Endpoints...\n")
print("=" * 80)

# Wait for server to be ready
print("\n1. Checking if server is ready...")
max_retries = 10
for i in range(max_retries):
    try:
        response = requests.get(f"{BASE_URL}/health", timeout=2)
        if response.status_code == 200:
            print("   [OK] Server is ready!")
            break
    except requests.exceptions.RequestException:
        if i < max_retries - 1:
            print(f"   Waiting for server... ({i+1}/{max_retries})")
            time.sleep(2)
        else:
            print("   [ERROR] Server not responding")
            exit(1)

# Test 1: Status endpoint
print("\n2. Testing /api/v1/status endpoint...")
try:
    response = requests.get(f"{BASE_URL}/api/v1/status")
    status = response.json()
    print(f"   [OK] Status: {json.dumps(status, indent=2)}")
except Exception as e:
    print(f"   [ERROR] {e}")

# Test 2: Search endpoint
print("\n3. Testing /api/v1/search endpoint...")
search_data = {
    "query": "What are the hardware requirements for ROS 2?",
    "limit": 3,
    "score_threshold": 0.25
}

try:
    response = requests.post(
        f"{BASE_URL}/api/v1/search",
        json=search_data,
        headers={"Content-Type": "application/json"}
    )

    if response.status_code == 200:
        result = response.json()
        print(f"   [OK] Found {result['count']} results")
        print(f"\n   Top result:")
        if result['results']:
            top = result['results'][0]
            print(f"   - Score: {top['score']:.4f}")
            print(f"   - Chapter: {top['chapter_id']}")
            print(f"   - Title: {top['title']}")
            print(f"   - Text: {top['text'][:150]}...")
    else:
        print(f"   [ERROR] Status {response.status_code}: {response.text}")
except Exception as e:
    print(f"   [ERROR] {e}")

# Test 3: Chat endpoint (without Groq - will return context)
print("\n4. Testing /api/v1/chat endpoint...")
chat_data = {
    "question": "What GPU do I need for this course?",
    "top_k": 3
}

try:
    response = requests.post(
        f"{BASE_URL}/api/v1/chat",
        json=chat_data,
        headers={"Content-Type": "application/json"}
    )

    if response.status_code == 200:
        result = response.json()
        print(f"   [OK] Chat response generated")
        print(f"   - Session ID: {result['session_id']}")
        print(f"   - Latency: {result['latency_ms']:.2f}ms")
        print(f"   - Sources: {len(result['sources'])} documents")
        print(f"\n   Answer preview:")
        print(f"   {result['answer'][:200]}...")
    else:
        print(f"   [ERROR] Status {response.status_code}: {response.text}")
except Exception as e:
    print(f"   [ERROR] {e}")

print("\n" + "=" * 80)
print("\n[SUCCESS] API endpoints tested!")
print("\nAPI Documentation: http://localhost:8000/docs")
