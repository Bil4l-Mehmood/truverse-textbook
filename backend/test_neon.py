"""Test Neon Postgres connection."""

import os
import asyncio
from dotenv import load_dotenv
from sqlalchemy.ext.asyncio import create_async_engine
from sqlalchemy import text

load_dotenv()

async def test_neon():
    print("Testing Neon Postgres connection...")

    try:
        engine = create_async_engine(
            os.getenv("NEON_DATABASE_URL"),
            echo=False,
        )

        async with engine.connect() as conn:
            result = await conn.execute(text("SELECT version()"))
            version = result.fetchone()
            print(f"   [OK] Neon Postgres connected!")
            print(f"   PostgreSQL version: {version[0][:50]}...")

        await engine.dispose()

    except Exception as e:
        print(f"   [ERROR] Neon error: {e}")

if __name__ == "__main__":
    asyncio.run(test_neon())
