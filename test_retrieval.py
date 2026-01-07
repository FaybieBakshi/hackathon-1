#!/usr/bin/env python3
"""
Test script to verify the retrieval system works properly.
"""
import os
import sys

# Add backend to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from backend.src.retrieval.retriever import retrieve_chunks
from backend.src.utils.config import load_config
from backend.src.utils.logger import setup_logger

logger = setup_logger()
config = load_config()

def test_retrieval():
    """Test the retrieval system directly."""
    print("Testing retrieval system...")

    # Test with a query that should match our documents
    query = "humanoid robots"
    print(f"Query: {query}")

    result = retrieve_chunks(
        query=query,
        top_k=5,
        min_score=0.1
    )

    print(f"Status: {result.status}")
    print(f"Number of chunks retrieved: {len(result.chunks)}")
    print(f"Execution time: {result.execution_time:.2f}ms")
    print(f"Total candidates: {result.total_candidates}")

    if result.chunks:
        print("\nRetrieved chunks:")
        for i, chunk in enumerate(result.chunks):
            print(f"\nChunk {i+1}:")
            print(f"  Score: {chunk.score}")
            print(f"  Content preview: {chunk.content[:200]}...")
            print(f"  Metadata: {chunk.metadata}")
    else:
        print("\nNo chunks retrieved - this indicates an issue with the retrieval system.")

    # Test with another query
    print("\n" + "="*50)
    query2 = "Physical AI"
    print(f"Query: {query2}")

    result2 = retrieve_chunks(
        query=query2,
        top_k=5,
        min_score=0.1
    )

    print(f"Status: {result2.status}")
    print(f"Number of chunks retrieved: {len(result2.chunks)}")
    print(f"Execution time: {result2.execution_time:.2f}ms")
    print(f"Total candidates: {result2.total_candidates}")

    if result2.chunks:
        print("\nRetrieved chunks:")
        for i, chunk in enumerate(result2.chunks):
            print(f"\nChunk {i+1}:")
            print(f"  Score: {chunk.score}")
            print(f"  Content preview: {chunk.content[:200]}...")
            print(f"  Metadata: {chunk.metadata}")
    else:
        print("\nNo chunks retrieved.")

if __name__ == "__main__":
    test_retrieval()