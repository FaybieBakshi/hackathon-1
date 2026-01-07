"""
Simple test to verify retrieval from the vector database
"""
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from src.retrieval.retriever import Retriever
from src.utils.logger import setup_logger

logger = setup_logger()

def test_retrieval():
    """Test retrieval functionality directly"""
    logger.info("Testing retrieval from vector database...")

    try:
        # Create a retriever instance
        retriever = Retriever()

        # Test different queries
        queries = ["Physical AI", "robotics", "computer vision", "embodied AI", "motion planning"]

        for query in queries:
            logger.info(f"Testing retrieval for query: '{query}'")

            # Perform retrieval
            result = retriever.retrieve_chunks(query, top_k=5, min_score=0.1)

            logger.info(f"Query '{query}' - Status: {result.status}, Chunks found: {len(result.chunks)}")

            if result.chunks:
                logger.info(f"  Found {len(result.chunks)} relevant chunks:")
                for i, chunk in enumerate(result.chunks[:2]):  # Show first 2 chunks
                    logger.info(f"    Chunk {i+1}: {chunk.content[:100]}...")
                    logger.info(f"    Score: {chunk.score}")
                    logger.info(f"    Metadata: {chunk.metadata}")
                if len(result.chunks) > 2:
                    logger.info(f"    ... and {len(result.chunks) - 2} more")
            else:
                logger.info(f"  No chunks found for query '{query}'")

            logger.info("---")

        return True

    except Exception as e:
        logger.error(f"Error during retrieval test: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_retrieval()
    if success:
        print("\nRetrieval test completed!")
    else:
        print("\nRetrieval test failed!")
        sys.exit(1)