"""
Script to check if documents were properly stored in Qdrant
"""
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from src.utils.config import load_config
from qdrant_client import QdrantClient
from src.utils.logger import setup_logger

logger = setup_logger()
config = load_config()

def check_documents():
    """Check if documents are properly stored in Qdrant"""
    logger.info("Checking documents in Qdrant vector database...")

    try:
        # Initialize Qdrant client
        qdrant_client = QdrantClient(
            url=config.QDRANT_URL,
            api_key=config.QDRANT_API_KEY,
            prefer_grpc=True
        )

        # Check if collection exists
        collection_name = config.QDRANT_COLLECTION_NAME
        collection_info = qdrant_client.get_collection(collection_name)

        logger.info(f"Collection '{collection_name}' exists")
        logger.info(f"Points count: {collection_info.points_count}")
        logger.info(f"Collection vectors config: {collection_info.config.params}")

        # If there are points, try to get a few to verify content
        if collection_info.points_count > 0:
            # Get a few points to verify they contain our test content
            points = qdrant_client.scroll(
                collection_name=collection_name,
                limit=5,  # Get first 5 points
                with_payload=True,
                with_vectors=False
            )

            logger.info(f"Retrieved {len(points[0])} points from collection")

            for i, point in enumerate(points[0]):
                logger.info(f"Point {i+1}:")
                logger.info(f"  ID: {point.id}")
                logger.info(f"  Content preview: {point.payload.get('content', '')[:100]}...")
                logger.info(f"  Metadata: {point.payload.get('metadata', {})}")
                logger.info("---")

            return True
        else:
            logger.warning("No points found in the collection")
            return False

    except Exception as e:
        logger.error(f"Error checking documents in Qdrant: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = check_documents()
    if success:
        print("\n✓ Documents found in Qdrant database!")
        print("The system is properly set up with documentation content.")
    else:
        print("\n✗ No documents found in Qdrant database")
        sys.exit(1)