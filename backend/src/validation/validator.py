"""
Validation Module
Validates that chunks are correctly stored and retrievable in Qdrant.
"""
from qdrant_client import QdrantClient
from typing import Dict, Any
from src.utils.config import load_config
from src.utils.logger import setup_logger


logger = setup_logger()
config = load_config()

# Initialize Qdrant client
qdrant_client = QdrantClient(
    url=config.QDRANT_URL,
    api_key=config.QDRANT_API_KEY,
    prefer_grpc=True
)


def validate_storage() -> Dict[str, Any]:
    """
    Validate that chunks are correctly stored and retrievable.

    Returns:
        Dictionary with validation results
    """
    collection_name = config.QDRANT_COLLECTION_NAME

    try:
        # Get collection info
        collection_info = qdrant_client.get_collection(collection_name)
        point_count = collection_info.points_count

        logger.info(f"Collection '{collection_name}' has {point_count} points")

        # Perform comprehensive validation
        if point_count > 0:
            # Test retrieval by performing a sample search
            sample_points = qdrant_client.scroll(
                collection_name=collection_name,
                limit=1
            )

            search_success = False
            validation_percentage = 0.0

            if sample_points and len(sample_points[0]) > 0:
                sample_point = sample_points[0][0]  # Get the first point
                sample_vector = sample_point.vector

                # Perform similarity search - check if query_points method exists
                if hasattr(qdrant_client, 'query_points'):
                    try:
                        search_results = qdrant_client.query_points(
                            collection_name=collection_name,
                            query=sample_vector,
                            limit=5
                        )
                        search_success = len(search_results.points) > 0
                    except Exception as search_error:
                        logger.warning(f"Query failed: {str(search_error)}")
                        search_success = False
                else:
                    logger.warning("Qdrant client does not have query_points method")
                    search_success = False

                logger.info(f"Sample search test: {'PASSED' if search_success else 'FAILED'}")

                # For a more thorough validation, check a sample of points
                # In a real implementation, we might validate more points, but for efficiency
                # we'll use a sample-based approach
                sample_size = min(10, point_count)  # Sample up to 10 points
                sample_points = qdrant_client.scroll(
                    collection_name=collection_name,
                    limit=sample_size
                )

                successful_retrievals = 0
                if sample_points and len(sample_points[0]) > 0:
                    for point in sample_points[0]:  # Access points from the first element of the tuple
                        try:
                            # Try to retrieve this specific point by ID - check if retrieve method exists
                            if hasattr(qdrant_client, 'retrieve'):
                                retrieved = qdrant_client.retrieve(
                                    collection_name=collection_name,
                                    ids=[point.id]
                                )
                                if retrieved:
                                    successful_retrievals += 1
                            else:
                                # If retrieve method doesn't exist, try query_points instead
                                if hasattr(qdrant_client, 'query_points'):
                                    search_results = qdrant_client.query_points(
                                        collection_name=collection_name,
                                        query=point.vector,
                                        limit=1
                                    )
                                    if search_results and search_results.points:
                                        successful_retrievals += 1
                        except Exception as retrieval_error:
                            logger.warning(f"Retrieval failed for point {point.id}: {str(retrieval_error)}")
                            # If direct retrieval fails, try query_points as fallback if available
                            if hasattr(qdrant_client, 'query_points'):
                                try:
                                    search_results = qdrant_client.query_points(
                                        collection_name=collection_name,
                                        query=point.vector,
                                        limit=1
                                    )
                                    if search_results and search_results.points:
                                        successful_retrievals += 1
                                except Exception as fallback_error:
                                    logger.warning(f"Fallback query also failed: {str(fallback_error)}")

                validation_percentage = successful_retrievals / sample_size if sample_size > 0 else 0.0
                logger.info(f"Retrieved {successful_retrievals}/{sample_size} sample points successfully ({validation_percentage:.2%})")
            else:
                logger.warning("No points found to validate")
        else:
            search_success = False
            validation_percentage = 0.0
            logger.warning("No points in collection to validate")

        # Prepare validation results
        results = {
            "total_points": point_count,
            "search_functional": search_success,
            "validation_percentage": validation_percentage,
            "threshold_met": validation_percentage >= config.VALIDATION_THRESHOLD,
            "status": "PASS" if (point_count > 0 and search_success and validation_percentage >= config.VALIDATION_THRESHOLD) else "FAIL"
        }

        logger.info(f"Validation results: {results}")

        return results

    except Exception as e:
        logger.error(f"Error during validation: {str(e)}")
        return {
            "total_points": 0,
            "search_functional": False,
            "validation_percentage": 0.0,
            "threshold_met": False,
            "status": "ERROR",
            "error": str(e)
        }