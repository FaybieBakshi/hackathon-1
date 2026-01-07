"""
Qdrant Storage Module
Stores embeddings and metadata in Qdrant Cloud.
"""
from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any
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


def store_embeddings(chunks: List[Dict[str, Any]], embeddings: List[List[float]], config) -> None:
    """
    Store chunks and embeddings in Qdrant Cloud.

    Args:
        chunks: List of chunk dictionaries
        embeddings: List of embedding vectors
        config: Configuration object
    """
    if len(chunks) != len(embeddings):
        raise ValueError("Number of chunks must match number of embeddings")

    # Define collection name
    collection_name = config.QDRANT_COLLECTION_NAME

    try:
        # Check if collection exists, create if not
        try:
            qdrant_client.get_collection(collection_name)
            logger.info(f"Collection '{collection_name}' exists")
        except:
            logger.info(f"Creating collection '{collection_name}'")
            qdrant_client.create_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(
                    size=len(embeddings[0]) if embeddings else 384,  # Default size if no embeddings
                    distance=models.Distance.COSINE
                )
            )
            logger.info(f"Collection '{collection_name}' created successfully")

        # Prepare points for insertion
        points = []
        for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
            point = models.PointStruct(
                id=i,
                vector=embedding,
                payload={
                    "content": chunk["content"],
                    "metadata": chunk.get("metadata", {}),
                    "token_count": chunk.get("token_count", 0)
                }
            )
            points.append(point)

        # Upload points to Qdrant
        logger.info(f"Uploading {len(points)} points to Qdrant...")
        qdrant_client.upsert(
            collection_name=collection_name,
            points=points
        )
        logger.info(f"Successfully stored {len(points)} embeddings in Qdrant")

    except Exception as e:
        logger.error(f"Error storing embeddings in Qdrant: {str(e)}")
        raise