"""
Core retrieval logic for the RAG system.

This module contains the main retrieval classes and functions for:
- Query processing and embedding
- Vector database interaction
- Result filtering and ranking
"""
import logging
import time
from typing import List, Dict, Any, Optional
from dataclasses import dataclass

import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models

from src.utils.config import load_config
from src.utils.logger import setup_logger
from src.retrieval.data import RetrievedChunk, RetrievalResult
from src.retrieval.filters import filter_duplicate_content




# Initialize clients and configuration
logger = setup_logger()
config = load_config()

# Initialize Cohere client for query embeddings
cohere_client = cohere.Client(config.COHERE_API_KEY)

# Initialize Qdrant client for vector search
qdrant_client = QdrantClient(
    url=config.QDRANT_URL,
    api_key=config.QDRANT_API_KEY,
    prefer_grpc=True
)


class Retriever:
    """Main class for handling retrieval operations."""

    def __init__(self):
        self.config = load_config()
        self.cohere_client = cohere.Client(self.config.COHERE_API_KEY)
        self.qdrant_client = QdrantClient(
            url=self.config.QDRANT_URL,
            api_key=self.config.QDRANT_API_KEY,
            prefer_grpc=True
        )

    def generate_query_embedding(self, query_text: str, model: str = None) -> List[float]:
        """
        Generate embedding for query text using Cohere.

        Args:
            query_text: The query text to embed
            model: Embedding model to use (defaults to config value)

        Returns:
            Embedding vector as a list of floats
        """
        model = model or self.config.COHERE_MODEL

        try:
            logger.info(f"Generating embedding for query: {query_text[:50]}...")

            # Add retry logic for rate limiting
            max_retries = 5
            retry_count = 0

            while retry_count < max_retries:
                try:
                    response = self.cohere_client.embed(
                        texts=[query_text],
                        model=model,
                        input_type="search_query"  # Optimize for search queries
                    )
                    break
                except cohere.errors.TooManyRequestsError as e:
                    retry_count += 1
                    if retry_count >= max_retries:
                        logger.error(f"Max retries reached for query embedding generation: {str(e)}")
                        raise
                    else:
                        delay = self.config.RATE_LIMIT_DELAY * (2 ** retry_count)  # Exponential backoff
                        logger.warning(f"Rate limited (429), waiting {delay}s before retry {retry_count}/{max_retries}")
                        time.sleep(delay)
                except Exception as e:
                    logger.error(f"Error generating query embedding: {str(e)}")
                    raise

            embedding = response.embeddings[0]
            logger.info(f"Generated embedding with {len(embedding)} dimensions")

            return embedding

        except Exception as e:
            logger.error(f"Error generating query embedding: {str(e)}")
            raise

    def retrieve_chunks(
        self,
        query: str,
        top_k: int = 5,
        min_score: float = 0.3,
        use_cache: bool = True,
        filter_duplicates: bool = True
    ) -> RetrievalResult:
        """
        Retrieve the most relevant text chunks for a given query.

        Args:
            query: The search query
            top_k: Number of top results to return
            min_score: Minimum confidence score threshold
            use_cache: Whether to use caching (placeholder for future implementation)
            filter_duplicates: Whether to filter duplicate content

        Returns:
            RetrievalResult containing the results and metadata
        """
        start_time = time.time()

        try:
            logger.info(f"Starting retrieval for query: {query}")

            # Validate query
            if not query or not query.strip():
                execution_time = time.time() - start_time
                logger.warning("Empty or whitespace-only query provided")
                return RetrievalResult(
                    query=query,
                    chunks=[],
                    execution_time=execution_time * 1000,
                    total_candidates=0,
                    confidence_threshold=min_score,
                    status="no_query"
                )

            # Check for extremely long queries
            if len(query) > 1000:  # Arbitrary threshold, can be configured
                logger.warning(f"Query is very long ({len(query)} chars), may affect performance")
                # Truncate or handle as needed
                query = query[:1000]

            # Generate embedding for the query
            query_embedding = self.generate_query_embedding(query)

            # Query Qdrant for similar vectors
            collection_name = self.config.QDRANT_COLLECTION_NAME
            try:
                search_results = self.qdrant_client.query_points(
                    collection_name=collection_name,
                    query=query_embedding,
                    limit=top_k * 2,  # Get more results to account for filtering
                    score_threshold=min_score
                )
            except Exception as qdrant_error:
                execution_time = time.time() - start_time
                logger.error(f"Qdrant query failed: {str(qdrant_error)}")
                return RetrievalResult(
                    query=query,
                    chunks=[],
                    execution_time=execution_time * 1000,
                    total_candidates=0,
                    confidence_threshold=min_score,
                    status="qdrant_unavailable"
                )

            # Convert search results to RetrievedChunk objects
            chunks = []
            for result in search_results.points:  # query_points returns QueryResponse with points attribute
                chunk = RetrievedChunk(
                    content=result.payload.get("content", ""),
                    score=result.score,
                    metadata=result.payload.get("metadata", {}),
                    id=str(result.id),
                    token_count=result.payload.get("token_count", 0)
                )
                chunks.append(chunk)

            # Filter duplicates if requested
            if filter_duplicates:
                chunks = filter_duplicate_content(chunks)

            # Handle low-confidence matches
            high_confidence_chunks = [chunk for chunk in chunks if chunk.score >= min_score]
            if not high_confidence_chunks and chunks:
                logger.info(f"Found {len(chunks)} results but none meet minimum confidence threshold of {min_score}")
                # Return all results with a warning status
                result_status = "low_confidence"
            else:
                result_status = "success" if chunks else "no_results"

            # Sort by score in descending order and limit to top_k
            chunks.sort(key=lambda x: x.score, reverse=True)
            chunks = chunks[:top_k]

            execution_time = time.time() - start_time

            result = RetrievalResult(
                query=query,
                chunks=chunks,
                execution_time=execution_time * 1000,  # Convert to milliseconds
                total_candidates=len(search_results.points),
                confidence_threshold=min_score,
                status=result_status
            )

            logger.info(f"Retrieved {len(chunks)} chunks in {result.execution_time:.2f}ms")

            return result

        except Exception as e:
            execution_time = time.time() - start_time
            logger.error(f"Error during retrieval: {str(e)}", exc_info=True)

            return RetrievalResult(
                query=query,
                chunks=[],
                execution_time=execution_time * 1000,
                total_candidates=0,
                confidence_threshold=min_score,
                status="error"
            )

    def query_qdrant(
        self,
        query: str,
        top_k: int = 5,
        min_score: float = 0.3,
        filter_duplicates: bool = True
    ) -> List[Dict[str, Any]]:
        """
        Query Qdrant directly and return formatted results.

        Args:
            query: The search query
            top_k: Number of top results to return
            min_score: Minimum confidence score threshold
            filter_duplicates: Whether to filter duplicate content

        Returns:
            List of dictionaries containing chunk information
        """
        result = self.retrieve_chunks(query, top_k, min_score, filter_duplicates=filter_duplicates)

        formatted_results = []
        for chunk in result.chunks:
            formatted_results.append({
                "content": chunk.content,
                "score": chunk.score,
                "metadata": chunk.metadata,
                "id": chunk.id,
                "token_count": chunk.token_count
            })

        return formatted_results


# Global retriever instance for backward compatibility
global_retriever = Retriever()


def retrieve_chunks(
    query: str,
    top_k: int = 5,
    min_score: float = 0.3,
    use_cache: bool = True
) -> RetrievalResult:
    """
    Convenience function to retrieve chunks using the global retriever.

    Args:
        query: The search query
        top_k: Number of top results to return
        min_score: Minimum confidence score threshold
        use_cache: Whether to use caching

    Returns:
        RetrievalResult containing the results and metadata
    """
    return global_retriever.retrieve_chunks(query, top_k, min_score, use_cache)


def query_qdrant(query: str, top_k: int = 5, min_score: float = 0.3) -> List[Dict[str, Any]]:
    """
    Convenience function to query Qdrant using the global retriever.

    Args:
        query: The search query
        top_k: Number of top results to return
        min_score: Minimum confidence score threshold

    Returns:
        List of dictionaries containing chunk information
    """
    return global_retriever.query_qdrant(query, top_k, min_score)