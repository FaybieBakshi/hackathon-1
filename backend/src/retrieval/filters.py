"""
Filter functions for retrieval results.

This module contains functions for:
- Duplicate content filtering
- Confidence score filtering
- Result ranking and ordering
"""
from typing import List, Dict, Any
from src.retrieval.data import RetrievedChunk


def filter_duplicate_content(chunks: List[RetrievedChunk]) -> List[RetrievedChunk]:
    """
    Filter out duplicate content from the retrieved chunks.

    Args:
        chunks: List of retrieved chunks

    Returns:
        List of chunks with duplicates removed
    """
    seen_content = set()
    unique_chunks = []

    for chunk in chunks:
        # Create a hashable representation of the content to detect duplicates
        content_key = chunk.content.strip().lower()[:100]  # Use first 100 chars as a simple key

        if content_key not in seen_content:
            seen_content.add(content_key)
            unique_chunks.append(chunk)

    return unique_chunks


def filter_by_confidence(chunks: List[RetrievedChunk], min_score: float) -> List[RetrievedChunk]:
    """
    Filter chunks based on minimum confidence score.

    Args:
        chunks: List of retrieved chunks
        min_score: Minimum confidence score threshold

    Returns:
        List of chunks with scores above the threshold
    """
    return [chunk for chunk in chunks if chunk.score >= min_score]


def filter_by_metadata(chunks: List[RetrievedChunk], filters: Dict[str, Any]) -> List[RetrievedChunk]:
    """
    Filter chunks based on metadata criteria.

    Args:
        chunks: List of retrieved chunks
        filters: Dictionary of metadata filters

    Returns:
        List of chunks that match the filter criteria
    """
    filtered_chunks = []

    for chunk in chunks:
        match = True
        for key, value in filters.items():
            if key not in chunk.metadata or chunk.metadata[key] != value:
                match = False
                break

        if match:
            filtered_chunks.append(chunk)

    return filtered_chunks


def rank_by_score(chunks: List[RetrievedChunk]) -> List[RetrievedChunk]:
    """
    Rank chunks by their confidence score in descending order.

    Args:
        chunks: List of retrieved chunks

    Returns:
        List of chunks ranked by score (highest first)
    """
    return sorted(chunks, key=lambda x: x.score, reverse=True)


def rank_by_length(chunks: List[RetrievedChunk], ascending: bool = False) -> List[RetrievedChunk]:
    """
    Rank chunks by their content length.

    Args:
        chunks: List of retrieved chunks
        ascending: Whether to sort in ascending order (shortest first) or descending (longest first)

    Returns:
        List of chunks ranked by length
    """
    return sorted(chunks, key=lambda x: len(x.content), reverse=not ascending)


def apply_all_filters(
    chunks: List[RetrievedChunk],
    min_score: float = 0.0,
    filters: Dict[str, Any] = None,
    remove_duplicates: bool = True
) -> List[RetrievedChunk]:
    """
    Apply all available filters to the chunks.

    Args:
        chunks: List of retrieved chunks
        min_score: Minimum confidence score threshold
        filters: Dictionary of metadata filters
        remove_duplicates: Whether to remove duplicate content

    Returns:
        List of filtered chunks
    """
    # Apply confidence filtering
    filtered_chunks = filter_by_confidence(chunks, min_score)

    # Apply metadata filtering if provided
    if filters:
        filtered_chunks = filter_by_metadata(filtered_chunks, filters)

    # Remove duplicates if requested
    if remove_duplicates:
        filtered_chunks = filter_duplicate_content(filtered_chunks)

    return filtered_chunks