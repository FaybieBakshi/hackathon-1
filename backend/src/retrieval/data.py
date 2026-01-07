"""
Shared data classes for the retrieval system.
"""
from typing import List, Dict, Any
from dataclasses import dataclass


@dataclass
class RetrievedChunk:
    """Represents a retrieved text chunk with metadata."""
    content: str
    score: float
    metadata: Dict[str, Any]
    id: str
    token_count: int


@dataclass
class RetrievalResult:
    """Represents the result of a retrieval operation."""
    query: str
    chunks: List[RetrievedChunk]
    execution_time: float
    total_candidates: int
    confidence_threshold: float
    status: str