"""
Shared data classes for the RAG agent.
"""
from typing import List, Dict, Any, Optional
from dataclasses import dataclass


# Data classes for agent responses
@dataclass
class AgentRequest:
    """Represents an agent request with query and context."""
    query: str
    conversation_history: List[Dict[str, str]] = None
    retrieved_chunks: List = None  # Using RetrievedChunk from retrieval.data
    max_tokens: int = 4096
    temperature: float = 0.3
    citation_format: str = "[{index}]"


@dataclass
class AgentResponse:
    """Represents the agent's response."""
    answer: str
    citations: Dict[str, Any]  # Will contain RetrievedChunk objects
    confidence_score: float
    token_usage: Dict[str, int]
    fallback_used: bool
    status: str


@dataclass
class ValidationResult:
    """Represents validation results for agent responses."""
    is_valid: bool
    issues: List[str]
    source_compliance: bool
    citation_accuracy: float
    content_relevance: float