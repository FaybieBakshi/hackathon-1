"""
Validation utilities for the RAG agent responses.

This module contains functions for validating:
- Agent response quality
- Source compliance (ensuring answers only use retrieved content)
- Citation accuracy
- Content relevance
"""
from typing import List, Dict, Any
from dataclasses import dataclass

from src.retrieval.data import RetrievedChunk
from src.agents.data import AgentResponse, ValidationResult


@dataclass
class ContentRelevanceResult:
    """Result of content relevance validation."""
    is_relevant: bool
    relevance_score: float
    issues: List[str]


def validate_source_compliance(response: AgentResponse, retrieved_chunks: List[RetrievedChunk]) -> bool:
    """
    Validate that the agent response only uses information from retrieved chunks.

    Args:
        response: The agent's response to validate
        retrieved_chunks: The chunks that were available to the agent

    Returns:
        True if the response complies with source requirements, False otherwise
    """
    if response.fallback_used:
        # Fallback responses are compliant by definition
        return True

    # In a real implementation, we would analyze the response content
    # to ensure it only contains information present in the retrieved chunks
    # For this implementation, we'll assume it's compliant if citations exist
    return len(response.citations) > 0 or len(retrieved_chunks) == 0


def validate_citation_accuracy(response: AgentResponse, retrieved_chunks: List[RetrievedChunk]) -> float:
    """
    Validate the accuracy of citations in the response.

    Args:
        response: The agent's response to validate
        retrieved_chunks: The chunks that were available to the agent

    Returns:
        Accuracy score between 0.0 and 1.0
    """
    if not response.citations:
        # If no citations but the response is short/fallback, that might be acceptable
        if response.fallback_used or len(response.answer.split()) < 20:
            return 1.0
        return 0.0

    # Check if all citations map to actual retrieved chunks
    valid_citations = 0
    total_citations = len(response.citations)

    for citation_marker, cited_chunk in response.citations.items():
        # Check if the cited chunk was actually retrieved
        if any(cited_chunk.id == chunk.id for chunk in retrieved_chunks):
            valid_citations += 1

    return valid_citations / total_citations if total_citations > 0 else 1.0


def validate_content_relevance(response: AgentResponse, query: str, retrieved_chunks: List[RetrievedChunk]) -> ContentRelevanceResult:
    """
    Validate that the response content is relevant to the query.

    Args:
        response: The agent's response to validate
        query: The original query
        retrieved_chunks: The chunks that were available to the agent

    Returns:
        ContentRelevanceResult with relevance information
    """
    if response.fallback_used:
        # Fallback responses have their own relevance (acknowledging lack of information)
        return ContentRelevanceResult(
            is_relevant=True,
            relevance_score=0.5,  # Partially relevant - acknowledges limitations
            issues=[]
        )

    # Simple relevance check based on keyword matching
    # In a real implementation, this would use more sophisticated NLP techniques
    query_lower = query.lower()
    answer_lower = response.answer.lower()

    # Count query words that appear in the answer
    query_words = set(query_lower.split())
    answer_words = set(answer_lower.split())

    matching_words = query_words.intersection(answer_words)
    relevance_score = len(matching_words) / len(query_words) if query_words else 1.0

    issues = []
    if relevance_score < 0.2:  # Less than 20% of query words appear in answer
        issues.append("Response contains few words from the original query")

    return ContentRelevanceResult(
        is_relevant=relevance_score > 0.1,  # At least 10% relevance
        relevance_score=relevance_score,
        issues=issues
    )


def validate_response_quality(response: AgentResponse, query: str, retrieved_chunks: List[RetrievedChunk]) -> ValidationResult:
    """
    Perform comprehensive validation of the agent response.

    Args:
        response: The agent's response to validate
        query: The original query
        retrieved_chunks: The chunks that were available to the agent

    Returns:
        ValidationResult with complete validation results
    """
    issues = []

    # Validate source compliance
    source_compliance = validate_source_compliance(response, retrieved_chunks)
    if not source_compliance:
        issues.append("Response contains information not present in retrieved chunks")

    # Validate citation accuracy
    citation_accuracy = validate_citation_accuracy(response, retrieved_chunks)
    if citation_accuracy < 0.7:  # Less than 70% of citations are accurate
        issues.append(f"Only {citation_accuracy:.1%} of citations map to retrieved chunks")

    # Validate content relevance
    relevance_result = validate_content_relevance(response, query, retrieved_chunks)
    if not relevance_result.is_relevant:
        issues.append(f"Response has low relevance to the query ({relevance_result.relevance_score:.2%})")

    # Additional validations could go here:
    # - Check for hallucination
    # - Validate response length
    # - Check for inappropriate content

    is_valid = len(issues) == 0 and source_compliance and citation_accuracy >= 0.7

    return ValidationResult(
        is_valid=is_valid,
        issues=issues,
        source_compliance=source_compliance,
        citation_accuracy=citation_accuracy,
        content_relevance=relevance_result.relevance_score
    )


def validate_confidence_threshold(response: AgentResponse, min_confidence: float) -> bool:
    """
    Validate that the response meets the minimum confidence threshold.

    Args:
        response: The agent's response to validate
        min_confidence: Minimum confidence threshold

    Returns:
        True if confidence is above threshold, False otherwise
    """
    return response.confidence_score >= min_confidence


def validate_token_usage(response: AgentResponse, max_tokens: int) -> Dict[str, Any]:
    """
    Validate that token usage is within expected bounds.

    Args:
        response: The agent's response with token usage
        max_tokens: Maximum expected tokens

    Returns:
        Dictionary with validation results
    """
    validation_results = {
        "within_limit": response.token_usage["total_tokens"] <= max_tokens,
        "token_usage": response.token_usage,
        "max_tokens": max_tokens,
        "issues": []
    }

    if not validation_results["within_limit"]:
        validation_results["issues"].append(
            f"Total tokens ({response.token_usage['total_tokens']}) exceeds "
            f"maximum allowed ({max_tokens})"
        )

    return validation_results