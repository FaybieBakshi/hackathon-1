"""
Citation management for the RAG agent.

This module handles citation generation and mapping between inline citations
and their corresponding source chunks.
"""
from typing import Dict, List
from dataclasses import dataclass

from src.retrieval.retriever import RetrievedChunk


@dataclass
class CitationMap:
    """
    Maps citation markers to source chunks and vice versa.
    """
    mappings: Dict[str, RetrievedChunk]  # Maps citation markers to source chunks
    next_index: int = 1  # Next index to use for citation
    chunk_to_citation: Dict[str, str] = None  # Reverse mapping from chunk IDs to citations

    def __post_init__(self):
        if self.chunk_to_citation is None:
            self.chunk_to_citation = {}


class CitationManager:
    """
    Manages citation generation and mapping for the RAG agent.
    """
    def __init__(self):
        self.citation_format = "[{index}]"

    def create_citation_map(self, chunks: List[RetrievedChunk]) -> CitationMap:
        """
        Create a citation map from a list of retrieved chunks.

        Args:
            chunks: List of retrieved chunks to create citations for

        Returns:
            CitationMap object mapping citation markers to chunks
        """
        mappings = {}
        chunk_to_citation = {}

        for i, chunk in enumerate(chunks, 1):
            citation_marker = f"[{i}]"
            mappings[citation_marker] = chunk
            chunk_to_citation[chunk.id] = citation_marker

        return CitationMap(
            mappings=mappings,
            next_index=len(chunks) + 1,
            chunk_to_citation=chunk_to_citation
        )

    def generate_citation_text(self, text: str, citation_map: CitationMap) -> str:
        """
        Generate text with appropriate citations embedded.

        This is a simplified approach - in a real implementation, we'd analyze
        the generated text to determine which citations to include.

        Args:
            text: The text that may need citations
            citation_map: The citation map to use for references

        Returns:
            Text with citations appropriately added
        """
        # In a real implementation, we would analyze the text to see which
        # chunks were referenced and add appropriate citations
        # For now, we'll just return the text as is
        return text

    def validate_citations(self, answer: str, citation_map: CitationMap) -> Dict[str, bool]:
        """
        Validate that citations in the answer properly reference the citation map.

        Args:
            answer: The answer text that may contain citations
            citation_map: The citation map to validate against

        Returns:
            Dictionary with validation results for each citation found
        """
        validation_results = {}

        # Find all citation markers in the answer
        import re
        citation_pattern = r'\[(\d+)\]'
        found_citations = re.findall(citation_pattern, answer)

        for citation_num in found_citations:
            citation_marker = f"[{citation_num}]"
            is_valid = citation_marker in citation_map.mappings
            validation_results[citation_marker] = is_valid

        return validation_results

    def format_citation_references(self, citation_map: CitationMap) -> str:
        """
        Format citation references for inclusion in the context.

        Args:
            citation_map: The citation map to format references for

        Returns:
            Formatted string with citation references
        """
        references = []
        for citation, chunk in citation_map.mappings.items():
            content_preview = chunk.content[:200]  # First 200 chars as preview
            reference = f"{citation}: {content_preview}..."
            references.append(reference)

        return "\n".join(references)

    def get_chunk_by_citation(self, citation: str, citation_map: CitationMap) -> RetrievedChunk:
        """
        Get the chunk corresponding to a citation marker.

        Args:
            citation: The citation marker (e.g., "[1]")
            citation_map: The citation map to look up the citation in

        Returns:
            The corresponding RetrievedChunk or None if not found
        """
        return citation_map.mappings.get(citation)

    def get_citation_by_chunk_id(self, chunk_id: str, citation_map: CitationMap) -> str:
        """
        Get the citation marker for a given chunk ID.

        Args:
            chunk_id: The ID of the chunk
            citation_map: The citation map to look up the citation in

        Returns:
            The corresponding citation marker or None if not found
        """
        return citation_map.chunk_to_citation.get(chunk_id)


# Global citation manager instance
citation_manager = CitationManager()