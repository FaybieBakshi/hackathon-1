"""
Tests for chunking module
"""
import pytest
from src.ingestion.chunker import chunk_text


def test_chunk_text_basic():
    """Test basic text chunking functionality."""
    text = "This is a test sentence. " * 20  # Create a longer text
    metadata = {"url": "test_url", "chapter": "test_chapter"}

    chunks = chunk_text(text, metadata)

    assert len(chunks) > 0
    assert "content" in chunks[0]
    assert "metadata" in chunks[0]
    assert chunks[0]["metadata"]["url"] == "test_url"


def test_chunk_text_empty():
    """Test chunking with empty text."""
    chunks = chunk_text("")
    assert chunks == []


def test_chunk_text_with_metadata():
    """Test that metadata is preserved in chunks."""
    text = "This is a test sentence. " * 10
    metadata = {"url": "test_url", "chapter": "test_chapter", "section": "test_section"}

    chunks = chunk_text(text, metadata)

    assert len(chunks) > 0
    for chunk in chunks:
        assert chunk["metadata"]["url"] == "test_url"
        assert chunk["metadata"]["chapter"] == "test_chapter"