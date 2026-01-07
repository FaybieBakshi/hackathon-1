"""
Tests for embedding module
"""
import pytest
from unittest.mock import Mock, patch
from src.embedding.generator import generate_embeddings


def test_generate_embeddings_basic():
    """Test basic embedding generation."""
    # Mock chunks for testing
    chunks = [
        {"content": "This is a test chunk 1."},
        {"content": "This is a test chunk 2."}
    ]

    # Since we can't call the actual Cohere API in tests, we'll mock it
    with patch('src.embedding.generator.cohere_client') as mock_client:
        mock_response = Mock()
        mock_response.embeddings = [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]]
        mock_client.embed.return_value = mock_response

        embeddings = generate_embeddings(chunks)

        assert len(embeddings) == 2
        assert mock_client.embed.called


def test_generate_embeddings_empty():
    """Test embedding generation with empty chunks."""
    embeddings = generate_embeddings([])
    assert embeddings == []