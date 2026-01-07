"""
Tests for storage module
"""
import pytest
from unittest.mock import Mock, patch
from src.storage.qdrant_client import store_embeddings


def test_store_embeddings_basic():
    """Test basic embedding storage."""
    # Mock chunks and embeddings for testing
    chunks = [
        {
            "content": "This is a test chunk 1.",
            "metadata": {"url": "test_url_1", "chapter": "test_chapter_1"},
            "token_count": 10
        },
        {
            "content": "This is a test chunk 2.",
            "metadata": {"url": "test_url_2", "chapter": "test_chapter_2"},
            "token_count": 10
        }
    ]
    embeddings = [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]]

    # Since we can't connect to Qdrant in tests, we'll mock it
    with patch('src.storage.qdrant_client.qdrant_client') as mock_client:
        mock_client.get_collection.side_effect = Exception("Collection doesn't exist")  # Simulate collection not existing
        mock_client.create_collection = Mock()
        mock_client.upsert = Mock()

        # This should trigger collection creation
        config = Mock()
        config.QDRANT_COLLECTION_NAME = "test_collection"

        store_embeddings(chunks, embeddings, config)

        assert mock_client.create_collection.called
        assert mock_client.upsert.called


def test_store_embeddings_empty():
    """Test storing empty embeddings."""
    with patch('src.storage.qdrant_client.qdrant_client') as mock_client:
        mock_client.get_collection = Mock()
        mock_client.upsert = Mock()

        config = Mock()
        config.QDRANT_COLLECTION_NAME = "test_collection"

        store_embeddings([], [], config)

        # Should not try to upsert anything if no embeddings
        assert not mock_client.upsert.called