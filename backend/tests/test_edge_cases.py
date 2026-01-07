"""
Test script for edge cases in the RAG pipeline
"""
import pytest
from unittest.mock import Mock, patch
from src.ingestion.fetcher import fetch_urls
from src.ingestion.cleaner import clean_text
from src.ingestion.chunker import chunk_text
from src.utils.config import Config


def test_unavailable_url():
    """Test handling of unavailable URLs."""
    with patch('src.ingestion.fetcher.requests.get') as mock_get:
        mock_get.side_effect = Exception("Connection error")

        result = fetch_urls(["http://unavailable-url.com"])

        # Should return empty string for failed URL
        assert result == [""]


def test_empty_content():
    """Test handling of pages with no text content."""
    # Test cleaner with empty content
    result = clean_text("")
    assert result == ""

    # Test chunker with empty content
    result = chunk_text("")
    assert result == []


def test_large_content():
    """Test handling of extremely large content."""
    # Create large text content
    large_text = "This is a test sentence. " * 10000

    # Test that it can be chunked without errors
    chunks = chunk_text(large_text)
    assert len(chunks) > 0

    # Test that all chunks have content
    for chunk in chunks:
        assert len(chunk["content"]) > 0


def test_config_validation():
    """Test configuration validation with missing environment variables."""
    # Temporarily clear environment variables to test validation
    import os
    original_keys = {}
    for key in ['COHERE_API_KEY', 'QDRANT_API_KEY', 'QDRANT_URL']:
        if key in os.environ:
            original_keys[key] = os.environ[key]
            del os.environ[key]

    try:
        # This should raise a ValueError due to missing required environment variables
        with pytest.raises(ValueError):
            Config()
    finally:
        # Restore original environment variables
        for key, value in original_keys.items():
            os.environ[key] = value


if __name__ == "__main__":
    print("Testing edge cases...")

    test_unavailable_url()
    print("✓ Unavailable URL test passed")

    test_empty_content()
    print("✓ Empty content test passed")

    test_large_content()
    print("✓ Large content test passed")

    # Skip config validation test since it would require changing environment
    print("✓ Edge case tests completed")