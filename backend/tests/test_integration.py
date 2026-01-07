"""
End-to-end integration test for the RAG pipeline
"""
import os
import tempfile
import json
from unittest.mock import Mock, patch
from src.main import main, calculate_content_hash
from src.ingestion.chunker import chunk_text
from src.utils.config import Config


def test_end_to_end_pipeline():
    """Test the complete pipeline from ingestion to storage."""
    print("Testing end-to-end pipeline...")

    # Create a mock configuration
    config = Mock()
    config.COHERE_API_KEY = "test_key"
    config.QDRANT_API_KEY = "test_key"
    config.QDRANT_URL = "https://test.qdrant.com"
    config.COHERE_MODEL = "embed-english-v3.0"
    config.CHUNK_SIZE_MIN = 500
    config.CHUNK_SIZE_MAX = 800
    config.CHUNK_OVERLAP = 100
    config.RATE_LIMIT_DELAY = 0.1
    config.REQUEST_TIMEOUT = 10
    config.QDRANT_COLLECTION_NAME = "test_collection"
    config.VALIDATION_THRESHOLD = 0.99

    # Mock all external dependencies
    with patch('src.utils.config.Config.__init__', return_value=None):
        with patch('src.utils.config.Config._validate_config', return_value=None):
            # Mock the config properties
            with patch('src.utils.config.load_config', return_value=config):
                # Mock all external services
                with patch('src.ingestion.fetcher.requests.get') as mock_get:
                    with patch('src.embedding.generator.cohere_client') as mock_cohere:
                        with patch('src.storage.qdrant_client.qdrant_client') as mock_qdrant:
                            with patch('src.validation.validator.qdrant_client') as mock_validator_qdrant:
                                # Setup mocks
                                mock_get.return_value.text = "<html><body><p>This is a test paragraph for the RAG pipeline. It contains meaningful content that should be processed properly.</p></body></html>"
                                mock_get.return_value.raise_for_status = Mock()

                                mock_cohere.embed.return_value.embeddings = [[0.1, 0.2, 0.3]]

                                mock_qdrant.get_collection.side_effect = Exception("Collection doesn't exist")
                                mock_qdrant.create_collection = Mock()
                                mock_qdrant.upsert = Mock()

                                mock_validator_qdrant.get_collection.return_value.points_count = 1
                                mock_validator_qdrant.scroll.return_value = [(Mock(), None)]
                                mock_validator_qdrant.search.return_value = [Mock()]
                                mock_validator_qdrant.retrieve.return_value = [Mock()]

                                # Test the main function with minimal inputs
                                try:
                                    main(urls=["https://test.com"], incremental=False)
                                    print("✓ End-to-end pipeline test completed successfully")
                                    return True
                                except Exception as e:
                                    print(f"✗ End-to-end pipeline test failed: {str(e)}")
                                    return False


def test_chunking_logic():
    """Test the chunking logic specifically."""
    print("Testing chunking logic...")

    # Test with a longer text to ensure it gets chunked
    long_text = "This is a test sentence. " * 50  # 150 words, should exceed 500 tokens

    # Mock config for chunking
    import src.ingestion.chunker
    from unittest.mock import Mock

    mock_config = Mock()
    mock_config.CHUNK_SIZE_MIN = 50
    mock_config.CHUNK_SIZE_MAX = 100
    mock_config.CHUNK_OVERLAP = 10

    # Temporarily replace config in chunker module
    original_config = src.ingestion.chunker.config
    src.ingestion.chunker.config = mock_config

    try:
        chunks = chunk_text(long_text, metadata={"url": "test_url"})

        # Should have multiple chunks
        assert len(chunks) > 1, f"Expected multiple chunks, got {len(chunks)}"

        # Each chunk should have content and metadata
        for chunk in chunks:
            assert "content" in chunk
            assert "metadata" in chunk
            assert chunk["metadata"]["url"] == "test_url"

        print("✓ Chunking logic test passed")
        return True
    except Exception as e:
        print(f"✗ Chunking logic test failed: {str(e)}")
        return False
    finally:
        # Restore original config
        src.ingestion.chunker.config = original_config


if __name__ == "__main__":
    print("Running end-to-end integration tests...")

    success1 = test_chunking_logic()
    success2 = test_end_to_end_pipeline()

    if success1 and success2:
        print("\n✓ All end-to-end tests passed!")
    else:
        print("\n✗ Some end-to-end tests failed!")