"""
Text Chunking Module
Chunks text into semantic segments with overlap and metadata preservation.
"""
import tiktoken
from typing import List, Dict, Any
from src.utils.config import load_config
from src.utils.logger import setup_logger


logger = setup_logger()
config = load_config()

# Global encoding instance - Initialize lazily to avoid network calls during import
_encoding = None


def get_encoding():
    """Get the tiktoken encoding instance, creating it lazily if needed."""
    global _encoding
    if _encoding is None:
        try:
            _encoding = tiktoken.encoding_for_model("gpt-3.5-turbo")
        except:
            _encoding = tiktoken.get_encoding("cl100k_base")
    return _encoding


def chunk_text(text: str, metadata: Dict[str, Any] = None) -> List[Dict[str, Any]]:
    """
    Split text into meaningful chunks with overlap.

    Args:
        text: Input text to chunk
        metadata: Additional metadata to include with each chunk

    Returns:
        List of chunk dictionaries with content and metadata
    """
    if not text:
        return []

    # Initialize tokenizer
    encoding = get_encoding()

    # Encode the text
    tokens = encoding.encode(text)
    logger.info(f"Text encoded into {len(tokens)} tokens")

    # Calculate chunk parameters
    chunk_size = min(config.CHUNK_SIZE_MAX, len(tokens))
    chunk_overlap = config.CHUNK_OVERLAP

    chunks = []
    start_idx = 0

    while start_idx < len(tokens):
        # Determine end index for this chunk
        end_idx = start_idx + chunk_size

        # Ensure we don't exceed the text length
        if end_idx > len(tokens):
            end_idx = len(tokens)

        # Extract token chunk
        chunk_tokens = tokens[start_idx:end_idx]

        # Decode back to text
        chunk_text = encoding.decode(chunk_tokens)

        # Create chunk object
        chunk_obj = {
            "content": chunk_text,
            "token_count": len(chunk_tokens),
            "metadata": metadata or {},
            "overlap_with_next": end_idx < len(tokens)  # Indicates if there's a next chunk
        }

        chunks.append(chunk_obj)

        # Move to next chunk position with overlap
        start_idx = end_idx - chunk_overlap if end_idx < len(tokens) else end_idx

        # Ensure we make progress to avoid infinite loops
        if start_idx == end_idx:
            start_idx += 1

    # Validate chunk sizes
    for i, chunk in enumerate(chunks):
        if chunk["token_count"] < config.CHUNK_SIZE_MIN and len(chunks) > 1:
            logger.warning(f"Chunk {i} has {chunk['token_count']} tokens, below minimum of {config.CHUNK_SIZE_MIN}")

    logger.info(f"Generated {len(chunks)} chunks from text")
    return chunks