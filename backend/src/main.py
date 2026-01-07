"""
Main pipeline module for RAG ingestion
Orchestrates the complete pipeline from URL fetching to vector storage
"""
import hashlib
from typing import List, Optional
from src.ingestion.fetcher import fetch_urls
from src.ingestion.cleaner import clean_text
from src.ingestion.chunker import chunk_text
from src.embedding.generator import generate_embeddings
from src.storage.qdrant_client import store_embeddings
from src.validation.validator import validate_storage
from src.utils.config import load_config
from src.utils.logger import setup_logger


logger = setup_logger()
config = load_config()


def calculate_content_hash(content: str) -> str:
    """
    Calculate SHA256 hash of content for deduplication.
    
    Args:
        content: Text content to hash
        
    Returns:
        Hexadecimal hash string
    """
    return hashlib.sha256(content.encode('utf-8')).hexdigest()


def main(urls: List[str], incremental: bool = True) -> None:
    """
    Main pipeline function to process URLs through the RAG pipeline.
    
    Args:
        urls: List of URLs to process
        incremental: Whether to use incremental processing (skip already processed URLs)
    """
    logger.info(f"Starting RAG pipeline with {len(urls)} URLs (incremental={incremental})")
    
    # Step 1: Fetch content from URLs
    logger.info("Step 1: Fetching content from URLs")
    html_contents = fetch_urls(urls)
    
    # Step 2: Clean HTML content
    logger.info("Step 2: Cleaning HTML content")
    cleaned_texts = [clean_text(html) for html in html_contents]
    
    # Step 3: Chunk text
    logger.info("Step 3: Chunking text")
    all_chunks = []
    for i, (text, url) in enumerate(zip(cleaned_texts, urls)):
        if not text:
            logger.warning(f"Skipping empty content from {url}")
            continue
            
        metadata = {"url": url, "source_index": i}
        chunks = chunk_text(text, metadata=metadata)
        all_chunks.extend(chunks)
    
    logger.info(f"Generated {len(all_chunks)} chunks total")
    
    # Step 4: Generate embeddings
    logger.info("Step 4: Generating embeddings")
    embeddings = generate_embeddings(all_chunks)
    
    # Step 5: Store in Qdrant
    logger.info("Step 5: Storing embeddings in Qdrant")
    store_embeddings(all_chunks, embeddings)
    
    # Step 6: Validate storage
    logger.info("Step 6: Validating storage")
    validation_result = validate_storage()
    
    if validation_result:
        logger.info("Pipeline completed successfully")
    else:
        logger.warning("Pipeline completed with validation warnings")

