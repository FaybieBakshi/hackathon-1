#!/usr/bin/env python3
"""
Document Ingestion Script
Indexes documents into the Qdrant vector database for RAG system.
"""
import os
import sys
import glob
from pathlib import Path

# Add backend to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from backend.src.ingestion.chunker import chunk_text
from backend.src.embedding.generator import generate_embeddings
from backend.src.storage.qdrant_client import store_embeddings
from backend.src.utils.config import load_config
from backend.src.utils.logger import setup_logger

logger = setup_logger()
config = load_config()

def read_document(file_path):
    """Read content from a document file."""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    return content

def ingest_documents(docs_dir):
    """Ingest all documents from the specified directory."""
    logger.info(f"Starting document ingestion from: {docs_dir}")

    # Find all markdown files
    doc_files = glob.glob(os.path.join(docs_dir, "*.md"))
    logger.info(f"Found {len(doc_files)} documents to process")

    if not doc_files:
        logger.warning("No documents found to process")
        return

    all_chunks = []

    for doc_file in doc_files:
        logger.info(f"Processing document: {doc_file}")

        try:
            # Read document content
            content = read_document(doc_file)

            # Create metadata for the document
            metadata = {
                "source": doc_file,
                "filename": os.path.basename(doc_file),
                "title": Path(doc_file).stem.replace('_', ' ').title()
            }

            # Chunk the document
            chunks = chunk_text(content, metadata)
            logger.info(f"Generated {len(chunks)} chunks from {doc_file}")

            # Add document-specific metadata to each chunk
            for chunk in chunks:
                chunk["metadata"]["source_document"] = doc_file
                chunk["metadata"]["document_title"] = metadata["title"]

            all_chunks.extend(chunks)

        except Exception as e:
            logger.error(f"Error processing document {doc_file}: {str(e)}")
            continue

    if not all_chunks:
        logger.error("No chunks were generated from any documents")
        return

    logger.info(f"Total chunks generated: {len(all_chunks)}")

    # Generate embeddings for all chunks
    logger.info("Generating embeddings...")
    try:
        embeddings = generate_embeddings(all_chunks)
        logger.info(f"Generated {len(embeddings)} embeddings")
    except Exception as e:
        logger.error(f"Error generating embeddings: {str(e)}")
        return

    # Store embeddings in Qdrant
    logger.info("Storing embeddings in Qdrant...")
    try:
        store_embeddings(all_chunks, embeddings, config)
        logger.info("Successfully stored embeddings in Qdrant")
    except Exception as e:
        logger.error(f"Error storing embeddings in Qdrant: {str(e)}")
        return

    logger.info("Document ingestion completed successfully!")

if __name__ == "__main__":
    # Use the sample_docs directory
    docs_directory = os.path.join(os.path.dirname(__file__), "sample_docs")

    if not os.path.exists(docs_directory):
        logger.error(f"Documents directory does not exist: {docs_directory}")
        sys.exit(1)

    ingest_documents(docs_directory)