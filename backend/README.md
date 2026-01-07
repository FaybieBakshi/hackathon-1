# RAG Embedding Pipeline Backend

Backend service for the RAG (Retrieval-Augmented Generation) system that processes book content, generates embeddings, and stores them in Qdrant Cloud.

## Overview

This backend service handles the complete pipeline for RAG system:
1. Fetches content from book pages
2. Cleans and extracts meaningful text
3. Chunks text semantically with overlap
4. Generates embeddings using Cohere
5. Stores embeddings in Qdrant Cloud with metadata
6. Validates storage and retrievability

## Prerequisites

- Python 3.11 or higher
- pip package manager
- UV (optional but recommended)

## Setup

1. Install dependencies:
   ```bash
   pip install -r requirements.txt
   # Or using UV:
   uv pip install -r requirements.txt
   ```

2. Set up environment variables:
   ```bash
   cp .env.example .env
   # Edit .env with your actual API keys
   ```

## Usage

Run the complete pipeline:
```bash
python src/main.py
```

Process specific URLs:
```bash
python src/main.py --urls "https://your-book.vercel.app/chapter1" "https://your-book.vercel.app/chapter2"
```

Run incremental processing:
```bash
python src/main.py --incremental
```

## Configuration

The pipeline can be configured via environment variables in `.env`:

- `COHERE_API_KEY`: Your Cohere API key for embedding generation
- `QDRANT_URL`: Your Qdrant Cloud cluster URL
- `QDRANT_API_KEY`: Your Qdrant API key
- `CHUNK_SIZE_MIN`: Minimum chunk size in tokens (default: 500)
- `CHUNK_SIZE_MAX`: Maximum chunk size in tokens (default: 800)
- `CHUNK_OVERLAP`: Overlap between chunks in tokens (default: 100)
- `RATE_LIMIT_DELAY`: Delay between API calls in seconds (default: 1.0)
- `REQUEST_TIMEOUT`: Timeout for HTTP requests in seconds (default: 30)

## Schema & Data Model

### Text Chunk Schema
Each text chunk contains:
- `content`: The actual text content (500-800 tokens)
- `metadata`: Object containing:
  - `url`: Source URL of the content
  - `chapter`: Chapter name/identifier from source
  - `section`: Section name/identifier from source
  - `source_hash`: Hash of original content for incremental processing
- `token_count`: Number of tokens in the content
- `overlap_with_next`: Boolean indicating if there's a next chunk

### Qdrant Collection Schema
The system stores embeddings in a Qdrant collection with the following payload:
- `content`: The text content of the chunk
- `metadata`: Contains URL, chapter, section, and other metadata
- `token_count`: Number of tokens in the chunk

## Chunking Strategy

The system uses semantic chunking with the following parameters:
- **Chunk Size Range**: 500-800 tokens (configurable via environment variables)
- **Overlap**: 100 tokens (configurable) to maintain context between chunks
- **Tokenization**: Uses tiktoken with gpt-3.5-turbo encoding
- **Overlap Logic**: Each chunk overlaps with the next by the specified number of tokens

This approach ensures that:
- Chunks contain meaningful semantic units
- Context is preserved across chunk boundaries
- Retrieval can access relevant context beyond the immediate chunk

## Embedding Settings

The system uses Cohere's embed-english-v3.0 model with the following configuration:
- **Model**: embed-english-v3.0 (optimized for English text)
- **Input Type**: "search_document" for search-optimized embeddings
- **Vector Dimensions**: 1024 (for embed-english-v3.0 model)

## Architecture

The backend is organized into several modules:

- `src/ingestion/`: Handles URL fetching, text cleaning, and chunking
- `src/embedding/`: Generates embeddings using Cohere
- `src/storage/`: Stores embeddings in Qdrant Cloud
- `src/utils/`: Configuration and logging utilities
- `src/validation/`: Validates storage and retrievability

## Testing

Run the test suite:
```bash
pytest tests/
```

Run specific tests:
```bash
pytest tests/test_ingestion.py  # Test ingestion components
pytest tests/test_chunking.py   # Test chunking logic
pytest tests/test_embedding.py  # Test embedding generation
pytest tests/test_storage.py    # Test storage operations
pytest tests/test_edge_cases.py # Test edge cases
```

## Features

- **Incremental Processing**: Only reprocesses changed/new pages using content hashing
- **Rate Limiting**: Handles API rate limits and server errors with exponential backoff
- **Error Handling**: Comprehensive error handling with retry logic for network issues
- **Validation**: Verifies >99% of chunks are correctly stored and retrievable
- **Logging**: Comprehensive logging for debugging and monitoring