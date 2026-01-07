# Quickstart: RAG Retrieval Pipeline

**Date**: 2026-01-01
**Feature**: RAG Retrieval Pipeline & Testing (002-rag-retrieval)

## Prerequisites

### Environment Setup
1. Ensure you have the same environment variables set as the embedding pipeline:
   ```bash
   export COHERE_API_KEY="your-cohere-api-key"
   export QDRANT_URL="your-qdrant-url"
   export QDRANT_API_KEY="your-qdrant-api-key"
   export QDRANT_COLLECTION_NAME="book_embeddings"  # Should match Spec 1
   ```

2. Make sure the Qdrant collection from Spec 1 has been populated with embeddings.

3. Install required dependencies:
   ```bash
   pip install -r requirements.txt
   # Or if using the virtual environment:
   cd backend && python -m pip install cohere qdrant-client python-dotenv
   ```

## Basic Usage

### 1. Run a Simple Retrieval
```bash
cd backend
python retrieve.py --query "your search query here" --top-k 5
```

### 2. Run the Test Suite
```bash
cd backend
python test_retrieval.py
```

### 3. Run Performance Benchmarks
```bash
cd backend
python benchmark.py
```

## Key Components

### retrieve.py
- Main entry point for retrieval functionality
- Contains `query_qdrant()` function to search the vector database
- Includes result filtering and top-k selection
- Handles edge cases like empty results and low-confidence matches

### test_retrieval.py
- Contains 20+ sample queries across different categories
- Validates that >95% of queries return relevant results
- Tests edge cases and error conditions

### benchmark.py
- Measures retrieval latency under different conditions
- Validates performance against targets (p95 < 500ms cold start, < 200ms cached)
- Generates performance reports

## Configuration

### Environment Variables
- `COHERE_API_KEY`: API key for generating query embeddings
- `QDRANT_URL`: URL for the Qdrant vector database
- `QDRANT_API_KEY`: API key for Qdrant access
- `QDRANT_COLLECTION_NAME`: Name of the collection with book embeddings (from Spec 1)
- `RETRIEVAL_TOP_K`: Default number of results to return (default: 5)
- `RETRIEVAL_MIN_SCORE`: Minimum confidence score threshold (default: 0.3)

### Default Values
- Top-k results: 5
- Minimum confidence score: 0.3
- Timeout for Qdrant queries: 30 seconds
- Batch size for query embedding: 32 (same as embedding pipeline)

## Example Code Usage

### Basic Retrieval
```python
from retrieve import retrieve_chunks

results = retrieve_chunks("your query here", top_k=5, min_score=0.3)
for chunk in results:
    print(f"Score: {chunk.score}, Content: {chunk.content[:100]}...")
```

### Advanced Retrieval with Filters
```python
from retrieve import retrieve_chunks

results = retrieve_chunks(
    "your query here",
    top_k=10,
    min_score=0.4,
    filters={"metadata.url": {"$contains": "specific-page"}}
)
```

## Validation Commands

### Run Complete Validation
```bash
# Run all tests and benchmarks
cd backend
python -m pytest tests/ -v
python test_retrieval.py --validate
python benchmark.py --validate
```

### Check Performance Targets
```bash
# Verify performance meets targets
python benchmark.py --targets
# Expected: p95 < 500ms (cold start), < 200ms (cached)
```

### Accuracy Validation
```bash
# Validate that >95% of queries return relevant results
python test_retrieval.py --accuracy
# Expected: >95% success rate
```