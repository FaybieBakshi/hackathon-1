# RAG Retrieval System Guide

## Overview

This guide explains how to use the RAG (Retrieval-Augmented Generation) retrieval system for querying the vector database and retrieving semantically relevant content chunks.

## Architecture

The retrieval system consists of:
- Query processing and embedding generation
- Vector database interaction with Qdrant
- Result filtering and ranking
- Performance benchmarking
- Comprehensive testing suite

## Usage

### Basic Retrieval

```python
from backend.retrieve import retrieve_chunks

# Retrieve top 5 most relevant chunks for a query
results = retrieve_chunks("your query here", top_k=5, min_score=0.3)
for chunk in results:
    print(f"Score: {chunk.score}, Content: {chunk.content[:100]}...")
```

### Advanced Retrieval

```python
from backend.retrieve import retrieve_chunks

# With custom parameters
results = retrieve_chunks(
    query="your query here",
    top_k=10,
    min_score=0.4,
    use_cache=False
)
```

### Performance Benchmarking

```python
from backend.benchmark import run_benchmark

# Run performance benchmark
metrics = run_benchmark(
    queries=["query1", "query2", "query3"],
    iterations=10
)
print(f"P95 latency: {metrics['cached_metrics']['p95_latency']:.2f}ms")
```

### Testing

```python
from backend.test_retrieval import run_comprehensive_tests

# Run all tests
results = run_comprehensive_tests()
print(f"Accuracy: {results['accuracy_rate']:.2%}")
```

## Configuration

The retrieval system uses the same configuration as the embedding pipeline:

- `COHERE_API_KEY`: API key for generating query embeddings
- `QDRANT_URL`: URL for the Qdrant vector database
- `QDRANT_API_KEY`: API key for Qdrant access
- `QDRANT_COLLECTION_NAME`: Name of the collection with book embeddings
- `RETRIEVAL_TOP_K`: Default number of results to return (default: 5)
- `RETRIEVAL_MIN_SCORE`: Minimum confidence score threshold (default: 0.3)

## Error Handling

The system handles various error conditions:

- Qdrant unavailability with appropriate status reporting
- Empty query results with "no_query" status
- Low-confidence matches with "low_confidence" status
- Duplicate content with filtering mechanisms
- Extremely long queries with truncation and warnings
- Qdrant service unavailability with "qdrant_unavailable" status

## Performance Targets

- Cold start p95 latency: < 500ms
- Cached p95 latency: < 200ms
- Accuracy: >95% of queries return relevant results

## CLI Interface

The system includes a CLI interface for direct testing:

```bash
python retrieve.py --query "your query here" --top-k 5 --min-score 0.3
```

## Edge Cases

The system handles various edge cases:

- Empty or whitespace-only queries
- Extremely long queries (>1000 characters)
- Queries with no matching results
- Low-confidence matches
- Qdrant service unavailability
- Malformed queries with special characters

## Validation

The system includes comprehensive validation:

- Performance metrics validation against targets
- Accuracy validation (>95% success rate)
- Edge case handling validation
- Confidence score validation