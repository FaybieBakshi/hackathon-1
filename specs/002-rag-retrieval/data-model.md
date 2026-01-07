# Data Model: RAG Retrieval Pipeline

**Date**: 2026-01-01
**Feature**: RAG Retrieval Pipeline & Testing (002-rag-retrieval)

## Core Data Structures

### Query
```python
class Query:
    text: str                    # The input query text
    embedding: List[float]       # Generated embedding vector
    top_k: int = 5               # Number of results to return
    min_score: float = 0.0       # Minimum confidence score threshold
    filters: Dict[str, Any]      # Optional metadata filters
```

### RetrievedChunk
```python
class RetrievedChunk:
    content: str                 # The text content of the chunk
    score: float                 # Semantic similarity score (0.0-1.0)
    metadata: Dict[str, Any]     # Original metadata (source URL, etc.)
    id: str                      # Qdrant point ID
    token_count: int             # Number of tokens in the chunk
```

### RetrievalResult
```python
class RetrievalResult:
    query: str                   # Original query text
    chunks: List[RetrievedChunk] # List of retrieved chunks
    execution_time: float        # Time taken for retrieval (ms)
    total_candidates: int        # Total candidates considered
    confidence_threshold: float  # Applied confidence threshold
    status: str                  # "success", "partial", "no_results", "error"
```

### QuerySet
```python
class QuerySet:
    name: str                    # Name/description of the query set
    queries: List[Query]         # List of test queries
    category: str                # "factual", "conceptual", "keyword-based"
    expected_outcomes: List[Dict[str, Any]]  # Expected results for validation
```

### PerformanceMetrics
```python
class PerformanceMetrics:
    p50_latency: float          # 50th percentile response time (ms)
    p90_latency: float          # 90th percentile response time (ms)
    p95_latency: float          # 95th percentile response time (ms)
    p99_latency: float          # 99th percentile response time (ms)
    throughput: float           # Queries per second
    cache_hit_rate: float       # Percentage of cached results used
    error_rate: float           # Percentage of failed queries
    total_queries: int          # Total number of queries processed
```

## Qdrant Schema

### Collection Structure
- **Collection Name**: Same as used in Spec 1 (from config.QDRANT_COLLECTION_NAME)
- **Vector Size**: Same as document embeddings (from Cohere model)
- **Distance**: Cosine similarity
- **Payload Schema**:
  ```json
  {
    "content": "string",
    "metadata": {
      "url": "string",
      "...": "additional metadata"
    },
    "token_count": "integer"
  }
  ```

## Test Data Structure

### Sample Queries
- **Factual Queries**: Direct questions seeking specific information
- **Conceptual Queries**: Questions about concepts, relationships, or ideas
- **Keyword-based Queries**: Queries using specific terms without full context

### Validation Criteria
- **Accuracy**: >95% of queries return at least one semantically correct chunk
- **Relevance**: Retrieved chunks should be contextually relevant to the query
- **Performance**: Meet latency requirements (p95 < 500ms cold, < 200ms cached)