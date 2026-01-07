# Data Model: RAG Embedding Pipeline

## Core Entities

### TextChunk
- **id**: String (UUID) - Unique identifier for the chunk
- **content**: String - The actual text content of the chunk (500-800 tokens)
- **metadata**: Object - Contains:
  - **url**: String - Source URL of the content
  - **chapter**: String - Chapter name/identifier from source
  - **section**: String - Section name/identifier from source
  - **source_hash**: String - Hash of original content for incremental processing
  - **created_at**: DateTime - Timestamp when chunk was created
  - **updated_at**: DateTime - Timestamp when chunk was last updated
- **token_count**: Integer - Number of tokens in the content
- **vector_id**: String - ID in the vector database (after embedding)

### EmbeddingVector
- **chunk_id**: String - Reference to the TextChunk
- **vector**: Array[Float] - The embedding vector from Cohere
- **model**: String - Name of the embedding model used (e.g., "embed-english-v3.0")
- **dimension**: Integer - Dimension of the embedding vector
- **created_at**: DateTime - Timestamp when embedding was generated

### ProcessingJob
- **id**: String (UUID) - Unique identifier for the job
- **status**: Enum (pending, processing, completed, failed) - Current status
- **total_pages**: Integer - Total number of pages to process
- **processed_pages**: Integer - Number of pages processed
- **total_chunks**: Integer - Total number of chunks generated
- **stored_chunks**: Integer - Number of chunks successfully stored
- **start_time**: DateTime - When the job started
- **end_time**: DateTime - When the job completed/failed
- **error_log**: Array[Object] - List of errors encountered during processing

### SourcePage
- **url**: String - URL of the source page
- **hash**: String - Content hash for incremental processing
- **last_fetched**: DateTime - Last time content was fetched
- **status**: Enum (new, processed, changed, unchanged) - Processing status
- **metadata**: Object - Additional metadata from the page

## Relationships

- ProcessingJob has many TextChunk instances
- TextChunk has one EmbeddingVector
- SourcePage generates many TextChunk instances

## Validation Rules

1. **TextChunk Content**: Must be between 500-800 tokens
2. **TextChunk Metadata**: URL, chapter, and section fields are required
3. **EmbeddingVector**: Must have valid vector dimensions matching the model
4. **ProcessingJob**: Must have valid status and accurate counts
5. **SourcePage**: URL must be a valid Vercel URL from the book site

## State Transitions

### ProcessingJob States:
- pending → processing (when started)
- processing → completed (when all pages processed successfully)
- processing → failed (when errors occur)

### SourcePage States:
- new → processed (after first processing)
- processed → changed (when content hash changes)
- changed → processed (after reprocessing)
- processed → unchanged (when content hash matches)