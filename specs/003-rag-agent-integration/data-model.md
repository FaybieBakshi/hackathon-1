# Data Model: RAG Agent Integration with Streaming

## Primary Entities from Feature Spec

### Query
- **Description**: A user's question or request that requires information from the book corpus
- **Fields**:
  - `query_text` (string): The text input from the user
  - `conversation_history` (list of dict): Previous Q&A pairs in the current session
  - `session_id` (string): Unique identifier for the conversation session
  - `options` (dict): Additional options like top_k, temperature, etc.

### Retrieved Chunks
- **Description**: Text segments from the book corpus retrieved by the system
- **Fields**:
  - `content` (string): The actual text content of the chunk
  - `score` (float): Relevance score of the chunk to the query
  - `metadata` (dict): Additional information about the source
  - `id` (string): Unique identifier for the chunk

### Generated Answer
- **Description**: The agent's response to the user's query
- **Fields**:
  - `answer` (string): The text response from the agent
  - `citations` (dict): Mapping between citation markers and source chunks
  - `confidence_score` (float): Confidence level in the answer
  - `token_usage` (dict): Token consumption metrics
  - `fallback_used` (bool): Whether a fallback response was used
  - `status` (string): Status of the response (success, low_confidence, error)

### Conversation History
- **Description**: Sequence of previous queries and answers in the current session
- **Fields**:
  - `query` (string): Previous user query
  - `answer` (string): Previous agent response
  - `timestamp` (datetime): When the interaction occurred

### Citation Map
- **Description**: Mapping between inline citation markers and their corresponding source chunks
- **Fields**:
  - `citation_marker` (string): The marker used in the answer (e.g., "[1]")
  - `source_chunk` (RetrievedChunk): The corresponding source content

### Context Window
- **Description**: The portion of information provided to the LLM for generating responses
- **Fields**:
  - `retrieved_chunks` (list of RetrievedChunk): Chunks included in context
  - `conversation_history` (ConversationHistory): Previous interactions
  - `token_count` (int): Number of tokens in the context
  - `max_tokens` (int): Maximum allowed tokens

### Confidence Score
- **Description**: A measure of how relevant retrieved chunks are to the current query
- **Fields**:
  - `score` (float): The confidence value (0.0 to 1.0)
  - `threshold` (float): Minimum acceptable confidence level
  - `relevance_indicators` (list): Factors contributing to the score

## API Request/Response Models

### ChatRequest
- `message` (string): The user's query message
- `session_id` (string): Session identifier
- `options` (dict): Additional options like top_k, temperature, etc.

### ChatResponse
- `answer` (string): The agent's response
- `citations` (list of dict): Source citations for the answer
- `confidence_score` (float): Confidence level of the response
- `session_id` (string): Session identifier
- `timestamp` (datetime): Response timestamp
- `status` (string): Response status

### StreamingEvent
- `event` (string): Type of event ('message', 'citations', 'done', 'error')
- `data` (string): JSON-encoded payload for the event
- `id` (string): Event identifier for tracking
- `retry` (int): Reconnection time in milliseconds (optional)

### StreamChunk
- `content` (string): Partial content of the response
- `is_final` (bool): Whether this is the final chunk
- `citations` (list): Citations associated with the response
- `confidence_score` (float): Confidence score of the response
- `status` (string): Current status of the stream