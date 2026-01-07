# RAG Agent Guide

## Overview

This guide explains how to use the RAG (Retrieval-Augmented Generation) agent for generating accurate, cited answers from book content using OpenAI's GPT models.

## Architecture

The RAG agent consists of:
- OpenAI integration for answer generation
- Citation management for inline citations
- Context window management for token efficiency
- Conversation history handling for follow-up questions
- Fallback response system for low-confidence scenarios

## Usage

### Basic Agent Usage

```python
from backend.agent import RAGAgent

# Initialize the agent
agent = RAGAgent()

# Simple query
response = agent.query("What is RAG?")
print(f"Answer: {response.answer}")
print(f"Citations: {response.citations}")
print(f"Confidence: {response.confidence_score}")
print(f"Status: {response.status}")
```

### With Conversation History

```python
from backend.agent import RAGAgent

agent = RAGAgent()

# First query
response1 = agent.query("What is RAG?")
print(f"Response: {response1.answer}")

# Follow-up query with conversation history
response2 = agent.query(
    "How does it improve language models?",
    conversation_history=[
        {"query": "What is RAG?", "answer": response1.answer}
    ]
)
print(f"Follow-up Response: {response2.answer}")
```

### Advanced Configuration

```python
from backend.agent import RAGAgent

agent = RAGAgent(
    model_name="gpt-4",
    temperature=0.1,
    max_tokens=1500,
    min_confidence_threshold=0.5
)

response = agent.query("Your query here")
```

## Configuration

The agent uses the following environment variables:

- `OPENAI_API_KEY`: API key for OpenAI access
- `COHERE_API_KEY`: API key for retrieval embeddings
- `QDRANT_URL`: URL for the Qdrant vector database
- `QDRANT_API_KEY`: API key for Qdrant access
- `QDRANT_COLLECTION_NAME`: Name of the collection with book embeddings
- `AGENT_MODEL`: OpenAI model to use (default: gpt-3.5-turbo)
- `MAX_TOKENS`: Maximum tokens for responses (default: 1000)
- `TEMPERATURE`: Temperature for response generation (default: 0.3)

## Features

### Citation Management
The agent automatically generates inline citations [1], [2] that map to source chunks from the book corpus.

### Context Window Management
The agent manages token limits efficiently, prioritizing the most relevant chunks within the context window.

### Conversation History
The agent handles follow-up questions by utilizing conversation history while maintaining a stateless design.

### Fallback Responses
When retrieval confidence is low, the agent provides appropriate fallback responses instead of generating potentially inaccurate answers.

### Comprehensive Testing
The system includes a QA test suite with 50+ examples to validate 90%+ correctness across factual, interpretive, and user-selected-text queries.

## Error Handling

The system handles various error conditions:

- OpenAI API errors with retry logic
- Low-confidence retrieval results with fallback responses
- Token limit exceeded with context window management
- Invalid input with proper validation

## Testing and Validation

Run the comprehensive test suite:

```bash
python backend/qa_test_suite.py
```

This will execute 50+ test cases across different categories and validate that the agent achieves 90%+ correctness.

## Performance Monitoring

The agent tracks performance metrics including:
- Response time
- Token usage
- Confidence scores
- Citation accuracy
- Fallback usage rates

These metrics help monitor the system's performance and identify areas for improvement.