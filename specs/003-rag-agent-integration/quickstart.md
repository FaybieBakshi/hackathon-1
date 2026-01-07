# Quickstart Guide: RAG Chatbot API with Streaming

## Overview
This guide provides quick instructions for setting up and running the FastAPI backend with streaming capabilities that integrates with the existing RAG agent for the Docusaurus-based book platform.

## Prerequisites
- Python 3.11+
- Node.js 18+ (for frontend development)
- Access to Qdrant vector database
- OpenAI API key or compatible service
- Environment variables configured

## Setup Instructions

### 1. Backend Setup
```bash
# Navigate to project root
cd hackathon-1

# Create virtual environment
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate

# Install dependencies
pip install fastapi uvicorn python-dotenv openai qdrant-client sse-starlette

# Install existing backend dependencies
cd backend
pip install -r requirements.txt
```

### 2. Environment Configuration
Create `.env` file in the project root:
```env
OPENAI_API_KEY=your_openai_api_key
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_URL=your_qdrant_url
COHERE_API_KEY=your_cohere_api_key
```

### 3. Run the FastAPI Server
```bash
# From project root
uvicorn api:app --reload --port 8002
```

### 4. Frontend Setup
```bash
# In a new terminal
cd frontend-ai-book

# Install dependencies
npm install

# Start development server
npm run start
```

## API Endpoints

### Chat Endpoint (Non-Streaming)
- **URL**: `POST /chat`
- **Request**:
```json
{
  "message": "Your question here",
  "session_id": "unique_session_id",
  "options": {
    "top_k": 5,
    "temperature": 0.3
  }
}
```
- **Response**:
```json
{
  "answer": "Response from the agent",
  "citations": [],
  "confidence_score": 0.8,
  "session_id": "unique_session_id",
  "timestamp": "2026-01-03T05:00:00Z",
  "status": "success"
}
```

### Chat with Selection Endpoint (Non-Streaming)
- **URL**: `POST /chat_with_selection`
- **Request**:
```json
{
  "message": "Your question here",
  "selected_text": "Selected text for context",
  "session_id": "unique_session_id",
  "options": {
    "top_k": 5,
    "temperature": 0.3
  }
}
```

### Streaming Chat Endpoint
- **URL**: `POST /chat/stream`
- **Request**:
```json
{
  "message": "Your question here",
  "session_id": "unique_session_id",
  "options": {
    "top_k": 5,
    "temperature": 0.3
  }
}
```
- **Response**: Server-Sent Events (SSE) stream with events:
  - `message`: Partial response content
  - `citations`: Source citations
  - `done`: Stream completion
  - `error`: Error events

### Streaming Chat with Selection Endpoint
- **URL**: `POST /chat_with_selection/stream`
- **Request**:
```json
{
  "message": "Your question here",
  "selected_text": "Selected text for context",
  "session_id": "unique_session_id",
  "options": {
    "top_k": 5,
    "temperature": 0.3
  }
}
```

### Health Check
- **URL**: `GET /health`
- **Response**: `{"status": "healthy"}`

## Frontend Integration
The React chat component is located at `frontend-ai-book/src/components/ChatWidget/` and is integrated into the Docusaurus theme. It communicates with the backend API endpoints and supports both regular and streaming responses.

## Testing
```bash
# Test the streaming API directly
curl -X POST http://localhost:8002/chat/stream \
  -H "Content-Type: application/json" \
  -H "Accept: text/event-stream" \
  -d '{"message": "What is this book about?", "session_id": "test_session"}'
```

## Development
The FastAPI application in `api.py` integrates with the existing RAG agent in `backend/agent.py` to provide streaming responses based on book content with proper citations. The React chat component handles both regular and streaming responses for a smooth user experience.