# RAG Chatbot Integration

This project implements a RAG (Retrieval-Augmented Generation) chatbot API with a React frontend widget for integration with Docusaurus documentation sites.

## Features

- **FastAPI Backend**: Provides REST API endpoints with Server-Sent Events (SSE) streaming
- **React Chat Widget**: Embeddable component for Docusaurus sites
- **Real-time Streaming**: Answers stream in real-time using SSE
- **Session Management**: Maintains conversation context indefinitely
- **Citation Support**: Provides citations to documentation sources
- **Rate Limiting**: Prevents abuse with configurable limits
- **Security**: Input sanitization to prevent injection attacks
- **CORS Support**: Configured for Docusaurus frontend domains

## API Endpoints

- `POST /chat` - Process text queries
- `POST /chat_with_selection` - Process queries with selected text context
- `GET /chat/stream?query={query}&session_id={id}` - Stream responses via SSE
- `GET /chat_with_selection/stream?query={query}&selected_text={text}&session_id={id}` - Stream responses with selection context
- `GET /health` - Health check endpoint

## Frontend Integration

The React chat widget can be integrated into Docusaurus sites by importing the component and adding it to the desired layout.

## Project Structure

```
backend/
├── src/
│   ├── api.py              # Main FastAPI application
│   ├── config.py           # Configuration settings
│   ├── logging_config.py   # Logging setup
│   ├── session_manager.py  # Session management
│   └── agents/
│       ├── base_agent.py   # Base RAG agent interface
│       └── rag_agent.py    # RAG agent implementation
│   └── middleware/
│       ├── cors.py         # CORS configuration
│       └── rate_limit.py   # Rate limiting middleware
frontend-ai-book/
└── src/
    └── components/
        └── ChatWidget/
            └── ChatWidget.jsx  # React chat widget component
```

## Setup

1. Install backend dependencies:
   ```bash
   cd backend
   pip install -r requirements.txt
   ```

2. Start the backend server:
   ```bash
   cd [project-root-directory]
   python api.py
   ```
   Or alternatively:
   ```bash
   cd [project-root-directory]
   uvicorn api:app --reload --port 8002
   ```

3. In a separate terminal, start the frontend:
   ```bash
   cd [project-root-directory]/frontend-ai-book
   npm install
   npm start
   ```

## Configuration

The application uses environment variables for configuration. Create a `.env` file in the `backend/` directory:

```
ENVIRONMENT=development
PRODUCTION_ORIGIN=https://your-production-domain.com
LOG_LEVEL=INFO
```

## Security

- Input sanitization is performed on all user inputs
- Rate limiting prevents abuse
- CORS is configured to allow only specified domains
- All API endpoints have proper error handling

## Session Management

The application maintains conversation context indefinitely as specified in the requirements. Session IDs are stored in the browser's localStorage.

## Development

The project follows a modular architecture that allows independent development of backend and frontend components.
