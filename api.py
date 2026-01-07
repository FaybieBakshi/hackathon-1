"""
FastAPI backend for the RAG Chatbot with Streaming Support.

This file implements the API endpoints that connect the Docusaurus frontend
chat widget to the existing RAG agent in the backend with streaming capabilities.
"""
import os
import sys
import json
import logging
from datetime import datetime
from typing import Dict, List, Optional, Any, AsyncGenerator
from pydantic import BaseModel, Field
from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
from sse_starlette.sse import EventSourceResponse

# Add parent directory to path so 'backend' can be imported as a package
parent_dir = os.path.dirname(os.path.abspath(__file__))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

# Import required modules from backend and set up logger
try:
    from backend.src.utils.config import load_config
    from backend.src.utils.logger import setup_logger
    # Set up logger first (before any try/except blocks)
    logger = setup_logger(name="api")
except ImportError as e:
    # If backend.src modules aren't available, set up basic logging
    import logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    logger = logging.getLogger("api")
    logger.warning(f"Could not import backend.src modules: {e}")
    # Create minimal stubs
    def load_config():
        return {}

# Try to import the proper RAG agent functionality using OpenAI Agent SDK
try:
    from backend.agent import RAGAgent, query_agent as backend_query_agent
    from backend.src.retrieval.retriever import retrieve_chunks
    AGENT_AVAILABLE = True
    logger.info("OpenAI Agent SDK RAG Agent imported successfully")

    # Create a query_agent function that uses the OpenAI Agent SDK RAG agent
    async def query_agent(query, session_id=None, max_tokens=1000, temperature=0.3, min_confidence_threshold=0.3):
        """Async wrapper function to use the OpenAI Agent SDK RAG agent for compatibility."""

        from backend.agent import RAGAgent

        # Create agent instance and run async query directly
        agent = RAGAgent(
            max_tokens=max_tokens,
            temperature=temperature,
            min_confidence_threshold=min_confidence_threshold
        )

        # Run the async query directly (no nested event loop)
        result = await agent.query(query, max_tokens=max_tokens, temperature=temperature)

        # Convert to the expected format
        from types import SimpleNamespace
        response = SimpleNamespace()
        response.answer = result.answer
        response.citations = result.citations
        response.confidence_score = getattr(result, 'confidence_score', 0.5)  # Default confidence score
        response.status = getattr(result, 'status', "success" if result.citations else "no_results")

        return response

except ImportError as e:
    logger.error(f"Could not import proper RAG Agent: {e}", exc_info=True)
    logger.warning(f"Import error details: {type(e).__name__}: {str(e)}")
    logger.info("Falling back to simple agent")

    # Try to import simple agent as fallback
    try:
        from backend.simple_agent import query_agent as simple_query_agent, SimpleRAGAgent
        AGENT_AVAILABLE = True
        RAGAgent = SimpleRAGAgent
        logger.info("Simple RAG Agent imported as fallback")

        # Wrap the simple query_agent to make it async
        async def query_agent(query, session_id=None, **kwargs):
            return simple_query_agent(query=query, session_id=session_id, **kwargs)
    except ImportError as fallback_error:
        logger.error(f"Fallback also failed: {fallback_error}", exc_info=True)
        logger.warning(f"Fallback import error details: {type(fallback_error).__name__}: {str(fallback_error)}")
        AGENT_AVAILABLE = False
        # Define placeholder functions for testing
        async def query_agent(query, session_id=None, **kwargs):
            return type('MockResponse', (), {
                'answer': f"Mock response for query: {query}",
                'citations': [],
                'confidence_score': 0.5,
                'status': 'success'
            })()

        class SimpleRAGAgent:
            def __init__(self, **kwargs):
                pass

        RAGAgent = SimpleRAGAgent


class ChatRequest(BaseModel):
    """Request model for chat endpoint."""
    message: str = Field(..., description="The user's message/query")
    session_id: str = Field(..., description="Unique session identifier")
    options: Optional[Dict[str, Any]] = Field(
        default_factory=dict,
        description="Additional options like top_k, temperature, etc."
    )


class ChatWithSelectionRequest(BaseModel):
    """Request model for chat with selection endpoint."""
    message: str = Field(..., description="The user's message/query")
    selected_text: str = Field(..., description="Text selected by the user for context")
    session_id: str = Field(..., description="Unique session identifier")
    options: Optional[Dict[str, Any]] = Field(
        default_factory=dict,
        description="Additional options like top_k, temperature, etc."
    )


class Citation(BaseModel):
    """Model for citation information."""
    id: str
    title: str
    url: str
    content: str


class ChatResponse(BaseModel):
    """Response model for chat endpoint."""
    answer: str
    citations: List[Citation]
    confidence_score: float
    session_id: str
    timestamp: str
    status: str


class HealthResponse(BaseModel):
    """Response model for health check endpoint."""
    status: str


# Global agent instance
rag_agent = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Initialize the RAG agent when the app starts."""
    global rag_agent
    global config
    logger.info("Initializing RAG Agent...")
    try:
        if AGENT_AVAILABLE:
            rag_agent = RAGAgent()
            logger.info("RAG Agent initialized successfully")
        else:
            rag_agent = RAGAgent()
            logger.warning("RAG Agent not available, using mock agent")
    except Exception as e:
        logger.error(f"Failed to initialize RAG Agent: {e}", exc_info=True)
        logger.warning("RAG Agent not available, using mock agent")
        # Create a minimal mock agent
        class MockRAGAgent:
            def __init__(self, **kwargs):
                pass
        rag_agent = MockRAGAgent()
    yield
    # Cleanup if needed
    logger.info("Shutting down RAG Agent...")


# Create FastAPI app
app = FastAPI(
    title="RAG Chatbot API with Streaming",
    description="API for the RAG-based chatbot that answers questions from book content with streaming support",
    version="1.0.0",
    lifespan=lifespan
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Configuration
config = load_config()


@app.get("/health", response_model=HealthResponse)
async def health_check():
    """Health check endpoint."""
    return HealthResponse(status="healthy")


@app.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """Process a chat message using the RAG agent."""
    try:
        logger.info(f"Processing chat request for session: {request.session_id}")

        # Extract options with defaults
        options = request.options or {}
        top_k = options.get('top_k', 5)
        temperature = options.get('temperature', 0.3)
        min_confidence = options.get('min_score', 0.3)

        # Query the RAG agent using the async function
        response = await query_agent(
            query=request.message,
            session_id=request.session_id,
            max_tokens=1000,
            temperature=temperature,
            min_confidence_threshold=min_confidence
        )

        # Format citations
        citations = []
        if hasattr(response, 'citations') and response.citations:
            for citation_id, chunk in response.citations.items():
                citations.append(
                    Citation(
                        id=citation_id,
                        title="Book Chapter",
                        url="/docs/intro",  # Default URL, can be customized
                        content=chunk.content[:200] + "..." if len(chunk.content) > 200 else chunk.content
                    )
                )

        # Create response
        chat_response = ChatResponse(
            answer=response.answer,
            citations=citations,
            confidence_score=response.confidence_score,
            session_id=request.session_id,
            timestamp=datetime.utcnow().isoformat() + "Z",
            status=response.status
        )

        logger.info(f"Chat response generated successfully for session: {request.session_id}")
        return chat_response

    except Exception as e:
        logger.error(f"Error processing chat request: {str(e)}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Error processing chat request: {str(e)}")


@app.post("/chat_with_selection", response_model=ChatResponse)
async def chat_with_selection(request: ChatWithSelectionRequest):
    """Process a chat message with selected text context."""
    try:
        logger.info(f"Processing chat with selection for session: {request.session_id}")

        # For now, we'll combine the selected text with the query
        # In a full implementation, this would be handled by the agent
        combined_query = f"Based on this text: '{request.selected_text}', {request.message}"

        # Extract options with defaults
        options = request.options or {}
        top_k = options.get('top_k', 5)
        temperature = options.get('temperature', 0.3)
        min_confidence = options.get('min_score', 0.3)

        # Query the RAG agent using the async function
        response = await query_agent(
            query=combined_query,
            session_id=request.session_id,
            max_tokens=1000,
            temperature=temperature,
            min_confidence_threshold=min_confidence
        )

        # Format citations
        citations = []
        if hasattr(response, 'citations') and response.citations:
            for citation_id, chunk in response.citations.items():
                citations.append(
                    Citation(
                        id=citation_id,
                        title="Book Chapter",
                        url="/docs/intro",  # Default URL, can be customized
                        content=chunk.content[:200] + "..." if len(chunk.content) > 200 else chunk.content
                    )
                )

        # Create response
        chat_response = ChatResponse(
            answer=response.answer,
            citations=citations,
            confidence_score=response.confidence_score,
            session_id=request.session_id,
            timestamp=datetime.utcnow().isoformat() + "Z",
            status=response.status
        )

        logger.info(f"Chat with selection response generated successfully for session: {request.session_id}")
        return chat_response

    except Exception as e:
        logger.error(f"Error processing chat with selection request: {str(e)}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Error processing chat with selection request: {str(e)}")


@app.post("/chat/stream")
async def chat_stream(request: Request, chat_request: ChatRequest):
    """Stream chat responses using Server-Sent Events."""
    async def event_generator():
        try:
            logger.info(f"Processing streaming chat request for session: {chat_request.session_id}")

            # Extract options with defaults
            options = chat_request.options or {}
            top_k = options.get('top_k', 5)
            temperature = options.get('temperature', 0.3)
            min_confidence = options.get('min_score', 0.3)
            max_tokens = options.get('max_tokens', 1000)  # Add max_tokens definition

            # Skip primary agent entirely and use simple agent to avoid API quota issues
            logger.info("Using simple agent to avoid API quota issues")
            from backend.simple_agent import query_agent as simple_query_agent
            response = simple_query_agent(
                query=chat_request.message,
                max_tokens=max_tokens,
                temperature=temperature,
                min_confidence_threshold=min_confidence
            )

            # Send initial message event
            yield {
                "event": "message",
                "data": json.dumps({
                    "content": response.answer,
                    "is_final": True,
                    "confidence_score": response.confidence_score,
                    "status": response.status
                })
            }

            # Format and send citations if available
            if hasattr(response, 'citations') and response.citations:
                citations = []
                for citation_id, chunk in response.citations.items():
                    citations.append({
                        "id": citation_id,
                        "title": "Book Chapter",
                        "url": "/docs/intro",  # Default URL, can be customized
                        "content": chunk.content[:200] + "..." if len(chunk.content) > 200 else chunk.content
                    })

                yield {
                    "event": "citations",
                    "data": json.dumps(citations)
                }

            # Send completion event
            yield {
                "event": "done",
                "data": json.dumps({
                    "session_id": chat_request.session_id,
                    "timestamp": datetime.utcnow().isoformat() + "Z"
                })
            }

        except Exception as e:
            logger.error(f"Error in streaming chat: {str(e)}", exc_info=True)
            yield {
                "event": "error",
                "data": json.dumps({
                    "error": "Error processing streaming chat request",
                    "message": str(e)
                })
            }

    return EventSourceResponse(event_generator())


@app.post("/chat_with_selection/stream")
async def chat_with_selection_stream(request: Request, chat_request: ChatWithSelectionRequest):
    """Stream chat responses with selection using Server-Sent Events."""
    async def event_generator():
        try:
            logger.info(f"Processing streaming chat with selection for session: {chat_request.session_id}")

            # Combine the selected text with the query
            combined_query = f"Based on this text: '{chat_request.selected_text}', {chat_request.message}"

            # Extract options with defaults
            options = chat_request.options or {}
            top_k = options.get('top_k', 5)
            temperature = options.get('temperature', 0.3)
            min_confidence = options.get('min_score', 0.3)

            # Query the RAG agent using the async function
            response = await query_agent(
                query=combined_query,
                session_id=chat_request.session_id,
                max_tokens=1000,
                temperature=temperature,
                min_confidence_threshold=min_confidence
            )

            # Send initial message event
            yield {
                "event": "message",
                "data": json.dumps({
                    "content": response.answer,
                    "is_final": True,
                    "confidence_score": response.confidence_score,
                    "status": response.status
                })
            }

            # Format and send citations if available
            if hasattr(response, 'citations') and response.citations:
                citations = []
                for citation_id, chunk in response.citations.items():
                    citations.append({
                        "id": citation_id,
                        "title": "Book Chapter",
                        "url": "/docs/intro",  # Default URL, can be customized
                        "content": chunk.content[:200] + "..." if len(chunk.content) > 200 else chunk.content
                    })

                yield {
                    "event": "citations",
                    "data": json.dumps(citations)
                }

            # Send completion event
            yield {
                "event": "done",
                "data": json.dumps({
                    "session_id": chat_request.session_id,
                    "timestamp": datetime.utcnow().isoformat() + "Z"
                })
            }

        except Exception as e:
            logger.error(f"Error in streaming chat with selection: {str(e)}", exc_info=True)
            yield {
                "event": "error",
                "data": json.dumps({
                    "error": "Error processing streaming chat with selection request",
                    "message": str(e)
                })
            }

    return EventSourceResponse(event_generator())


@app.get("/")
async def root():
    """Root endpoint for basic information."""
    return {
        "message": "RAG Chatbot API with Streaming",
        "version": "1.0.0",
        "endpoints": [
            {"method": "GET", "path": "/health", "description": "Health check"},
            {"method": "POST", "path": "/chat", "description": "Chat with RAG agent (non-streaming)"},
            {"method": "POST", "path": "/chat/stream", "description": "Chat with RAG agent (streaming)"},
            {"method": "POST", "path": "/chat_with_selection", "description": "Chat with selected text context (non-streaming)"},
            {"method": "POST", "path": "/chat_with_selection/stream", "description": "Chat with selected text context (streaming)"}
        ]
    }


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "api:app",
        host="0.0.0.0",
        port=8002,  # Using port 8002 to avoid conflicts
        reload=True
    )