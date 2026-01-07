import asyncio
import json
import time
import html
import re
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import StreamingResponse
from typing import Optional
from sse_starlette.sse import EventSourceResponse
from .middleware.cors import setup_cors
from .middleware.rate_limit import rate_limit_middleware
from .logging_config import setup_logging
from .agents.rag_agent import RAGAgent
from .agents.base_agent import QueryResult


def sanitize_input(input_text: str) -> str:
    """
    Sanitize user input to prevent injection attacks
    """
    if not input_text:
        return input_text

    # Remove potentially dangerous characters/sequences
    # Remove script tags and their content
    input_text = re.sub(r'<script[^>]*>.*?</script>', '', input_text, flags=re.IGNORECASE | re.DOTALL)

    # Remove javascript: and vbscript: protocols
    input_text = re.sub(r'javascript:', '', input_text, flags=re.IGNORECASE)
    input_text = re.sub(r'vbscript:', '', input_text, flags=re.IGNORECASE)

    # Remove event handlers
    input_text = re.sub(r'on\w+\s*=', '', input_text, flags=re.IGNORECASE)

    # Escape HTML characters
    input_text = html.escape(input_text)

    # Limit length to prevent overly large inputs
    max_length = 2000
    if len(input_text) > max_length:
        input_text = input_text[:max_length]

    return input_text.strip()


# Initialize logging
setup_logging()

app = FastAPI(
    title="RAG Chatbot API",
    description="API for RAG-based chatbot with documentation querying capabilities",
    version="1.0.0"
)

# Setup CORS middleware
setup_cors(app)

# Setup rate limiting middleware
rate_limit_middleware(app)

@app.get("/")
async def root():
    return {"message": "RAG Chatbot API is running"}

@app.post("/chat")
async def chat(query: str, session_id: str = None):
    """Chat endpoint that accepts text queries and returns relevant answers with citations"""
    if not query or not query.strip():
        raise HTTPException(status_code=400, detail="Query cannot be empty")

    # Sanitize input to prevent injection attacks
    sanitized_query = sanitize_input(query)

    try:
        rag_agent = RAGAgent()
        result = await rag_agent.query(sanitized_query, session_id)
        return result
    except ValueError as e:
        # Handle validation errors specifically
        raise HTTPException(status_code=400, detail=f"Invalid query: {str(e)}")
    except Exception as e:
        # Log the error for debugging
        import logging
        logging.error(f"Error in /chat endpoint: {str(e)}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")

@app.post("/chat_with_selection")
async def chat_with_selection(query: str, selected_text: str, session_id: str = None):
    """Chat endpoint that accepts a text query and selected text, returning context-aware answers"""
    if not query or not query.strip():
        raise HTTPException(status_code=400, detail="Query cannot be empty")

    # Sanitize inputs to prevent injection attacks
    sanitized_query = sanitize_input(query)
    sanitized_selected_text = sanitize_input(selected_text)

    try:
        rag_agent = RAGAgent()
        result = await rag_agent.query_with_selection(sanitized_query, sanitized_selected_text, session_id)
        return result
    except ValueError as e:
        # Handle validation errors specifically
        raise HTTPException(status_code=400, detail=f"Invalid query or selection: {str(e)}")
    except Exception as e:
        # Log the error for debugging
        import logging
        logging.error(f"Error in /chat_with_selection endpoint: {str(e)}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")

@app.get("/chat/stream")
async def chat_stream(query: str, session_id: str = None):
    """Streaming chat endpoint that returns responses via Server-Sent Events"""
    if not query or not query.strip():
        raise HTTPException(status_code=400, detail="Query cannot be empty")

    # Sanitize input to prevent injection attacks
    sanitized_query = sanitize_input(query)

    async def event_generator():
        try:
            rag_agent = RAGAgent()
            async for chunk in rag_agent.stream_query(sanitized_query, session_id):
                yield {"event": "message", "data": chunk}
        except ValueError as e:
            yield {"event": "error", "data": f"Invalid query: {str(e)}"}
        except Exception as e:
            # Log the error for debugging
            import logging
            logging.error(f"Error in /chat/stream endpoint: {str(e)}", exc_info=True)
            yield {"event": "error", "data": f"Error processing query: {str(e)}"}

    return EventSourceResponse(event_generator())

@app.get("/chat_with_selection/stream")
async def chat_with_selection_stream(query: str, selected_text: str, session_id: str = None):
    """Streaming chat endpoint that accepts query and selection, returning context-aware responses via SSE"""
    if not query or not query.strip():
        raise HTTPException(status_code=400, detail="Query cannot be empty")

    # Sanitize inputs to prevent injection attacks
    sanitized_query = sanitize_input(query)
    sanitized_selected_text = sanitize_input(selected_text)

    async def event_generator():
        try:
            rag_agent = RAGAgent()
            async for chunk in rag_agent.stream_query_with_selection(sanitized_query, sanitized_selected_text, session_id):
                yield {"event": "message", "data": chunk}
        except ValueError as e:
            yield {"event": "error", "data": f"Invalid query or selection: {str(e)}"}
        except Exception as e:
            # Log the error for debugging
            import logging
            logging.error(f"Error in /chat_with_selection/stream endpoint: {str(e)}", exc_info=True)
            yield {"event": "error", "data": f"Error processing query: {str(e)}"}

    return EventSourceResponse(event_generator())

@app.get("/health")
async def health_check():
    """Health endpoint that returns system status information"""
    # In a real implementation, this would check database connections,
    # external service availability, etc.
    # For now, we'll return a basic health status

    # Simulate checking various system components
    database_status = "healthy"  # Would check actual DB connection in real impl
    external_apis_status = "healthy"  # Would check actual external services in real impl
    memory_usage = "normal"  # Would check actual memory in real impl
    uptime = "running"  # Would calculate actual uptime in real impl

    return {
        "status": "healthy",
        "service": "RAG Chatbot API",
        "version": "1.0.0",
        "timestamp": time.time(),
        "details": {
            "database": database_status,
            "external_apis": external_apis_status,
            "memory_usage": memory_usage,
            "uptime": uptime
        }
    }