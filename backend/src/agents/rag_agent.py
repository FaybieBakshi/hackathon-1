import asyncio
import logging
import time
from typing import List, Dict, Any
from .base_agent import BaseRAGAgent, QueryResult
from ..config import get_settings
from ..session_manager import session_manager, ChatSession
import sys
import os
# Add the backend directory to the Python path to allow imports from the main agent module
backend_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, backend_dir)

from agent import RAGAgent as OpenAIRAGAgent

# Remove the added path to restore original state
sys.path.remove(backend_dir)


logger = logging.getLogger(__name__)


class RAGAgent(BaseRAGAgent):
    """Implementation of the RAG agent for documentation querying using OpenAI Agent SDK"""

    def __init__(self, model_name: str = "gpt-3.5-turbo", max_context_tokens: int = 128000, min_confidence_threshold: float = 0.3, max_tokens: int = 1000, temperature: float = 0.3):
        self.settings = get_settings()
        # Use the OpenAI Agent SDK implementation
        self.openai_agent = OpenAIRAGAgent(
            model_name=model_name,
            max_context_tokens=max_context_tokens,
            min_confidence_threshold=min_confidence_threshold,
            max_tokens=max_tokens,
            temperature=temperature
        )
        # Initialize any required models or connections here
        logger.info(f"Initializing OpenAI Agent SDK RAG Agent with model: {model_name}")

    async def query(self, query_text: str, session_id: str = None) -> QueryResult:
        """Process a text query and return a result with citations using OpenAI Agent SDK"""
        if session_id:
            # Add user query to session history
            await session_manager.add_message_to_session(session_id, {
                "type": "user",
                "content": query_text,
                "timestamp": time.time()
            })

        logger.info(f"Processing query with OpenAI Agent SDK: {query_text[:50]}...")

        try:
            # Use the OpenAI Agent SDK implementation
            result = await self.openai_agent.query(query_text)

            answer = result.answer
            citations = result.citations
            confidence_score = result.confidence_score
            status = result.status

            # Create sources list from citations
            sources = []
            if citations:
                for citation in citations.values():
                    if hasattr(citation, 'metadata') and citation.metadata.get("url"):
                        source_url = citation.metadata.get("url")
                        if source_url and source_url not in sources:
                            sources.append(source_url)

        except Exception as e:
            logger.error(f"Error during OpenAI Agent query: {str(e)}")
            answer = f"Sorry, I encountered an error processing your query '{query_text}'. You might try rephrasing your question."
            citations = []
            sources = []
            confidence_score = 0.0
            status = "error"

        query_result = QueryResult(
            answer=answer,
            citations=citations,
            sources=sources,
            query=query_text,
            timestamp=time.time()
        )

        if session_id:
            # Add assistant response to session history
            await session_manager.add_message_to_session(session_id, {
                "type": "assistant",
                "content": answer,
                "citations": citations,
                "timestamp": time.time()
            })

        logger.info(f"Query processed with OpenAI Agent SDK: {query_text[:30]}...")
        return query_result

    async def query_with_selection(self, query_text: str, selected_text: str, session_id: str = None) -> QueryResult:
        """Process a query with selected text context and return a result with citations using OpenAI Agent SDK"""
        if session_id:
            # Add user query to session history
            await session_manager.add_message_to_session(session_id, {
                "type": "user",
                "content": f"Query: {query_text}, Selected: {selected_text}",
                "timestamp": time.time()
            })

        logger.info(f"Processing query with selection using OpenAI Agent SDK: {query_text[:30]}... with selection: {selected_text[:30]}...")

        # Combine query and selected text for better retrieval
        combined_query = f"{query_text} related to {selected_text}"

        try:
            # Use the OpenAI Agent SDK implementation
            result = await self.openai_agent.query(combined_query)

            answer = result.answer
            citations = result.citations
            confidence_score = result.confidence_score
            status = result.status

            # Create sources list from citations
            sources = []
            if citations:
                for citation in citations.values():
                    if hasattr(citation, 'metadata') and citation.metadata.get("url"):
                        source_url = citation.metadata.get("url")
                        if source_url and source_url not in sources:
                            sources.append(source_url)

        except Exception as e:
            logger.error(f"Error during OpenAI Agent query with selection: {str(e)}")
            answer = f"Sorry, I encountered an error processing your query '{combined_query}'. You might try rephrasing your question."
            citations = []
            sources = []
            confidence_score = 0.0
            status = "error"

        query_result = QueryResult(
            answer=answer,
            citations=citations,
            sources=sources,
            query=query_text,
            timestamp=time.time()
        )

        if session_id:
            # Add assistant response to session history
            await session_manager.add_message_to_session(session_id, {
                "type": "assistant",
                "content": answer,
                "citations": citations,
                "timestamp": time.time()
            })

        logger.info(f"Query with selection processed using OpenAI Agent SDK")
        return query_result

    async def stream_query(self, query_text: str, session_id: str = None):
        """Stream query response as SSE events using OpenAI Agent SDK"""
        if session_id:
            # Add user query to session history
            await session_manager.add_message_to_session(session_id, {
                "type": "user",
                "content": query_text,
                "timestamp": time.time()
            })

        logger.info(f"Streaming query with OpenAI Agent SDK: {query_text[:50]}...")

        try:
            # Use the OpenAI Agent SDK implementation for the full response
            result = await self.openai_agent.query(query_text)
            full_response = result.answer

            # Yield the full response as a single part for now
            # In a full streaming implementation, this would be broken down
            yield full_response

        except Exception as e:
            logger.error(f"Error during OpenAI Agent streaming query: {str(e)}")
            error_message = f"Sorry, I encountered an error processing your query '{query_text}'. You might try rephrasing your question."
            yield error_message

        if session_id:
            # Add assistant response to session history
            await session_manager.add_message_to_session(session_id, {
                "type": "assistant",
                "content": full_response if 'full_response' in locals() else "Error occurred",
                "timestamp": time.time()
            })

    async def stream_query_with_selection(self, query_text: str, selected_text: str, session_id: str = None):
        """Stream query response with selection context as SSE events using OpenAI Agent SDK"""
        if session_id:
            # Add user query to session history
            await session_manager.add_message_to_session(session_id, {
                "type": "user",
                "content": f"Query: {query_text}, Selected: {selected_text}",
                "timestamp": time.time()
            })

        logger.info(f"Streaming query with selection using OpenAI Agent SDK: {query_text[:30]}... with selection: {selected_text[:30]}...")

        # Combine query and selected text for better retrieval
        combined_query = f"{query_text} related to {selected_text}"

        try:
            # Use the OpenAI Agent SDK implementation for the full response
            result = await self.openai_agent.query(combined_query)
            full_response = result.answer

            # Yield the full response as a single part for now
            # In a full streaming implementation, this would be broken down
            yield full_response

        except Exception as e:
            logger.error(f"Error during OpenAI Agent streaming query with selection: {str(e)}")
            error_message = f"Sorry, I encountered an error processing your query '{combined_query}'. You might try rephrasing your question."
            yield error_message

        if session_id:
            # Add assistant response to session history
            await session_manager.add_message_to_session(session_id, {
                "type": "assistant",
                "content": full_response if 'full_response' in locals() else "Error occurred",
                "timestamp": time.time()
            })