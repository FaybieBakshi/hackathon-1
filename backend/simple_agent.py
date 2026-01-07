"""
Simple RAG Agent Module
Integrates retrieved book chunks to generate accurate, cited answers.
"""
import os
import json
import logging
import time
from typing import List, Dict, Any, Optional
from dataclasses import dataclass
from datetime import datetime

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__)))
from src.utils.config import load_config
from src.utils.logger import setup_logger

# Import retrieval with error handling to support both modes
try:
    from src.retrieval.retriever import retrieve_chunks
    from src.retrieval.data import RetrievedChunk
    RETRIEVAL_AVAILABLE = True
except Exception as e:
    RETRIEVAL_AVAILABLE = False
    RetrievedChunk = None
    print(f"Retrieval system not available: {e}")


@dataclass
class AgentResponse:
    """
    Response from the RAG agent.
    """
    answer: str
    citations: Dict[str, Any]
    confidence_score: float
    token_usage: Dict[str, int]
    fallback_used: bool
    status: str


class SimpleRAGAgent:
    """
    Simple RAG Agent that integrates retrieval and generation without complex agent SDKs.
    """
    def __init__(
        self,
        max_context_tokens: int = 128000,
        min_confidence_threshold: float = 0.3,
        max_tokens: int = 1000,
        temperature: float = 0.3
    ):
        """
        Initialize the Simple RAG Agent.

        Args:
            max_context_tokens: Maximum tokens for the context window
            min_confidence_threshold: Minimum confidence for proceeding with generation
            max_tokens: Maximum tokens for the response
            temperature: Temperature for response generation
        """
        self.max_context_tokens = max_context_tokens
        self.min_confidence_threshold = min_confidence_threshold
        self.max_tokens = max_tokens
        self.temperature = temperature

        # Set up logging
        self.logger = setup_logger()

    def _get_simple_response(self, query: str) -> str:
        """
        Generate a simple response based on keyword matching when retrieval is not available.
        """
        query_lower = query.lower()

        # Define simple keyword-based responses
        if any(keyword in query_lower for keyword in ["hello", "hi", "hey", "greetings"]):
            return "Hello! I'm your AI assistant for Physical AI and Robotics. How can I help you today?"
        elif any(keyword in query_lower for keyword in ["physical ai", "physical artificial intelligence"]):
            return "Physical AI refers to the field that combines artificial intelligence with physical systems, particularly robotics. It focuses on creating AI systems that can understand and interact with the physical world through robots and other embodied agents."
        elif any(keyword in query_lower for keyword in ["robot", "robotics", "humanoid"]):
            return "Robotics is an interdisciplinary field that includes computer science, electrical engineering, and mechanical engineering. It focuses on the design, construction, operation, and use of robots. Humanoid robots are specifically designed to resemble the human body in shape."
        elif any(keyword in query_lower for keyword in ["chat", "help", "assist"]):
            return "I'm here to help you with questions about Physical AI, Robotics, and related topics. Please ask me anything about these subjects!"
        elif any(keyword in query_lower for keyword in ["what", "how", "explain"]):
            return f"I received your query: '{query}'. I can help you with information about Physical AI, Robotics, and related topics. For more specific answers, please ask detailed questions about these subjects."
        else:
            return f"I received your query: '{query}'. I'm an AI assistant focused on Physical AI and Robotics. For the best results, please ask specific questions about these topics."

    def query(
        self,
        query: str,
        conversation_history: List[Dict[str, str]] = None,
        max_tokens: int = None,
        temperature: float = None
    ) -> AgentResponse:
        """
        Process a query through the simple RAG agent.

        Args:
            query: The user's query
            conversation_history: Previous Q&A pairs in the session
            max_tokens: Maximum tokens for the response (defaults to instance setting)
            temperature: Temperature for response generation (defaults to instance setting)

        Returns:
            AgentResponse with the answer and metadata
        """
        start_time = time.time()
        max_tokens = max_tokens or self.max_tokens
        temperature = temperature or self.temperature

        try:
            self.logger.info(f"Processing query: {query}")

            if not RETRIEVAL_AVAILABLE:
                # If retrieval is not available due to API issues, use a simple keyword-based response
                answer = self._get_simple_response(query)
                return AgentResponse(
                    answer=answer,
                    citations={},
                    confidence_score=0.5,  # Moderate confidence for simple keyword matching
                    token_usage={"prompt_tokens": 0, "completion_tokens": 0, "total_tokens": 0},
                    fallback_used=True,
                    status="fallback"
                )

            # Retrieve relevant chunks from the book corpus
            retrieval_result = retrieve_chunks(
                query,
                top_k=5,
                min_score=self.min_confidence_threshold
            )

            if not retrieval_result.chunks:
                # No relevant chunks found, return fallback response
                answer = f"Sorry, I couldn't find any relevant information about '{query}' in the documentation. You might try rephrasing your question or checking other sections of the book."
                return AgentResponse(
                    answer=answer,
                    citations={},
                    confidence_score=0.0,
                    token_usage={"prompt_tokens": 0, "completion_tokens": 0, "total_tokens": 0},
                    fallback_used=True,
                    status="no_results"
                )

            # Format the context for the response
            context_parts = ["Here is the relevant information from the book:"]
            citations = {}

            for i, chunk in enumerate(retrieval_result.chunks):
                citation_marker = f"[{i+1}]"
                context_parts.append(f"{citation_marker} {chunk.content}")
                citations[citation_marker] = chunk

            context = "\n\n".join(context_parts)

            # Generate a response based on the context
            # In a real implementation, this would call an LLM, but for now we'll create a simple response
            answer_parts = [
                f"Based on the book content, here's what I found about '{query}':",
                "",
                context
            ]

            answer = "\n".join(answer_parts)

            # Calculate confidence based on the highest score of retrieved chunks
            confidence_score = max(chunk.score for chunk in retrieval_result.chunks) if retrieval_result.chunks else 0.0
            fallback_used = confidence_score < self.min_confidence_threshold
            status = "success" if not fallback_used else "low_confidence"

            execution_time = time.time() - start_time

            self.logger.info(f"Generated answer in {execution_time:.2f}s with {len(citations)} citations")

            return AgentResponse(
                answer=answer,
                citations=citations,
                confidence_score=confidence_score,
                token_usage={"prompt_tokens": 0, "completion_tokens": 0, "total_tokens": 0},  # Placeholder
                fallback_used=fallback_used,
                status=status
            )

        except Exception as e:
            self.logger.error(f"Error in SimpleRAGAgent.query: {str(e)}", exc_info=True)
            execution_time = time.time() - start_time
            return AgentResponse(
                answer="I encountered an error while processing your request.",
                citations={},
                confidence_score=0.0,
                token_usage={"prompt_tokens": 0, "completion_tokens": 0, "total_tokens": 0},
                fallback_used=True,
                status="error"
            )


def query_agent(
    query: str,
    conversation_history: List[Dict[str, str]] = None,
    max_tokens: int = 1000,
    temperature: float = 0.3,
    min_confidence_threshold: float = 0.3
) -> AgentResponse:
    """
    Query the simple RAG agent with a given query and optional conversation history.

    Args:
        query: The user's query
        conversation_history: Previous Q&A pairs in the session
        max_tokens: Maximum tokens for the response
        temperature: Temperature for response generation
        min_confidence_threshold: Minimum confidence for proceeding

    Returns:
        AgentResponse with the answer and metadata
    """
    agent = SimpleRAGAgent(
        max_tokens=max_tokens,
        temperature=temperature,
        min_confidence_threshold=min_confidence_threshold
    )

    return agent.query(query, conversation_history, max_tokens, temperature)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Simple RAG Agent CLI")
    parser.add_argument("--query", type=str, required=True, help="Query to ask the agent")
    parser.add_argument("--max-tokens", type=int, default=1000, help="Maximum tokens for response")
    parser.add_argument("--temperature", type=float, default=0.3, help="Temperature for response generation")
    parser.add_argument("--min-confidence", type=float, default=0.3, help="Minimum confidence threshold")
    parser.add_argument("--top-k", type=int, default=5, help="Number of top results to retrieve")

    args = parser.parse_args()

    # Create agent with specified parameters
    agent = SimpleRAGAgent(
        max_tokens=args.max_tokens,
        temperature=args.temperature,
        min_confidence_threshold=args.min_confidence
    )

    result = agent.query(
        query=args.query,
        max_tokens=args.max_tokens,
        temperature=args.temperature
    )

    print(f"Query: {args.query}")
    print(f"Status: {result.status}")
    print(f"Fallback used: {result.fallback_used}")
    print(f"Confidence: {result.confidence_score:.3f}")
    print(f"Tokens used: {result.token_usage['total_tokens']}")
    print(f"Answer: {result.answer}\n")

    if result.citations:
        print("Citations:")
        for citation, chunk in result.citations.items():
            print(f"  {citation}: {chunk.content[:100]}...")
    else:
        print("No citations generated.")