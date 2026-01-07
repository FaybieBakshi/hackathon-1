"""
RAG Agent Module
Integrates retrieved book chunks with OpenAI's GPT models to generate accurate, cited answers.
"""
import os
import json
import logging
import time
import sys
import asyncio
from typing import List, Dict, Any, Optional
from dotenv import load_dotenv
from openai import AsyncOpenAI
import tiktoken

# Load environment variables
load_dotenv()

# Add the backend directory to the Python path to allow imports from src
backend_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, backend_dir)

# Import from src
from src.utils.config import load_config
from src.utils.logger import setup_logger
from src.retrieval.retriever import retrieve_chunks
from src.retrieval.data import RetrievedChunk
from src.agents.data import AgentRequest, AgentResponse, ValidationResult
from src.agents.citation_manager import citation_manager
from src.agents.context_manager import ContextManager, context_manager
from src.utils.validators import validate_response_quality, validate_confidence_threshold
from src.utils.token_counter import count_tokens


def get_relevant_chunks(query: str, top_k: int = 5) -> List[Dict]:
    """
    Retrieve relevant chunks using the retrieval system.

    Args:
        query: The search query
        top_k: Number of top results to return

    Returns:
        List of chunks with metadata
    """
    logger = setup_logger()
    logger.info(f"Retrieving chunks for query: {query}")

    # Retrieve relevant chunks from the book corpus
    retrieval_result = retrieve_chunks(query, top_k=top_k)

    chunks_with_metadata = []
    for i, chunk in enumerate(retrieval_result.chunks):
        chunk_data = {
            "id": f"[{i+1}]",
            "content": chunk.content,
            "score": chunk.score,
            "metadata": chunk.metadata
        }
        chunks_with_metadata.append(chunk_data)

    return chunks_with_metadata


class RAGAgent:
    """
    Main RAG Agent class that integrates retrieval and generation with citation management.
    """
    def __init__(
        self,
        model_name: str = "gpt-3.5-turbo",
        max_context_tokens: int = 128000,
        min_confidence_threshold: float = 0.3,
        max_tokens: int = 1000,
        temperature: float = 0.3
    ):
        """
        Initialize the RAG Agent with OpenAI client.

        Args:
            model_name: OpenAI model to use for generation
            max_context_tokens: Maximum tokens for the context window
            min_confidence_threshold: Minimum confidence for proceeding with generation
            max_tokens: Maximum tokens for the response
            temperature: Temperature for response generation
        """
        self.model_name = model_name
        self.max_context_tokens = max_context_tokens
        self.min_confidence_threshold = min_confidence_threshold
        self.max_tokens = max_tokens
        self.temperature = temperature

        # Initialize OpenAI client with environment variable
        openai_api_key = os.getenv("OPENAI_API_KEY")
        if not openai_api_key:
            logger = setup_logger()
            logger.warning("OPENAI_API_KEY environment variable not set. Using OpenRouter API key.")
            # Fallback to the OpenRouter key if OpenAI key is not available
            openai_api_key = "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"

        base_url = os.getenv("OPENAI_BASE_URL")
        if base_url and "openrouter" in base_url:
            self.client = AsyncOpenAI(
                api_key=openai_api_key,
                base_url=base_url
            )
        else:
            self.client = AsyncOpenAI(
                api_key=openai_api_key
            )

        # Initialize components
        self.citation_manager = citation_manager
        self.context_manager = ContextManager(max_context_tokens)

        # Set up logging
        self.logger = setup_logger()

    async def generate_response(self, query: str, conversation_history: List[Dict] = None) -> Dict[str, Any]:
        """
        Generate a response using retrieved context and citations.

        Args:
            query: The user's query
            conversation_history: Previous conversation exchanges

        Returns:
            Dictionary with answer and source chunks
        """
        self.logger.info(f"Processing query: {query}")

        # Retrieve relevant chunks
        relevant_chunks = get_relevant_chunks(query, top_k=5)

        if not relevant_chunks:
            return {
                "answer": "I couldn't find any relevant information to answer your question.",
                "sources": []
            }

        # Format context with citations
        context_parts = []
        for chunk in relevant_chunks:
            context_parts.append(f"Source {chunk['id']}: {chunk['content']}")

        context = "\n\n".join(context_parts)

        # Prepare conversation history (last 3 exchanges)
        history_context = ""
        if conversation_history:
            recent_history = conversation_history[-3:]  # Last 3 exchanges
            for exchange in recent_history:
                role = exchange.get("role", "user")
                content = exchange.get("content", "")
                history_context += f"{role.capitalize()}: {content}\n"

        # Create system prompt that instructs to cite sources
        system_prompt = f"""You are an AI assistant that answers questions based only on the provided book content.
        Answer the question using only the information provided in the retrieved book content.
        When referencing specific information from the book content, use the citation format [1], [2], etc.
        Do not make up information that is not in the provided context.

        Retrieved content:
        {context}"""

        # Prepare messages for the API call
        messages = [
            {"role": "system", "content": system_prompt}
        ]

        # Add conversation history if available
        if history_context.strip():
            messages.append({"role": "system", "content": f"Recent conversation:\n{history_context}"})

        messages.append({"role": "user", "content": query})

        try:
            # Call the OpenAI API
            response = await self.client.chat.completions.create(
                model=self.model_name,
                messages=messages,
                max_tokens=self.max_tokens,
                temperature=self.temperature
            )

            answer = response.choices[0].message.content

            # Extract citations from the answer
            import re
            citation_pattern = r'\[(\d+)\]'
            found_citations = re.findall(citation_pattern, answer)

            # Map citations to source chunks
            sources = []
            for citation in found_citations:
                citation_id = f"[{citation}]"
                for chunk in relevant_chunks:
                    if chunk['id'] == citation_id:
                        sources.append({
                            "id": chunk['id'],
                            "content": chunk['content'],
                            "metadata": chunk['metadata']
                        })
                        break

            return {
                "answer": answer,
                "sources": sources
            }

        except Exception as e:
            self.logger.error(f"Error generating response: {str(e)}")
            # Check if it's an API quota/limit error and provide a more helpful response
            error_str = str(e).lower()
            if 'quota' in error_str or 'rate' in error_str or 'limit' in error_str or '429' in error_str:
                # For quota/rate limit issues, return a message indicating the database is unavailable
                return {
                    "answer": f"Hello! I received your query: '{query}'. However, I'm currently unable to access the AI model due to usage limits. The documentation database has information about this topic, but I can't retrieve it right now. Please try again later.",
                    "sources": []
                }
            else:
                return {
                    "answer": f"Sorry, I encountered an error processing your query: {str(e)}",
                    "sources": []
                }

    async def query(
        self,
        query: str,
        conversation_history: List[Dict[str, str]] = None,
        max_tokens: int = None,
        temperature: float = None
    ) -> AgentResponse:
        """
        Process a query through the RAG agent.

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

            # Generate response using the new method
            result = await self.generate_response(query, conversation_history)
            answer = result["answer"]
            sources = result["sources"]

            # Convert sources to citations format expected by AgentResponse
            citations = {}
            for source in sources:
                citations[source["id"]] = type('Chunk', (), {
                    'content': source["content"],
                    'metadata': source["metadata"]
                })()

            # Extract confidence based on source availability
            confidence_score = 0.8 if sources else 0.3
            fallback_used = len(sources) == 0
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
            error_msg = str(e)
            self.logger.error(f"Error in RAGAgent.query: {error_msg}", exc_info=True)

            # Check if it's an API key error and provide a more helpful message
            if "api_key" in error_msg.lower() or "OPENAI_API_KEY" in error_msg:
                answer = "OpenAI API key is not configured. Please set the OPENAI_API_KEY environment variable to use the RAG agent."
                status = "configuration_error"
            else:
                answer = "I encountered an error while processing your request."
                status = "error"

            execution_time = time.time() - start_time
            return AgentResponse(
                answer=answer,
                citations={},
                confidence_score=0.0,
                token_usage={"prompt_tokens": 0, "completion_tokens": 0, "total_tokens": 0},
                fallback_used=True,
                status=status
            )


def query_agent(
    query: str,
    conversation_history: List[Dict[str, str]] = None,
    max_tokens: int = 1000,
    temperature: float = 0.3,
    min_confidence_threshold: float = 0.3
) -> AgentResponse:
    """
    Query the RAG agent with a given query and optional conversation history.

    Args:
        query: The user's query
        conversation_history: Previous Q&A pairs in the session
        max_tokens: Maximum tokens for the response
        temperature: Temperature for response generation
        min_confidence_threshold: Minimum confidence for proceeding

    Returns:
        AgentResponse with the answer and metadata
    """
    agent = RAGAgent(
        max_tokens=max_tokens,
        temperature=temperature,
        min_confidence_threshold=min_confidence_threshold
    )

    # Run the async query in an event loop
    async def run_query():
        return await agent.query(query, conversation_history, max_tokens, temperature)

    # Run the async function
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        result = loop.run_until_complete(run_query())
        return result
    finally:
        loop.close()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="RAG Agent CLI")
    parser.add_argument("--query", type=str, required=True, help="Query to ask the agent")
    parser.add_argument("--max-tokens", type=int, default=1000, help="Maximum tokens for response")
    parser.add_argument("--temperature", type=float, default=0.3, help="Temperature for response generation")
    parser.add_argument("--min-confidence", type=float, default=0.3, help="Minimum confidence threshold")
    parser.add_argument("--model", type=str, default="gpt-3.5-turbo", help="OpenAI model to use")
    parser.add_argument("--top-k", type=int, default=5, help="Number of top results to retrieve")

    args = parser.parse_args()

    # Create agent with specified parameters
    agent = RAGAgent(
        model_name=args.model,
        max_tokens=args.max_tokens,
        temperature=args.temperature,
        min_confidence_threshold=args.min_confidence
    )

    async def run_main():
        result = await agent.query(
            query=args.query,
            max_tokens=args.max_tokens,
            temperature=args.temperature
        )

        print(f"Query: {args.query}")
        print(f"Model: {args.model}")
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

    # Run the async function
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(run_main())
    finally:
        loop.close()