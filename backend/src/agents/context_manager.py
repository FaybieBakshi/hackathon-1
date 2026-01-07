"""
Context window management for the RAG agent.

This module handles context window construction, token management,
conversation history management, and chunk prioritization.
"""
import time
from typing import List, Dict, Any, Optional
from dataclasses import dataclass

from src.utils.token_counter import TokenCounter, count_tokens, truncate_text
from src.retrieval.retriever import RetrievedChunk


@dataclass
class ConversationContext:
    """
    Represents the conversation context with history and metadata.
    """
    history: List[Dict[str, str]]  # List of {query: str, answer: str} pairs
    max_history_length: int = 10  # Maximum number of exchanges to retain
    context_window_tokens: int = 0  # Current token count in context
    last_access_time: float = None  # Time of last access for potential cleanup

    def __post_init__(self):
        if self.last_access_time is None:
            self.last_access_time = time.time()

    def add_exchange(self, query: str, answer: str):
        """Add a query-answer exchange to the history."""
        self.history.append({"query": query, "answer": answer})
        # Keep only the most recent exchanges
        if len(self.history) > self.max_history_length:
            self.history = self.history[-self.max_history_length:]
        self.last_access_time = time.time()

    def get_formatted_history(self) -> str:
        """Get the conversation history formatted for inclusion in context."""
        if not self.history:
            return ""

        formatted = "Previous conversation:\n"
        for i, exchange in enumerate(self.history[-3:], 1):  # Use last 3 exchanges
            formatted += f"Q{i}: {exchange['query']}\n"
            formatted += f"A{i}: {exchange['answer']}\n\n"

        return formatted


@dataclass
class ContextWindow:
    """
    Represents the context window with all components.
    """
    system_prompt: str = ""
    conversation_context: str = ""
    retrieved_context: str = ""
    query_context: str = ""
    total_tokens: int = 0
    available_tokens: int = 0
    chunk_tokens: Dict[str, int] = None

    def __post_init__(self):
        if self.chunk_tokens is None:
            self.chunk_tokens = {}


class ContextManager:
    """
    Manages context window construction and conversation history for the RAG agent.
    """
    def __init__(self, max_context_tokens: int = 128000):  # Default for GPT-4
        self.max_context_tokens = max_context_tokens
        self.token_counter = TokenCounter()
        self.min_confidence_threshold = 0.3

    def build_context_window(
        self,
        query: str,
        retrieved_chunks: List[RetrievedChunk],
        conversation_history: List[Dict[str, str]] = None,
        max_tokens: int = 4096
    ) -> ContextWindow:
        """
        Build a context window with conversation history, retrieved chunks, and query.

        Args:
            query: The current query
            retrieved_chunks: Retrieved chunks to include in context
            conversation_history: Previous Q&A pairs in the session
            max_tokens: Maximum tokens for the response

        Returns:
            ContextWindow object with all context components
        """
        # Prepare conversation context
        conversation_context = ""
        if conversation_history:
            conv_ctx = ConversationContext(history=conversation_history)
            conversation_context = conv_ctx.get_formatted_history()

        # Filter and prioritize chunks by relevance score
        prioritized_chunks = self.prioritize_chunks(retrieved_chunks)

        # Build retrieved context with token management
        retrieved_context, chunk_tokens = self.build_retrieved_context(
            prioritized_chunks, max_tokens
        )

        # Construct system prompt
        system_prompt = (
            "You are an AI assistant that answers questions based only on the provided book content.\n"
            "Answer the question using only the information provided in the relevant book content.\n"
            "If the book content doesn't contain sufficient information to answer the question, say so.\n"
            "When referencing specific information from the book content, use the citation format [1], [2], etc.\n\n"
        )

        # Combine all context components
        full_context = f"{system_prompt}\n"
        if conversation_context:
            full_context += f"{conversation_context}\n"
        full_context += f"{retrieved_context}\n"
        full_context += f"Question: {query}\n"
        full_context += "Answer: "

        # Calculate token counts
        total_tokens = self.token_counter.count_tokens(full_context)
        available_tokens = max_tokens - total_tokens

        return ContextWindow(
            system_prompt=system_prompt,
            conversation_context=conversation_context,
            retrieved_context=retrieved_context,
            query_context=f"Question: {query}\nAnswer: ",
            total_tokens=total_tokens,
            available_tokens=max(0, available_tokens),
            chunk_tokens=chunk_tokens
        )

    def prioritize_chunks(self, chunks: List[RetrievedChunk]) -> List[RetrievedChunk]:
        """
        Prioritize chunks by relevance score (highest first).

        Args:
            chunks: List of retrieved chunks to prioritize

        Returns:
            List of chunks sorted by relevance score (highest first)
        """
        return sorted(chunks, key=lambda x: x.score, reverse=True)

    def build_retrieved_context(
        self,
        chunks: List[RetrievedChunk],
        max_response_tokens: int
    ) -> tuple[str, Dict[str, int]]:
        """
        Build the retrieved context string respecting token limits.

        Args:
            chunks: Prioritized list of chunks to include
            max_response_tokens: Maximum tokens available for the response

        Returns:
            Tuple of (retrieved context string, chunk token counts)
        """
        # Calculate how many tokens we can use for context
        # Reserve some tokens for the system prompt and query
        reserved_tokens = 500  # Estimated tokens for system prompt and query
        available_context_tokens = self.max_context_tokens - reserved_tokens - max_response_tokens

        context_parts = ["Relevant book content:\n"]
        chunk_tokens = {}
        current_tokens = self.token_counter.count_tokens(context_parts[0])

        for i, chunk in enumerate(chunks):
            chunk_text = f"[{i+1}] {chunk.content}\n\n"
            chunk_token_count = self.token_counter.count_tokens(chunk_text)

            if current_tokens + chunk_token_count <= available_context_tokens:
                context_parts.append(chunk_text)
                chunk_tokens[f"[{i+1}]"] = chunk_token_count
                current_tokens += chunk_token_count
            else:
                # If adding this chunk would exceed limits, stop here
                break

        return "".join(context_parts), chunk_tokens

    def enhance_token_counter_with_context(self, context_window: ContextWindow) -> int:
        """
        Enhance token counting with accurate calculation for the context window.

        Args:
            context_window: The context window to count tokens for

        Returns:
            Total token count for the context window
        """
        total_tokens = 0
        total_tokens += self.token_counter.count_tokens(context_window.system_prompt)
        total_tokens += self.token_counter.count_tokens(context_window.conversation_context)
        total_tokens += self.token_counter.count_tokens(context_window.retrieved_context)
        total_tokens += self.token_counter.count_tokens(context_window.query_context)

        return total_tokens

    def enforce_token_limits(self, context_window: ContextWindow, max_tokens: int) -> ContextWindow:
        """
        Enforce token limits on the context window.

        Args:
            context_window: The context window to enforce limits on
            max_tokens: Maximum tokens allowed

        Returns:
            ContextWindow with enforced token limits
        """
        current_total = self.enhance_token_counter_with_context(context_window)

        if current_total <= max_tokens:
            # Context is within limits
            context_window.total_tokens = current_total
            context_window.available_tokens = max_tokens - current_total
            return context_window

        # Context exceeds limits - need to reduce
        # First, try reducing conversation history
        reduced_conv_context = self._reduce_conversation_context(
            context_window.conversation_context,
            max_tokens,
            context_window
        )

        if self.token_counter.count_tokens(reduced_conv_context) + \
           self.token_counter.count_tokens(context_window.retrieved_context) + \
           self.token_counter.count_tokens(context_window.system_prompt) + \
           self.token_counter.count_tokens(context_window.query_context) <= max_tokens:
            context_window.conversation_context = reduced_conv_context
            new_total = self.enhance_token_counter_with_context(context_window)
            context_window.total_tokens = new_total
            context_window.available_tokens = max_tokens - new_total
            return context_window

        # If still over, reduce retrieved context
        reduced_retrieved_context = self._reduce_retrieved_context(
            context_window.retrieved_context,
            max_tokens,
            context_window
        )

        context_window.retrieved_context = reduced_retrieved_context
        new_total = self.enhance_token_counter_with_context(context_window)
        context_window.total_tokens = new_total
        context_window.available_tokens = max_tokens - new_total

        return context_window

    def _reduce_conversation_context(self, conv_context: str, max_tokens: int, original_context: ContextWindow) -> str:
        """
        Reduce conversation context to fit within token limits.

        Args:
            conv_context: The conversation context to reduce
            max_tokens: Maximum tokens allowed
            original_context: The original context window for reference

        Returns:
            Reduced conversation context
        """
        if not conv_context or self.token_counter.count_tokens(conv_context) == 0:
            return conv_context

        # Use token-aware truncation
        max_conv_tokens = max_tokens - (
            self.token_counter.count_tokens(original_context.system_prompt) +
            self.token_counter.count_tokens(original_context.retrieved_context) +
            self.token_counter.count_tokens(original_context.query_context) +
            100  # Buffer for safety
        )

        if max_conv_tokens <= 0:
            return ""  # No room for conversation context

        return self.token_counter.truncate_text(conv_context, max_conv_tokens)

    def _reduce_retrieved_context(self, retrieved_context: str, max_tokens: int, original_context: ContextWindow) -> str:
        """
        Reduce retrieved context to fit within token limits.

        Args:
            retrieved_context: The retrieved context to reduce
            max_tokens: Maximum tokens allowed
            original_context: The original context window for reference

        Returns:
            Reduced retrieved context
        """
        if not retrieved_context or self.token_counter.count_tokens(retrieved_context) == 0:
            return retrieved_context

        # Calculate available tokens after accounting for other components
        other_tokens = (
            self.token_counter.count_tokens(original_context.system_prompt) +
            self.token_counter.count_tokens(original_context.conversation_context) +
            self.token_counter.count_tokens(original_context.query_context) +
            100  # Buffer for safety
        )

        max_retrieved_tokens = max_tokens - other_tokens

        if max_retrieved_tokens <= 0:
            return ""  # No room for retrieved context

        return self.token_counter.truncate_text(retrieved_context, max_retrieved_tokens)

    def add_dynamic_truncation(self, text: str, max_tokens: int) -> str:
        """
        Add dynamic truncation of text to fit within token limits.

        Args:
            text: The text to truncate
            max_tokens: Maximum tokens allowed

        Returns:
            Truncated text that fits within the token limit
        """
        return self.token_counter.truncate_text(text, max_tokens)

    def manage_conversation_history(
        self,
        history: List[Dict[str, str]],
        max_history_tokens: int = 2000
    ) -> List[Dict[str, str]]:
        """
        Manage conversation history to stay within token limits.

        Args:
            history: List of conversation exchanges
            max_history_tokens: Maximum tokens for conversation history

        Returns:
            Trimmed conversation history that fits within token limits
        """
        if not history:
            return []

        # Start with the most recent exchanges and work backwards
        token_count = 0
        trimmed_history = []

        for exchange in reversed(history):
            exchange_text = f"Q: {exchange['query']}\nA: {exchange['answer']}\n\n"
            exchange_tokens = self.token_counter.count_tokens(exchange_text)

            if token_count + exchange_tokens <= max_history_tokens:
                trimmed_history.insert(0, exchange)  # Add to beginning to maintain order
                token_count += exchange_tokens
            else:
                # If adding this exchange would exceed limits, stop here
                break

        return trimmed_history

    def enhance_conversation_context(self, conversation_history: List[Dict[str, str]]) -> ConversationContext:
        """
        Enhance conversation context with improved formatting and token management.

        Args:
            conversation_history: List of conversation exchanges

        Returns:
            Enhanced ConversationContext object
        """
        # Use the existing logic to manage history length
        managed_history = self.manage_conversation_history(conversation_history)

        # Create a ConversationContext object with the managed history
        enhanced_context = ConversationContext(
            history=managed_history,
            max_history_length=len(managed_history),
            context_window_tokens=0,  # Will be calculated separately
            last_access_time=time.time()
        )

        # Calculate the token count for this conversation context
        context_text = enhanced_context.get_formatted_history()
        enhanced_context.context_window_tokens = self.token_counter.count_tokens(context_text)

        return enhanced_context

    def implement_stateless_design(self, conversation_history: List[Dict[str, str]]) -> List[Dict[str, str]]:
        """
        Implement stateless design with explicit session history management.

        Args:
            conversation_history: The conversation history to manage

        Returns:
            The conversation history as passed in (stateless - no internal storage)
        """
        # This is inherently stateless - we simply return the history as received
        # without storing it internally, maintaining the stateless design
        return conversation_history

    def add_history_length_limiting(self, conversation_history: List[Dict[str, str]], max_exchanges: int = 10) -> List[Dict[str, str]]:
        """
        Add history length limiting to prevent token overflow.

        Args:
            conversation_history: List of conversation exchanges
            max_exchanges: Maximum number of exchanges to keep

        Returns:
            Trimmed conversation history with limited length
        """
        if len(conversation_history) <= max_exchanges:
            return conversation_history

        # Keep the most recent exchanges
        return conversation_history[-max_exchanges:]

    def integrate_conversation_with_context(self, context_window: ContextWindow, conversation_history: List[Dict[str, str]]) -> ContextWindow:
        """
        Integrate conversation history with context window construction.

        Args:
            context_window: The existing context window
            conversation_history: The conversation history to integrate

        Returns:
            Updated context window with integrated conversation history
        """
        # Enhance the conversation context
        enhanced_conversation_context = self.enhance_conversation_context(conversation_history)

        # Update the context window with the enhanced conversation context
        context_window.conversation_context = enhanced_conversation_context.get_formatted_history()

        # Recalculate token counts after integration
        context_window.total_tokens = self.enhance_token_counter_with_context(context_window)
        context_window.available_tokens = self.max_context_tokens - context_window.total_tokens

        return context_window

    def validate_context_window(self, context_window: ContextWindow) -> Dict[str, Any]:
        """
        Validate that the context window respects token limits.

        Args:
            context_window: The context window to validate

        Returns:
            Dictionary with validation results
        """
        validation_results = {
            "within_token_limit": context_window.total_tokens <= self.max_context_tokens,
            "total_tokens": context_window.total_tokens,
            "max_tokens": self.max_context_tokens,
            "available_for_response": context_window.available_tokens,
            "issues": []
        }

        if not validation_results["within_token_limit"]:
            validation_results["issues"].append(
                f"Context window ({context_window.total_tokens} tokens) exceeds "
                f"maximum ({self.max_context_tokens} tokens)"
            )

        if context_window.available_tokens < 0:
            validation_results["issues"].append(
                f"Available tokens for response is negative ({context_window.available_tokens})"
            )

        return validation_results

    def calculate_optimal_chunks(
        self,
        chunks: List[RetrievedChunk],
        context_budget: int
    ) -> List[RetrievedChunk]:
        """
        Calculate the optimal set of chunks to include within the token budget.

        Args:
            chunks: List of chunks to choose from (assumed to be sorted by relevance)
            context_budget: Number of tokens available for chunks

        Returns:
            List of chunks that fit within the budget, maintaining relevance order
        """
        selected_chunks = []
        current_token_count = 0

        for chunk in chunks:
            chunk_token_count = self.token_counter.count_tokens(chunk.content)
            citation_token_count = self.token_counter.count_tokens(f"[X] \n\n")  # Approximate citation overhead

            if current_token_count + chunk_token_count + citation_token_count <= context_budget:
                selected_chunks.append(chunk)
                current_token_count += chunk_token_count + citation_token_count
            else:
                # If adding this chunk would exceed the budget, stop here
                break

        return selected_chunks


# Global context manager instance
context_manager = ContextManager()