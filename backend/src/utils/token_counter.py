"""
Token counting utilities for context window management.
"""
import tiktoken
from typing import Dict, List, Union


class TokenCounter:
    """
    Utility class for counting tokens in text using tiktoken.
    """
    def __init__(self, encoding_name: str = "cl100k_base"):
        """
        Initialize the token counter with a specific encoding.

        Args:
            encoding_name: The name of the encoding to use (default: cl100k_base for GPT-4/GPT-3.5-turbo)
        """
        self.encoding_name = encoding_name
        self._encoding = None  # Lazy load the encoding
        self.cache: Dict[str, int] = {}

    @property
    def encoding(self):
        """Lazy load the encoding to avoid network calls during import."""
        if self._encoding is None:
            self._encoding = tiktoken.get_encoding(self.encoding_name)
        return self._encoding

    def count_tokens(self, text: str) -> int:
        """
        Count the number of tokens in a given text.

        Args:
            text: The text to count tokens for

        Returns:
            The number of tokens in the text
        """
        if text in self.cache:
            return self.cache[text]

        token_count = len(self.encoding.encode(text))
        self.cache[text] = token_count
        return token_count

    def count_tokens_batch(self, texts: List[str]) -> List[int]:
        """
        Count tokens for a batch of texts efficiently.

        Args:
            texts: List of texts to count tokens for

        Returns:
            List of token counts corresponding to each text
        """
        token_counts = []
        for text in texts:
            if text in self.cache:
                token_counts.append(self.cache[text])
            else:
                token_count = len(self.encoding.encode(text))
                self.cache[text] = token_count
                token_counts.append(token_count)
        return token_counts

    def truncate_text(self, text: str, max_tokens: int) -> str:
        """
        Truncate text to fit within a maximum token count.

        Args:
            text: The text to truncate
            max_tokens: Maximum number of tokens allowed

        Returns:
            Truncated text that fits within the token limit
        """
        tokens = self.encoding.encode(text)
        if len(tokens) <= max_tokens:
            return text

        truncated_tokens = tokens[:max_tokens]
        truncated_text = self.encoding.decode(truncated_tokens)

        # Try to end at a sentence or word boundary
        last_sentence = truncated_text.rfind('.')
        last_space = truncated_text.rfind(' ')

        if last_sentence > len(text) // 2:  # If the last sentence is in the second half
            truncated_text = truncated_text[:last_sentence + 1]
        elif last_space > len(text) // 2:  # If the last space is in the second half
            truncated_text = truncated_text[:last_space]

        return truncated_text

    def calculate_available_tokens(self, context: str, max_context_tokens: int = 4096) -> int:
        """
        Calculate how many tokens are available for content in a given context.

        Args:
            context: The context that will be used
            max_context_tokens: Maximum tokens allowed in the context (default: 4096 for GPT-3.5-turbo)

        Returns:
            Number of tokens available for additional content
        """
        context_tokens = self.count_tokens(context)
        available_tokens = max_context_tokens - context_tokens
        return max(0, available_tokens)


# Global token counter instance - Initialize lazily to avoid network calls during import
default_token_counter = None


def get_default_token_counter():
    """Get the default token counter instance, creating it lazily if needed."""
    global default_token_counter
    if default_token_counter is None:
        default_token_counter = TokenCounter()
    return default_token_counter


def count_tokens(text: str) -> int:
    """
    Count tokens in text using the default token counter.

    Args:
        text: The text to count tokens for

    Returns:
        The number of tokens in the text
    """
    return get_default_token_counter().count_tokens(text)


def truncate_text(text: str, max_tokens: int) -> str:
    """
    Truncate text to fit within a maximum token count using the default token counter.

    Args:
        text: The text to truncate
        max_tokens: Maximum number of tokens allowed

    Returns:
        Truncated text that fits within the token limit
    """
    return get_default_token_counter().truncate_text(text, max_tokens)


