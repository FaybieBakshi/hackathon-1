from abc import ABC, abstractmethod
from typing import List, Dict, Any, AsyncGenerator
from dataclasses import dataclass


@dataclass
class QueryResult:
    """Represents the result of a RAG query"""
    answer: str
    citations: List[Dict[str, Any]]
    sources: List[str]
    query: str
    timestamp: float


class BaseRAGAgent(ABC):
    """Base interface for RAG agents"""

    @abstractmethod
    async def query(self, query_text: str) -> QueryResult:
        """Process a text query and return a result with citations"""
        pass

    @abstractmethod
    async def query_with_selection(self, query_text: str, selected_text: str) -> QueryResult:
        """Process a query with selected text context and return a result with citations"""
        pass

    @abstractmethod
    async def stream_query(self, query_text: str) -> AsyncGenerator[str, None]:
        """Stream query response as SSE events"""
        pass

    @abstractmethod
    async def stream_query_with_selection(self, query_text: str, selected_text: str) -> AsyncGenerator[str, None]:
        """Stream query response with selection context as SSE events"""
        pass