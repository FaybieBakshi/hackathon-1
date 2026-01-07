import asyncio
import time
from typing import Dict, Any, Optional
from dataclasses import dataclass
from datetime import datetime


@dataclass
class ChatSession:
    """Represents a chat session with conversation history"""
    session_id: str
    created_at: float
    last_activity: float
    history: list
    user_id: Optional[str] = None


class SessionManager:
    """Manages chat sessions with indefinite retention as per clarification"""

    def __init__(self):
        # In a real implementation, this would use Redis or database
        # For this implementation, we'll use in-memory storage
        self.sessions: Dict[str, ChatSession] = {}
        self.lock = asyncio.Lock()

    async def create_session(self, session_id: str, user_id: Optional[str] = None) -> ChatSession:
        """Create a new chat session"""
        async with self.lock:
            session = ChatSession(
                session_id=session_id,
                created_at=time.time(),
                last_activity=time.time(),
                history=[],
                user_id=user_id
            )
            self.sessions[session_id] = session
            return session

    async def get_session(self, session_id: str) -> Optional[ChatSession]:
        """Get an existing session by ID"""
        async with self.lock:
            session = self.sessions.get(session_id)
            if session:
                # Update last activity time
                session.last_activity = time.time()
            return session

    async def add_message_to_session(self, session_id: str, message: Dict[str, Any]) -> bool:
        """Add a message to the session history"""
        async with self.lock:
            session = self.sessions.get(session_id)
            if session:
                session.history.append(message)
                session.last_activity = time.time()
                return True
            return False

    async def get_session_history(self, session_id: str) -> list:
        """Get the message history for a session"""
        async with self.lock:
            session = self.sessions.get(session_id)
            if session:
                return session.history.copy()
            return []

    async def clear_session(self, session_id: str) -> bool:
        """Clear the history of a session while keeping the session"""
        async with self.lock:
            session = self.sessions.get(session_id)
            if session:
                session.history = []
                session.last_activity = time.time()
                return True
            return False


# Global session manager instance
session_manager = SessionManager()