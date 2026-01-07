import asyncio
import json
from typing import AsyncGenerator
from src.api import app
from src.agents.rag_agent import RAGAgent
from src.session_manager import session_manager


async def test_api_endpoints():
    """Test the implemented API endpoints"""
    print("Testing RAG Chatbot API endpoints...")

    # Test the RAG agent directly
    print("\n1. Testing RAG Agent directly:")
    agent = RAGAgent()

    # Test basic query
    print("   Testing basic query...")
    result = await agent.query("What is FastAPI?")
    print(f"   Query: {result.query}")
    print(f"   Answer: {result.answer[:100]}...")
    print(f"   Citations: {len(result.citations)}")

    # Test query with selection
    print("\n   Testing query with selection...")
    result2 = await agent.query_with_selection("Explain this", "FastAPI is a modern, fast web framework for building APIs with Python")
    print(f"   Query: {result2.query}")
    print(f"   Answer: {result2.answer[:100]}...")

    # Test streaming
    print("\n   Testing streaming response...")
    stream_gen = agent.stream_query("Tell me about APIs")
    response_parts = []
    async for chunk in stream_gen:
        response_parts.append(chunk)
        if len(response_parts) >= 3:  # Just take first few chunks for demo
            break
    print(f"   Stream response parts: {response_parts}")

    # Test session management
    print("\n2. Testing Session Management:")
    session_id = "test_session_123"

    # Create a session
    session = await session_manager.create_session(session_id)
    print(f"   Created session: {session.session_id}")

    # Add a message to the session
    await session_manager.add_message_to_session(session_id, {
        "type": "user",
        "content": "What is FastAPI?",
        "timestamp": session.created_at
    })

    # Get session history
    history = await session_manager.get_session_history(session_id)
    print(f"   Session history has {len(history)} messages")

    print("\n3. API Endpoints Available:")
    print("   - GET / (root)")
    print("   - POST /chat (query endpoint)")
    print("   - POST /chat_with_selection (query with selection)")
    print("   - GET /chat/stream (streaming query)")
    print("   - GET /chat_with_selection/stream (streaming with selection)")
    print("   - GET /health (health check)")
    print("   - GET /docs (Swagger UI)")
    print("   - GET /redoc (ReDoc)")

    print("\n4. To run the server:")
    print("   python main.py")
    print("   Or: uvicorn src.api:app --reload --port 8000")

    print("\n5. Example API calls:")
    print("   curl -X POST http://localhost:8000//chat -d 'query=What is FastAPI?'")
    print("   curl -X POST http://localhost:8000//chat_with_selection -d 'query=Explain this&selected_text=FastAPI is great'")
    print("   GET http://localhost:8000/health")

    print("\nAll tests completed successfully!")


def run_tests():
    """Run the API tests"""
    print("Starting RAG Chatbot API tests...")
    asyncio.run(test_api_endpoints())


if __name__ == "__main__":
    run_tests()