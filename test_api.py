"""
Test script for the RAG Chatbot API
"""
import requests
import json

def test_api():
    base_url = "http://localhost:8002"

    # Test health endpoint
    print("Testing health endpoint...")
    try:
        response = requests.get(f"{base_url}/health")
        print(f"Health check: {response.status_code} - {response.json()}")
    except Exception as e:
        print(f"Health check failed: {e}")

    # Test chat endpoint
    print("\nTesting chat endpoint...")
    try:
        chat_payload = {
            "message": "What is this book about?",
            "session_id": "test_session_123",
            "options": {
                "top_k": 5,
                "temperature": 0.3
            }
        }

        response = requests.post(f"{base_url}/chat",
                                json=chat_payload,
                                headers={"Content-Type": "application/json"})
        print(f"Chat response: {response.status_code}")
        if response.status_code == 200:
            result = response.json()
            print(f"Answer: {result.get('answer', '')[:200]}...")
            print(f"Confidence: {result.get('confidence_score')}")
            print(f"Status: {result.get('status')}")
        else:
            print(f"Error: {response.text}")
    except Exception as e:
        print(f"Chat endpoint test failed: {e}")

    # Test root endpoint
    print("\nTesting root endpoint...")
    try:
        response = requests.get(f"{base_url}/")
        print(f"Root endpoint: {response.status_code}")
        if response.status_code == 200:
            result = response.json()
            print(f"Message: {result.get('message')}")
            print(f"Available endpoints: {len(result.get('endpoints', []))}")
    except Exception as e:
        print(f"Root endpoint test failed: {e}")

if __name__ == "__main__":
    test_api()