import uvicorn
import argparse
from src.api import app
from src.config import get_settings


def main():
    settings = get_settings()

    parser = argparse.ArgumentParser(description="RAG Chatbot API Server")
    parser.add_argument("--host", default="0.0.0.0", help="Host to bind to")
    parser.add_argument("--port", type=int, default=8000, help="Port to bind to")
    parser.add_argument("--reload", action="store_true", help="Enable auto-reload")

    args = parser.parse_args()

    print(f"Starting RAG Chatbot API server on {args.host}:{args.port}")
    print(f"Environment: {settings.environment}")

    uvicorn.run(
        "src.api:app",
        host=args.host,
        port=args.port,
        reload=args.reload,
        log_level=settings.log_level.lower()
    )


if __name__ == "__main__":
    main()
