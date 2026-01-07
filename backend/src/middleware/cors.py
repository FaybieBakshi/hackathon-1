from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from ..config import Settings, get_settings

def setup_cors(app: FastAPI):
    """Configure CORS middleware for Docusaurus frontend domains"""
    settings = get_settings()

    # Combine default origins with production origins based on environment
    all_origins = settings.allowed_origins + settings.production_origins

    app.add_middleware(
        CORSMiddleware,
        allow_origins=all_origins,
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )