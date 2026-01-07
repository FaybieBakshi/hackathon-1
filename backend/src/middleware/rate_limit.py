from fastapi import FastAPI, Request, HTTPException
from fastapi.responses import JSONResponse
from typing import Dict, Optional
from time import time
from ..config import get_settings


class RateLimiter:
    def __init__(self):
        self.requests: Dict[str, list] = {}
        settings = get_settings()
        self.max_requests = settings.rate_limit_requests
        self.window_size = settings.rate_limit_window

    def is_allowed(self, identifier: str) -> bool:
        """Check if a request from the identifier is allowed"""
        current_time = time()

        # Clean old requests outside the window
        if identifier in self.requests:
            self.requests[identifier] = [
                req_time for req_time in self.requests[identifier]
                if current_time - req_time < self.window_size
            ]
        else:
            self.requests[identifier] = []

        # Check if limit is exceeded
        if len(self.requests[identifier]) >= self.max_requests:
            return False

        # Add current request
        self.requests[identifier].append(current_time)
        return True


# Global rate limiter instance
rate_limiter = RateLimiter()


def rate_limit_middleware(app: FastAPI):
    """Add rate limiting middleware to the FastAPI app"""

    @app.middleware("http")
    async def limit_requests(request: Request, call_next):
        # Get client IP
        client_ip = request.client.host

        # Check if request is allowed
        if not rate_limiter.is_allowed(client_ip):
            return JSONResponse(
                status_code=429,
                content={"detail": "Rate limit exceeded"}
            )

        response = await call_next(request)
        return response