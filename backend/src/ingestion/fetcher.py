"""
URL Fetching Module
Fetches content from provided URLs using requests with proper error handling and rate limiting.
"""
import requests
import time
from typing import List
from src.utils.config import load_config
from src.utils.logger import setup_logger


logger = setup_logger()
config = load_config()


def fetch_urls(urls: List[str]) -> List[str]:
    """
    Fetch content from a list of URLs.

    Args:
        urls: List of URLs to fetch content from

    Returns:
        List of HTML content strings
    """
    contents = []

    for i, url in enumerate(urls):
        retry_count = 0
        max_retries = 3

        while retry_count < max_retries:
            try:
                # Add rate limiting delay
                if i > 0 or retry_count > 0:
                    time.sleep(config.RATE_LIMIT_DELAY)

                logger.info(f"Fetching URL: {url} (attempt {retry_count + 1})")
                response = requests.get(
                    url,
                    headers={
                        "User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36"
                    },
                    timeout=config.REQUEST_TIMEOUT
                )

                response.raise_for_status()
                contents.append(response.text)
                logger.info(f"Successfully fetched {url}")
                break  # Exit retry loop on success

            except requests.exceptions.RequestException as e:
                if isinstance(e, requests.exceptions.HTTPError) and hasattr(e.response, 'status_code'):
                    status_code = e.response.status_code
                    if status_code == 429:  # Rate limited
                        logger.warning(f"Rate limited for {url}, waiting longer...")
                        time.sleep(config.RATE_LIMIT_DELAY * 5)  # Wait longer for rate limiting
                        retry_count += 1
                    elif 400 <= status_code < 500:
                        logger.error(f"Client error for {url}: {status_code}")
                        contents.append("")  # Add empty string as placeholder
                        break  # Don't retry client errors
                    else:  # Server errors
                        logger.warning(f"Server error for {url}: {status_code}, retrying...")
                        retry_count += 1
                elif isinstance(e, requests.exceptions.Timeout):
                    logger.warning(f"Timeout for {url}, retrying...")
                    retry_count += 1
                else:
                    logger.error(f"Failed to fetch {url}: {str(e)}")
                    contents.append("")  # Add empty string as placeholder
                    break  # Don't retry other request exceptions

        if retry_count >= max_retries:
            logger.error(f"Failed to fetch {url} after {max_retries} attempts")
            contents.append("")  # Add empty string as placeholder

    return contents