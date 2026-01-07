"""
URL Fetcher for Book Site
Fetches all book page URLs from the deployed site
"""
import requests
from bs4 import BeautifulSoup
from urllib.parse import urljoin, urlparse
from typing import List
import logging

from src.utils.logger import setup_logger
from src.utils.config import load_config

logger = setup_logger()
config = load_config()

def fetch_book_urls(base_url: str = None) -> List[str]:
    """
    Fetch all book page URLs from the deployed site.

    Args:
        base_url: The base URL of the book site. If None, uses config.BOOK_SITE_URL

    Returns:
        List of URLs to process
    """
    if not base_url:
        base_url = config.BOOK_SITE_URL

    if not base_url:
        logger.error("No base URL provided and BOOK_SITE_URL not configured")
        return []

    logger.info(f"Fetching book URLs from: {base_url}")

    # Try to get URLs from sitemap first
    urls = fetch_urls_from_sitemap(base_url)

    if not urls:
        logger.info("No URLs found in sitemap, trying alternative methods...")
        urls = fetch_urls_from_site(base_url)

    if not urls:
        logger.warning(f"No URLs found for site: {base_url}")
        # Return the base URL as a fallback
        urls = [base_url]

    logger.info(f"Found {len(urls)} URLs to process")
    return urls

def fetch_urls_from_sitemap(base_url: str) -> List[str]:
    """
    Fetch URLs from sitemap.xml
    """
    sitemap_url = urljoin(base_url, 'sitemap.xml')
    logger.info(f"Fetching sitemap from: {sitemap_url}")

    try:
        response = requests.get(sitemap_url, timeout=config.REQUEST_TIMEOUT)
        response.raise_for_status()

        # Parse the sitemap - try xml parser first, fallback to html parser
        try:
            soup = BeautifulSoup(response.content, 'xml')
        except Exception:
            # If xml parser fails, try html parser for XML content
            soup = BeautifulSoup(response.content, 'html.parser')
        urls = []

        # Look for <url><loc> elements in sitemap
        for loc in soup.find_all('loc'):
            url = loc.text.strip()
            if url.startswith(base_url):  # Only include URLs from the same domain
                urls.append(url)

        logger.info(f"Found {len(urls)} URLs in sitemap")
        return urls

    except requests.RequestException as e:
        logger.warning(f"Failed to fetch sitemap: {e}")
        return []
    except Exception as e:
        logger.warning(f"Failed to parse sitemap: {e}")
        return []

def fetch_urls_from_site(base_url: str) -> List[str]:
    """
    Alternative method to fetch URLs by parsing the site
    """
    logger.info(f"Attempting to find URLs by parsing site: {base_url}")

    try:
        response = requests.get(base_url, timeout=config.REQUEST_TIMEOUT)
        response.raise_for_status()

        soup = BeautifulSoup(response.content, 'html.parser')

        urls = set()
        # Look for links in navigation, main content areas, etc.
        for link in soup.find_all('a', href=True):
            href = link['href']
            full_url = urljoin(base_url, href)

            # Filter for likely book content pages
            if (full_url.startswith(base_url) and
                not any(exclude in full_url.lower() for exclude in [
                    'mailto:', 'javascript:', '.pdf', '.jpg', '.png', '.css', '.js'
                ]) and
                len(full_url) > len(base_url)):  # Make sure it's a subpage
                urls.add(full_url)

        url_list = list(urls)
        logger.info(f"Found {len(url_list)} URLs from site parsing")
        return url_list

    except requests.RequestException as e:
        logger.warning(f"Failed to fetch URLs from site: {e}")
        return []
    except Exception as e:
        logger.warning(f"Failed to parse site for URLs: {e}")
        return []