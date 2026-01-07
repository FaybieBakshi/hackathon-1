"""
Script to run the RAG pipeline with actual book URLs from the deployed site
"""
import os
import sys
from urllib.parse import urljoin, urlparse
import requests
from bs4 import BeautifulSoup

# Add backend to path
sys.path.insert(0, os.path.dirname(__file__))

def fetch_book_urls_from_sitemap(base_url):
    """
    Fetch all book page URLs from the sitemap.xml
    """
    sitemap_url = urljoin(base_url, 'sitemap.xml')
    print(f"Fetching sitemap from: {sitemap_url}")

    try:
        response = requests.get(sitemap_url, timeout=30)
        response.raise_for_status()

        # Parse the sitemap
        soup = BeautifulSoup(response.content, 'xml')
        urls = []

        # Look for <url><loc> elements in sitemap
        for loc in soup.find_all('loc'):
            url = loc.text.strip()
            if url.startswith(base_url):  # Only include URLs from the same domain
                urls.append(url)

        print(f"Found {len(urls)} URLs in sitemap")
        return urls

    except requests.RequestException as e:
        print(f"Failed to fetch sitemap: {e}")
        # If sitemap fails, try to get URLs by other means
        return fetch_urls_from_nav(base_url)

def fetch_urls_from_nav(base_url):
    """
    Alternative method to fetch URLs by looking for navigation elements
    """
    print(f"Attempting to find URLs from navigation on: {base_url}")

    try:
        response = requests.get(base_url, timeout=30)
        response.raise_for_status()

        soup = BeautifulSoup(response.content, 'html.parser')

        urls = set()
        # Look for links in navigation, main content areas, etc.
        for link in soup.find_all('a', href=True):
            href = link['href']
            full_url = urljoin(base_url, href)

            # Filter for likely book content pages (not navigation elements)
            if (full_url.startswith(base_url) and
                not any(exclude in full_url for exclude in ['/', '#', 'mailto:', 'javascript:']) and
                len(full_url) > len(base_url)):  # Make sure it's a subpage
                urls.add(full_url)

        url_list = list(urls)
        print(f"Found {len(url_list)} URLs from navigation")
        return url_list

    except requests.RequestException as e:
        print(f"Failed to fetch URLs from navigation: {e}")
        return []

def load_deploy_url():
    """
    Load the deploy URL from environment or .env file
    """
    # Check environment variable first
    deploy_url = os.getenv('DEPLOY_VERCEL_URL')

    if not deploy_url:
        # If not in environment, try to read from .env file
        env_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), '.env')
        if os.path.exists(env_path):
            with open(env_path, 'r') as f:
                for line in f:
                    if line.startswith('DEPLOY_VERCEL_URL='):
                        deploy_url = line.split('=', 1)[1].strip().strip('"\'')
                        # Remove any comment after the URL
                        if '#' in deploy_url:
                            deploy_url = deploy_url.split('#')[0].strip().rstrip('",')
                        break

    return deploy_url

def main():
    print("RAG Pipeline Runner")
    print("="*50)

    # Load the deploy URL
    deploy_url = load_deploy_url()

    if not deploy_url:
        print("ERROR: DEPLOY_VERCEL_URL not found in environment or .env file")
        print("Please set the DEPLOY_VERCEL_URL environment variable or add it to your .env file")
        return False

    print(f"Using deploy URL: {deploy_url}")

    # Fetch URLs from the deployed site
    urls = fetch_book_urls_from_sitemap(deploy_url)

    if not urls:
        print("No URLs found, using a sample URL for testing")
        urls = [deploy_url]  # Use the main URL as fallback

    print(f"Processing {len(urls)} URLs:")
    for i, url in enumerate(urls[:5]):  # Show first 5 URLs
        print(f"  {i+1}. {url}")
    if len(urls) > 5:
        print(f"  ... and {len(urls) - 5} more URLs")

    # Import and run the main pipeline
    try:
        from src.main import main as pipeline_main
        print(f"\nStarting RAG pipeline with {len(urls)} URLs...")
        pipeline_main(urls=urls, incremental=True)  # Use incremental processing
        print("\nRAG pipeline completed successfully!")
        return True
    except Exception as e:
        print(f"Error running pipeline: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = main()
    if not success:
        sys.exit(1)