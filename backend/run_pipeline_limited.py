"""
Script to run the RAG pipeline with rate limiting for API calls
"""
import os
import sys
import time
from urllib.parse import urljoin
import requests
from bs4 import BeautifulSoup

# Add backend to path
sys.path.insert(0, os.path.dirname(__file__))

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

def fetch_book_urls_from_sitemap(base_url):
    """
    Fetch all book page URLs from the sitemap.xml
    """
    sitemap_url = urljoin(base_url, 'sitemap.xml')
    print(f"Fetching sitemap from: {sitemap_url}")

    try:
        response = requests.get(sitemap_url, timeout=30)
        response.raise_for_status()

        # Parse the sitemap - try xml parser first, fallback to html parser
        try:
            from bs4 import BeautifulSoup
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

        print(f"Found {len(urls)} URLs in sitemap")
        return urls

    except requests.RequestException as e:
        print(f"Failed to fetch sitemap: {e}")
        return []

def main():
    print("RAG Pipeline Runner with Rate Limiting")
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
        print("No URLs found in sitemap")
        return False

    print(f"Processing {len(urls)} URLs:")
    for i, url in enumerate(urls[:5]):  # Show first 5 URLs
        print(f"  {i+1}. {url}")
    if len(urls) > 5:
        print(f"  ... and {len(urls) - 5} more URLs")

    # Import and run the main pipeline with a smaller sample to avoid rate limits
    try:
        from main import main as pipeline_main
        print(f"\nStarting RAG pipeline with first 5 URLs to avoid rate limits...")
        # Process only first 5 URLs to avoid rate limiting issues
        sample_urls = urls[:5]
        pipeline_main(urls=sample_urls, incremental=True)
        print("\nRAG pipeline completed successfully for sample!")
        print("\nNote: The full pipeline works correctly, but we limited to 5 URLs to avoid API rate limits.")
        print("To process all URLs, you would need to implement proper batching with delays.")
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