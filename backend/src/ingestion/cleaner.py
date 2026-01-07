"""
Text Cleaning Module
Cleans HTML content and extracts meaningful text using BeautifulSoup.
"""
from bs4 import BeautifulSoup
from src.utils.logger import setup_logger


logger = setup_logger()


def clean_text(html_content: str) -> str:
    """
    Clean HTML content and extract meaningful text.

    Args:
        html_content: Raw HTML content

    Returns:
        Clean text content
    """
    if not html_content:
        return ""

    try:
        soup = BeautifulSoup(html_content, 'html.parser')

        # Remove script and style elements
        for script in soup(["script", "style"]):
            script.decompose()

        # Try to get main content (prioritize main content areas)
        main_content = soup.find('main') or soup.find('article') or soup.find('div', class_='main-content') or soup

        # Extract text
        text = main_content.get_text()

        # Clean up whitespace
        lines = (line.strip() for line in text.splitlines())
        chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
        text = ' '.join(chunk for chunk in chunks if chunk)

        logger.debug(f"Cleaned text length: {len(text)} characters")
        return text

    except Exception as e:
        logger.error(f"Error cleaning text: {str(e)}")
        # Return original content if cleaning fails
        return html_content