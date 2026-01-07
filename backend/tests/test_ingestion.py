"""
Tests for ingestion module
"""
import pytest
from src.ingestion.cleaner import clean_text


def test_clean_text_with_html():
    """Test that HTML tags are removed from text."""
    html_content = "<html><body><p>This is a <strong>test</strong> paragraph.</p></body></html>"
    expected = "This is a test paragraph."

    result = clean_text(html_content)
    assert expected in result


def test_clean_text_empty():
    """Test that empty content returns empty string."""
    result = clean_text("")
    assert result == ""


def test_clean_text_none():
    """Test that None content returns empty string."""
    result = clean_text(None)
    assert result == ""