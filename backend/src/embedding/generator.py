"""
Embedding Generation Module
Generates embeddings using the Cohere API.
"""
import cohere
import time
from typing import List, Dict, Any
from src.utils.config import load_config
from src.utils.logger import setup_logger


logger = setup_logger()
config = load_config()

# Initialize Cohere client
cohere_client = cohere.Client(config.COHERE_API_KEY)


def generate_embeddings(chunks: List[Dict[str, Any]], model: str = None) -> List[List[float]]:
    """
    Generate embeddings for text chunks using Cohere.

    Args:
        chunks: List of chunk dictionaries containing 'content' field
        model: Embedding model to use (defaults to config value)

    Returns:
        List of embedding vectors
    """
    if not chunks:
        return []

    model = model or config.COHERE_MODEL
    texts = [chunk['content'] for chunk in chunks]

    try:
        logger.info(f"Generating embeddings for {len(texts)} chunks using model: {model}")

        # Process embeddings in smaller batches to avoid rate limits
        all_embeddings = []
        batch_size = 32  # Conservative batch size for Cohere API

        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]
            logger.info(f"Processing batch {i//batch_size + 1}/{(len(texts) - 1)//batch_size + 1} with {len(batch)} items")

            max_retries = 5
            retry_count = 0

            while retry_count < max_retries:
                try:
                    response = cohere_client.embed(
                        texts=batch,
                        model=model,
                        input_type="search_document"  # Optimize for search use case
                    )
                    break
                except cohere.errors.TooManyRequestsError as e:
                    retry_count += 1
                    if retry_count >= max_retries:
                        logger.error(f"Max retries reached for embeddings generation: {str(e)}")
                        raise
                    else:
                        delay = config.RATE_LIMIT_DELAY * (2 ** retry_count)  # Exponential backoff
                        logger.warning(f"Rate limited (429), waiting {delay}s before retry {retry_count}/{max_retries}")
                        time.sleep(delay)
                except Exception as e:
                    logger.error(f"Error generating embeddings for batch: {str(e)}")
                    raise

            batch_embeddings = response.embeddings
            all_embeddings.extend(batch_embeddings)
            logger.info(f"Generated {len(batch_embeddings)} embeddings for batch {i//batch_size + 1}")

            # Add a small delay between batches to be respectful to the API
            if i + batch_size < len(texts):
                time.sleep(0.5)

        logger.info(f"Generated {len(all_embeddings)} embeddings successfully")

        return all_embeddings

    except Exception as e:
        logger.error(f"Error generating embeddings: {str(e)}")
        raise