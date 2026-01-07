# Quickstart: RAG Embedding Pipeline

## Prerequisites

- Python 3.11 or higher
- pip package manager
- UV (alternative to pip, optional but recommended)

## Setup

### 1. Clone and Navigate to Backend Directory

```bash
mkdir -p backend
cd backend
```

### 2. Create Virtual Environment and Install Dependencies

Using UV (recommended):
```bash
uv venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
uv pip install -r requirements.txt
```

Or using pip:
```bash
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

### 3. Set Up Environment Variables

Copy the example environment file:
```bash
cp .env.example .env
```

Then edit `.env` with your actual API keys:
```bash
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_URL=your_qdrant_cluster_url_here
```

## Usage

### 1. Run the Full Pipeline

Execute the main script to run the complete pipeline:
```bash
python src/main.py
```

### 2. Run with Specific URLs

To process specific book pages:
```bash
python src/main.py --urls "https://your-book.vercel.app/chapter1" "https://your-book.vercel.app/chapter2"
```

### 3. Run Incremental Processing

To process only changed/new pages:
```bash
python src/main.py --incremental
```

## Configuration

The pipeline can be configured via environment variables in `.env`:

- `COHERE_API_KEY`: Your Cohere API key for embedding generation
- `QDRANT_URL`: Your Qdrant Cloud cluster URL
- `QDRANT_API_KEY`: Your Qdrant API key
- `CHUNK_SIZE_MIN`: Minimum chunk size in tokens (default: 500)
- `CHUNK_SIZE_MAX`: Maximum chunk size in tokens (default: 800)
- `CHUNK_OVERLAP`: Overlap between chunks in tokens (default: 100)
- `RATE_LIMIT_DELAY`: Delay between API calls in seconds (default: 1)

## Testing

Run the test suite:
```bash
pytest tests/
```

Run specific tests:
```bash
pytest tests/test_ingestion.py  # Test ingestion components
pytest tests/test_chunking.py   # Test chunking logic
pytest tests/test_embedding.py  # Test embedding generation
pytest tests/test_storage.py    # Test storage operations
```

## Validation

After running the pipeline, validate the results:
```bash
python src/validation/validator.py
```

This will check that >99% of chunks are correctly stored and retrievable.