# Quickstart Guide: Retrieval Pipeline Validation for Book-Aware RAG System

**Feature**: 005-retrieval-validation
**Date**: 2025-12-16

## Overview

This guide provides instructions to quickly set up and run the retrieval validation system for the RAG pipeline. The system validates that queries retrieve relevant book content from the vector database with semantic alignment to user intent.

## Prerequisites

- Python 3.8+
- pip package manager
- Git (for cloning the repository)
- Cohere API key
- Qdrant Cloud instance access
- Access to the book content that has been indexed in Qdrant

## Setup Instructions

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-directory>
```

### 2. Create Virtual Environment

```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies

```bash
pip install fastapi uvicorn cohere qdrant-client pydantic pytest requests
```

### 4. Configure Environment Variables

Create a `.env` file in the project root with the following:

```bash
# Cohere API configuration
COHERE_API_KEY=your_cohere_api_key_here

# Qdrant configuration
QDRANT_URL=your_qdrant_cloud_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION_NAME=book_chunks

# Application configuration
LOG_LEVEL=INFO
PORT=8001
HOST=0.0.0.0
```

### 5. Verify Configuration

```bash
# Check that environment variables are properly set
python -c "
import os
print('COHERE_API_KEY configured:', bool(os.getenv('COHERE_API_KEY')))
print('QDRANT_URL configured:', bool(os.getenv('QDRANT_URL')))
print('QDRANT_API_KEY configured:', bool(os.getenv('QDRANT_API_KEY')))
"
```

## Running the Validation System

### 1. Start the Validation Server

```bash
uvicorn backend.rag_pipeline.validation.api.validation_endpoints:app --host 0.0.0.0 --port 8001
```

Or if you have a main application file:

```bash
uvicorn main:app --host 0.0.0.0 --port 8001
```

### 2. Verify Server is Running

Open your browser to: `http://localhost:8001/health` or `http://localhost:8001/docs` for API documentation

### 3. Test Basic Retrieval Validation

```bash
curl -X POST http://localhost:8001/api/v1/retrieval/validate \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are neural networks?",
    "top_k": 3,
    "filters": {
      "chapter": "Chapter 3"
    }
  }'
```

### 4. Access the Test Interface

Visit: `http://localhost:8001/api/v1/retrieval/test-interface?q=neural%20networks&k=5`

## Validation Examples

### Example 1: General Book Question

```bash
curl -X POST http://localhost:8001/api/v1/retrieval/validate \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Explain the fundamentals of machine learning",
    "top_k": 5
  }'
```

### Example 2: Chapter-Specific Question

```bash
curl -X POST http://localhost:8001/api/v1/retrieval/validate \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the main algorithms covered in Chapter 4?",
    "top_k": 3,
    "filters": {
      "chapter": "Chapter 4"
    }
  }'
```

### Example 3: Paragraph-Level Query

```bash
curl -X POST http://localhost:8001/api/v1/retrieval/validate \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Find the specific formula for backpropagation",
    "top_k": 2,
    "filters": {
      "section": "Backpropagation Algorithm"
    }
  }'
```

## CLI Validation Script

### Running Batch Validation

Create a simple validation script to test multiple queries:

```python
# validate_retrieval.py
import requests
import json

BASE_URL = "http://localhost:8001/api/v1/retrieval/validate"

test_queries = [
    {
        "query": "What are neural networks?",
        "top_k": 3
    },
    {
        "query": "Explain reinforcement learning",
        "top_k": 4,
        "filters": {
            "chapter": "Chapter 5"
        }
    },
    {
        "query": "How does gradient descent work?",
        "top_k": 2
    }
]

for i, query_obj in enumerate(test_queries):
    print(f"Testing query {i+1}: {query_obj['query'][:50]}...")

    response = requests.post(BASE_URL, json=query_obj)

    if response.status_code == 200:
        result = response.json()
        print(f"  ✓ Retrieved {len(result['retrieved_results'])} results")
        print(f"  ✓ Precision: {result['precision_metric']:.2f}")
        print(f"  ✓ Latency: {result['latency_ms']:.2f}ms")
        print(f"  ✓ Validation passed: {result['validation_passed']}")
    else:
        print(f"  ✗ Error: {response.status_code} - {response.text}")

    print()
```

Run the script:
```bash
python validate_retrieval.py
```

## Testing and Quality Assurance

### Run Unit Tests

```bash
python -m pytest tests/validation/ -v
```

### Run Validation Against Success Criteria

```bash
python -m pytest tests/validation/test_quality_metrics.py -v
```

## Monitoring and Debugging

### Check Validation Results

The system provides detailed validation results including:
- Precision and recall metrics
- Latency measurements
- Source attribution for each retrieved chunk
- Semantic alignment scores

### Debug Interface

Access the debug interface at:
`http://localhost:8001/api/v1/retrieval/test-interface`

This provides a human-readable interface to test queries and see detailed results.

## Troubleshooting

### Common Issues

1. **Connection Errors**:
   - Verify QDRANT_URL and QDRANT_API_KEY are correct
   - Check network connectivity to Qdrant Cloud

2. **API Key Errors**:
   - Verify COHERE_API_KEY is valid
   - Check for rate limiting on Cohere API

3. **No Results Returned**:
   - Verify the book content has been properly indexed in Qdrant
   - Check that the collection name matches QDRANT_COLLECTION_NAME

4. **High Latency**:
   - Monitor Cohere API usage for rate limits
   - Check Qdrant Cloud instance performance

### Enable Verbose Logging

Set LOG_LEVEL to DEBUG in your environment:
```bash
export LOG_LEVEL=DEBUG
```

## Next Steps

1. **Customize Validation Criteria**: Modify validation thresholds based on your specific requirements
2. **Add More Test Queries**: Expand the test suite with domain-specific queries
3. **Performance Testing**: Run stress tests to validate performance under load
4. **Integration Testing**: Test the validation system with your full RAG pipeline