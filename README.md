# AI Robotics Textbook - RAG Pipeline

This repository contains an educational AI robotics textbook with an integrated RAG (Retrieval-Augmented Generation) pipeline for semantic search capabilities.

## Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Architecture](#architecture)
- [RAG Pipeline](#rag-pipeline)
- [Validation System](#validation-system)
- [Installation](#installation)
- [Configuration](#configuration)
- [Usage](#usage)
- [API Endpoints](#api-endpoints)
- [Monitoring and Metrics](#monitoring-and-metrics)
- [Deployment](#deployment)
- [Documentation](#documentation)
- [Contributing](#contributing)

## Overview

This project provides a comprehensive educational resource for AI and robotics, featuring:
- Interactive educational content covering ROS 2, Digital Twin, AI-Robot Brain, and VLA (Vision-Language-Action models)
- Semantic search capabilities using a RAG pipeline
- Web-based textbook deployed on GitHub Pages
- Integration with Cohere for embeddings and Qdrant for vector storage
- Comprehensive validation and monitoring system

## Features

- **Modular Educational Content**: Four comprehensive modules covering key AI robotics topics
- **Semantic Search**: Find relevant content using natural language queries
- **Interactive Code Examples**: Run and experiment with code snippets directly in the browser
- **Mobile Responsive**: Optimized for learning on any device
- **Extensible Architecture**: Easy to add new content and features
- **Validation System**: Quality assurance for retrieval accuracy and performance
- **Real-time Monitoring**: Performance metrics and health checks
- **Metadata Filtering**: Search by chapter, section, and other metadata

## Architecture

The project consists of:
- **Frontend**: Docusaurus-based textbook website
- **Backend**: Python-based RAG pipeline with validation system
- **Vector Database**: Qdrant Cloud for storing embeddings
- **Embeddings**: Cohere API for generating vector representations
- **Validation System**: Quality assurance and monitoring services
- **Deployment**: GitHub Pages for the textbook, with backend services for the RAG pipeline

## RAG Pipeline

The RAG pipeline enables semantic search over the textbook content through the following components:

### Core Services
1. **Content Extraction**: Extracts content from the deployed textbook website
2. **Content Chunking**: Splits content into semantically meaningful chunks
3. **Embedding Generation**: Creates vector embeddings using Cohere API
4. **Vector Storage**: Stores embeddings in Qdrant vector database
5. **Semantic Search**: Provides semantic search capabilities

### Key Features
- Automatic content extraction from the textbook website
- Semantic chunking with overlap to preserve context
- High-quality embeddings using Cohere's multilingual model
- Efficient vector storage and retrieval
- Comprehensive error handling and retry mechanisms
- Real-time metrics and monitoring
- Metadata filtering for targeted searches

### Configuration
The pipeline is configured through environment variables:

```bash
# Cohere API Configuration
COHERE_API_KEY=your_cohere_api_key
COHERE_MODEL=embed-multilingual-v2.0  # Updated model
COHERE_EMBED_BATCH_SIZE=32
COHERE_REQUEST_TIMEOUT=30

# Qdrant Configuration
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_URL=your_qdrant_cloud_url
QDRANT_COLLECTION_NAME=ai_textbook_chunks  # Updated collection name
QDRANT_VECTOR_DIMENSION=768  # Updated dimension for multilingual model
QDRANT_REQUEST_TIMEOUT=30

# General Configuration
REQUEST_TIMEOUT=30
REQUEST_DELAY=0.1
MAX_RETRIES=3

# Chunking Configuration
CHUNK_MAX_SIZE=1000
CHUNK_MIN_SIZE=100
CHUNK_OVERLAP_RATIO=0.2
```

## Validation System

The RAG pipeline includes a comprehensive validation system to ensure quality and performance:

### Validation Components
1. **Semantic Alignment**: Measures alignment between queries and retrieved content
2. **Precision Metrics**: Evaluates relevance of retrieved results
3. **Recall Metrics**: Measures completeness of retrieved results
4. **Performance Monitoring**: Tracks latency and success rates
5. **Health Checks**: Monitors service availability

### Validation Criteria
- **Semantic Alignment**: ≥70% alignment with query intent
- **Precision**: ≥50% precision for retrieved results
- **Recall**: ≥30% recall for expected sources
- **Latency**: ≤1500ms response time
- **Success Rate**: ≥95% successful queries

## Installation

### Prerequisites
- Python 3.8+
- Access to Cohere API
- Access to Qdrant Cloud or self-hosted Qdrant instance

### Setup
1. Clone the repository:
```bash
git clone https://github.com/your-username/hackathon-01-AI-textbook.git
cd hackathon-01-AI-textbook
```

2. Navigate to the backend directory:
```bash
cd backend
```

3. Create and activate a virtual environment:
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

4. Install dependencies:
```bash
pip install -r requirements.txt
```

5. Create a `.env` file with your configuration (see Configuration section above)

## Usage

### Running the RAG Pipeline

The pipeline can be run in several ways:

#### Single URL Processing
```python
from backend.rag_pipeline.validation.pipelines.rag_pipeline import rag_pipeline

# Process a single URL
results = rag_pipeline.run_pipeline_from_single_url("https://your-book-site.com/chapter1")
```

#### Multiple URLs Processing
```python
# Process multiple URLs
urls = [
    "https://your-book-site.com/chapter1",
    "https://your-book-site.com/chapter2"
]
results = rag_pipeline.run_pipeline_from_urls(urls)
```

#### Entire Website Processing
```python
# Process an entire website
results = rag_pipeline.run_pipeline_from_website(
    base_url="https://your-book-site.com",
    max_pages=100
)
```

### Semantic Search
```python
from backend.rag_pipeline.validation.services.retrieval_service import retrieval_service
from backend.rag_pipeline.validation.models.validation_models import RetrievalQuery

# Create a retrieval query
query = RetrievalQuery(
    query_text="Explain neural networks in robotics",
    top_k=5,
    filters={"chapter": "Chapter 3"}  # Optional metadata filtering
)

# Perform semantic search
results = retrieval_service.retrieve_similar_chunks(query)

for result in results:
    print(f"Score: {result.similarity_score:.3f}")
    print(f"Content: {result.content[:200]}...")
    print(f"Source: {result.source_url}")
    print(f"Chapter: {result.chapter}")
```

### Validation
```python
from backend.rag_pipeline.validation.services.retrieval_service import retrieval_service
from backend.rag_pipeline.validation.models.validation_models import ValidationRequest

# Create a validation request
validation_request = ValidationRequest(
    query="Explain neural networks in robotics",
    expected_sources=["https://your-book-site.com/chapter3"],
    filters={"chapter": "Chapter 3"},
    top_k=5
)

# Perform validation
validation_result = retrieval_service.validate_retrieval_quality(validation_request)

print(f"Validation Passed: {validation_result.validation_passed}")
print(f"Semantic Alignment: {validation_result.semantic_alignment_score:.3f}")
print(f"Precision: {validation_result.precision_metric:.3f}")
print(f"Latency: {validation_result.latency_ms:.2f}ms")
```

## API Endpoints

The RAG pipeline validation system provides the following API endpoints:

### Validation Endpoints
- `POST /api/v1/retrieval/validate` - Validate retrieval quality for a given query
- `GET /api/v1/retrieval/test-interface` - Human-readable test interface
- `GET /api/v1/retrieval/health` - Health check for the validation service
- `POST /api/v1/retrieval/batch-validate` - Validate multiple retrieval requests in batch

### Metrics Endpoints
- `GET /metrics` - Metrics in Prometheus format
- `GET /metrics/summary` - JSON metrics summary
- `GET /health` - Health check for metrics service

## Monitoring and Metrics

The system provides comprehensive monitoring and metrics collection:

### Metrics Collected
- Retrieval latency (p50, p95, p99 percentiles)
- Validation success/failure rates
- API request rates and errors
- System health status
- Quality metrics (precision, recall, semantic alignment)

### Monitoring Tools
- Prometheus metrics endpoint at `/metrics`
- JSON summary at `/metrics/summary`
- Health check at `/health`
- Application logs in structured format

## Deployment

The RAG pipeline can be deployed using multiple methods:

### Local Development
Use the provided deployment scripts:

**Unix/Linux/macOS:**
```bash
cd backend/rag_pipeline/validation/scripts
chmod +x deploy-local.sh
sudo ./deploy-local.sh install
sudo ./deploy-local.sh start
```

**Windows:**
```cmd
cd backend\rag_pipeline\validation\scripts
powershell -File deploy-local.ps1 -Command Install
powershell -File deploy-local.ps1 -Command Start
```

### Production Deployment
For production environments:
```bash
cd backend/rag_pipeline/validation/scripts
chmod +x deploy-production.sh
sudo ./deploy-production.sh deploy
```

### Docker Deployment
Build and run with Docker:
```bash
cd backend/rag_pipeline/validation
chmod +x build-docker.sh
./build-docker.sh
docker-compose up -d
```

### Kubernetes Deployment
Apply the Kubernetes configuration:
```bash
kubectl apply -f backend/rag_pipeline/validation/k8s-deployment.yaml
```

## Documentation

- [API Documentation](backend/rag_pipeline/docs/api.md): Detailed API reference
- [User Guide](backend/rag_pipeline/docs/user_guide.md): Complete user guide for operating the pipeline
- [Configuration Validation](backend/rag_pipeline/utils/config_validator.py): Configuration validation utilities
- [Deployment Script](backend/rag_pipeline/scripts/deploy_pipeline.sh): Deployment script for setting up and running the pipeline

## Project Structure

```
backend/
└── rag_pipeline/            # RAG pipeline implementation
    ├── models/              # Data models (BookContent, ContentChunk, EmbeddingVector, VectorRecord)
    ├── services/            # Core services (content extraction, chunking, embedding generation, vector storage, semantic search)
    ├── utils/               # Utilities (logging, exceptions, helpers, content cleaning, validation)
    ├── pipelines/           # Pipeline orchestration (extraction pipeline, main RAG pipeline)
    ├── docs/                # Documentation (API docs, user guide)
    ├── tests/               # Unit and integration tests
    ├── scripts/             # Deployment and utility scripts
    ├── config.py            # Configuration management
    ├── __init__.py          # Package initialization
    └── run_rag_pipeline.py  # Main pipeline execution script
```

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes
4. Add tests for new functionality
5. Run the test suite: `python -m pytest backend/rag_pipeline/validation/tests/`
6. Commit your changes (`git commit -m 'Add amazing feature'`)
7. Push to the branch (`git push origin feature/amazing-feature`)
8. Open a Pull Request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Support

For support with the RAG pipeline or to report issues, please open an issue in this repository.