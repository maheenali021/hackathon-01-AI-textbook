# Implementation Plan: Retrieval Pipeline Validation for Book-Aware RAG System

**Branch**: `005-retrieval-validation` | **Date**: 2025-12-16 | **Spec**: specs/005-retrieval-validation/spec.md
**Input**: Feature specification from `/specs/005-retrieval-validation/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of a validation system for the retrieval pipeline that extracts relevant book content from the vector database to ensure accurate, grounded context is provided to the RAG agent before response generation. The system will validate that queries retrieve relevant document chunks from Qdrant with semantic alignment to user intent, supporting general book questions, chapter-specific questions, and paragraph-level queries with retrieval transparency and performance monitoring.

## Technical Context

**Language/Version**: Python 3.8+
**Primary Dependencies**: FastAPI, Cohere, qdrant-client, Pydantic
**Storage**: Qdrant Cloud vector database, Neon Serverless Postgres for user data
**Testing**: pytest, FastAPI TestClient
**Target Platform**: Linux server, cloud deployment
**Project Type**: web - determines source structure
**Performance Goals**: 95%+ of queries under 1.5 seconds, 90%+ precision rate
**Constraints**: <1.5s p95 retrieval latency, 90%+ precision, secure API key handling
**Scale/Scope**: Support 1000+ concurrent validation queries, handle book-sized content

### Dependencies
- `fastapi` - For API endpoints and server
- `uvicorn` - For ASGI server
- `cohere` - For embedding generation
- `qdrant-client` - For Qdrant vector database interaction
- `pydantic` - For data validation
- `pytest` - For testing framework
- `requests` - For HTTP requests

### Unknowns (NEEDS CLARIFICATION)
- **Cohere API key configuration**: How will the Cohere API key be securely configured in the environment?
- **Qdrant connection details**: What are the specific connection parameters for the Qdrant Cloud instance?
- **Existing vector collection structure**: What is the schema of the existing Qdrant collection for book content?
- **Book content indexing method**: How was the book content originally indexed in Qdrant?

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Technology Stack Compliance**: Uses FastAPI and Qdrant Cloud as required by constitution
- **Grounded AI**: Ensures retrieval pipeline validates that content is grounded in book content (FR-006, FR-007)
- **RAG Backend Integration**: Integrates with existing RAG infrastructure for retrieval validation
- **API Design and Security**: Implements secure API endpoints with proper authentication
- **Production-Grade Engineering**: Includes comprehensive error handling and logging

## Project Structure

### Documentation (this feature)

```text
specs/005-retrieval-validation/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── rag_pipeline/
│   ├── validation/
│   │   ├── __init__.py
│   │   ├── retrieval_validator.py
│   │   ├── api/
│   │   │   ├── __init__.py
│   │   │   ├── validation_endpoints.py
│   │   │   └── test_interface.py
│   │   ├── models/
│   │   │   ├── __init__.py
│   │   │   └── validation_models.py
│   │   ├── services/
│   │   │   ├── __init__.py
│   │   │   └── retrieval_service.py
│   │   └── utils/
│   │       ├── __init__.py
│   │       └── validation_utils.py
└── tests/
    └── validation/
        ├── __init__.py
        ├── test_retrieval_validator.py
        ├── test_api_endpoints.py
        └── test_quality_metrics.py
```

**Structure Decision**: Backend service with FastAPI validation endpoints, following the unified system architecture and technology stack compliance requirements from the constitution.

## Phase 0: Research & Resolution

### Research Tasks
1. **Cohere Integration Research**
   - Task: Research best practices for Cohere embedding generation in Python
   - Task: Investigate Cohere embedding models compatible with existing system
   - Task: Document Cohere API rate limits and error handling patterns

2. **Qdrant Vector Search Research**
   - Task: Research Qdrant vector search patterns and parameters
   - Task: Investigate metadata filtering capabilities in Qdrant
   - Task: Document Qdrant connection and authentication methods

3. **Book Content Schema Research**
   - Task: Research existing vector collection schema for book content
   - Task: Document metadata structure used for chapter/section filtering
   - Task: Identify document chunk format and content structure

4. **FastAPI Validation Patterns Research**
   - Task: Research FastAPI patterns for validation endpoints
   - Task: Investigate FastAPI middleware for logging and monitoring
   - Task: Document best practices for API response formatting

### Expected Outcomes
- Complete understanding of Cohere integration requirements
- Clear picture of Qdrant collection schema and metadata structure
- Understanding of existing book content indexing approach
- Best practices for validation API implementation

## Phase 1: Design & Contracts

### Data Model: `data-model.md`

#### Core Entities

**RetrievalQuery**
- `query_text`: String - The user's search query
- `filters`: Object - Optional metadata filters (chapter, section, etc.)
- `top_k`: Integer - Number of results to retrieve (default: 5)
- `query_embedding`: Array<Float> - Vector representation of query

**RetrievalResult**
- `id`: String - Unique identifier for the result
- `content`: String - Retrieved document chunk content
- `source_url`: String - URL of the source document
- `chapter`: String - Chapter title from metadata
- `section`: String - Section title from metadata
- `similarity_score`: Float - Similarity score (0-1)
- `confidence_score`: Float - Confidence in relevance (0-1)
- `retrieval_timestamp`: DateTime - When retrieved

**ValidationRequest**
- `query`: String - Original user query
- `expected_sources`: Array<String> - Expected source documents (optional)
- `filters`: Object - Metadata filters to apply

**ValidationResponse**
- `query`: String - Original query
- `retrieved_results`: Array<RetrievalResult> - Retrieved content chunks
- `semantic_alignment_score`: Float - Score of alignment with intent
- `precision_metric`: Float - Precision of results
- `latency_ms`: Float - Time taken for retrieval
- `validation_passed`: Boolean - Whether validation criteria met

### API Contracts: `/contracts/`

#### Validation Endpoint Contract

**POST** `/api/v1/retrieval/validate`
- **Description**: Validates retrieval quality for a given query
- **Request Body** (`ValidationRequest`):
```json
{
  "query": "What are the key concepts in neural networks?",
  "expected_sources": ["chapter-3-neural-networks.md"],
  "filters": {
    "chapter": "Chapter 3",
    "section": null
  }
}
```

**Response** (`ValidationResponse`):
```json
{
  "query": "What are the key concepts in neural networks?",
  "retrieved_results": [
    {
      "id": "chunk-123",
      "content": "Neural networks consist of layers of interconnected nodes...",
      "source_url": "https://book.example.com/chapter-3-neural-networks",
      "chapter": "Chapter 3",
      "section": "Introduction to Neural Networks",
      "similarity_score": 0.87,
      "confidence_score": 0.92,
      "retrieval_timestamp": "2025-12-16T10:30:00Z"
    }
  ],
  "semantic_alignment_score": 0.91,
  "precision_metric": 0.89,
  "latency_ms": 420.5,
  "validation_passed": true
}
```

**GET** `/api/v1/retrieval/test-interface`
- **Description**: Debug endpoint for manual testing with human-readable output
- **Query Parameters**:
  - `q` (required): Query text
  - `k` (optional): Number of results (default: 5)
  - `chapter` (optional): Chapter filter
  - `section` (optional): Section filter

**Response**: HTML page with formatted results for manual validation

### Quickstart Guide: `quickstart.md`

#### Setting Up Retrieval Validation

1. **Install Dependencies**
```bash
pip install -r requirements-validation.txt
```

2. **Configure Environment**
```bash
export COHERE_API_KEY=your_cohere_api_key
export QDRANT_URL=your_qdrant_cloud_url
export QDRANT_API_KEY=your_qdrant_api_key
export QDRANT_COLLECTION_NAME=book_chunks
```

3. **Run Validation Server**
```bash
uvicorn validation_server:app --host 0.0.0.0 --port 8001
```

4. **Test Basic Retrieval**
```bash
curl -X POST http://localhost:8001/api/v1/retrieval/validate \
  -H "Content-Type: application/json" \
  -d '{"query": "What are neural networks?"}'
```

5. **Access Test Interface**
Visit `http://localhost:8001/api/v1/retrieval/test-interface?q=neural%20networks`

## Phase 2: Implementation Tasks

### Task T001: Implement retrieval query function
- **Objective**: Create function to embed user query and perform vector search in Qdrant
- **Deliverables**:
  - `retrieval_service.py` with `retrieve_similar_chunks()` function
  - Integration with Cohere for query embedding
  - Integration with Qdrant for vector search
- **Acceptance Criteria**:
  - Function successfully embeds query text using Cohere
  - Function performs vector similarity search in Qdrant
  - Function returns top-k most similar document chunks
  - Function includes proper error handling for API failures

### Task T002: Implement filtering and ranking
- **Objective**: Add metadata filtering and result ranking capabilities
- **Deliverables**:
  - Enhanced retrieval function with metadata filters
  - Top-K selection with configurable parameters
  - Confidence scoring for results
- **Acceptance Criteria**:
  - Results can be filtered by chapter, section metadata
  - Top-K parameter controls number of returned results
  - Confidence scores are calculated and returned
  - Filtering respects Qdrant metadata structure

### Task T003: Build retrieval testing interface
- **Objective**: Create CLI script and FastAPI debug endpoint with human-readable output
- **Deliverables**:
  - FastAPI validation server with test interface
  - CLI script for batch validation
  - Human-readable result formatting
- **Acceptance Criteria**:
  - Web interface displays results in readable format
  - CLI script processes multiple queries efficiently
  - Output includes source attribution and similarity scores
  - Interface provides debugging information

### Task T004: Validate retrieval quality
- **Objective**: Implement manual test queries and comparison with source text
- **Deliverables**:
  - Test suite with various query types
  - Comparison logic for retrieved vs. source content
  - Quality metrics calculation
- **Acceptance Criteria**:
  - Manual test queries validate basic functionality
  - Retrieved chunks are compared with source text for relevance
  - Quality metrics (precision, recall, alignment) are computed
  - Validation covers general, chapter-specific, and paragraph-level queries

### Task T005: Log and document results
- **Objective**: Implement comprehensive logging and documentation of validation results
- **Deliverables**:
  - Structured logging for retrieval inputs/outputs
  - Documentation of known limitations and edge cases
  - Performance metrics tracking
- **Acceptance Criteria**:
  - All retrieval operations are logged with inputs/outputs
  - Known limitations are documented with mitigation strategies
  - Performance metrics are tracked and reported
  - Logs enable debugging and monitoring

## Success Criteria Validation

Upon completion, the implementation must satisfy:
- **SC-001**: 90%+ precision rate for general book questions
- **SC-002**: 95%+ semantic alignment with user intent
- **SC-003**: 95%+ success rate for all query types
- **SC-004**: 100% of results with source attribution
- **SC-005**: Under 1.5 seconds retrieval latency (95%+ of queries)
- **SC-006**: 0% hallucinated context introduction
- **SC-007**: 100% of test scenarios allow validation with transparent inspection
- **SC-008**: 90%+ recall rate for relevant content retrieval

## Known Limitations & Edge Cases

- **Cross-chapter matching**: Queries matching content across multiple chapters may return mixed results
- **No relevant content**: Queries with no matching book content need graceful handling
- **Database unavailability**: Temporary Qdrant unavailability should be handled with appropriate fallbacks
- **Malformed queries**: Extremely long or malformed queries should be sanitized before processing

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| | | |
