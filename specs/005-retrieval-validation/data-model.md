# Data Model: Retrieval Pipeline Validation for Book-Aware RAG System

**Feature**: 005-retrieval-validation
**Date**: 2025-12-16

## Core Entities

### RetrievalQuery
Represents a user's information request that triggers the retrieval validation process.

- **query_text** (String)
  - Description: The user's search query text
  - Validation: Required, non-empty, max 1000 characters
  - Example: "What are the key concepts in neural networks?"

- **filters** (Object)
  - Description: Optional metadata filters to constrain the search
  - Structure:
    - `chapter` (String, optional): Chapter title to filter by
    - `section` (String, optional): Section title to filter by
    - `min_similarity` (Float, optional): Minimum similarity threshold (0.0-1.0)
  - Default: `{}` (no filters)
  - Example: `{"chapter": "Chapter 3", "section": "Introduction"}`

- **top_k** (Integer)
  - Description: Number of results to retrieve
  - Validation: Required, between 1 and 20, default 5
  - Example: 5

- **query_embedding** (Array<Float>)
  - Description: Vector representation of the query text
  - Validation: Array of floats, length must match vector dimension (1024 for Cohere model)
  - Example: `[0.1, 0.2, 0.3, ...]` (1024 values)

### RetrievalResult
Contains a single retrieved document chunk with relevance metrics and source information.

- **id** (String)
  - Description: Unique identifier for the result
  - Validation: Required, UUID format
  - Example: "chunk-12345-abcde"

- **content** (String)
  - Description: Retrieved document chunk content
  - Validation: Required, non-empty, max 5000 characters
  - Example: "Neural networks consist of layers of interconnected nodes..."

- **source_url** (String)
  - Description: URL of the source document
  - Validation: Required, valid URL format
  - Example: "https://book.example.com/chapter-3-neural-networks"

- **chapter** (String)
  - Description: Chapter title from metadata
  - Validation: Required, non-empty
  - Example: "Chapter 3: Neural Networks"

- **section** (String)
  - Description: Section title from metadata
  - Validation: Optional, may be empty
  - Example: "Introduction to Neural Networks"

- **similarity_score** (Float)
  - Description: Similarity score between query and result (0-1)
  - Validation: Required, between 0.0 and 1.0
  - Example: 0.87

- **confidence_score** (Float)
  - Description: Confidence in the relevance of the result (0-1)
  - Validation: Required, between 0.0 and 1.0
  - Example: 0.92

- **retrieval_timestamp** (DateTime)
  - Description: When the result was retrieved
  - Validation: Required, ISO 8601 format
  - Example: "2025-12-16T10:30:00Z"

### ValidationRequest
Represents a validation request to test the retrieval pipeline with specific criteria.

- **query** (String)
  - Description: Original user query to validate
  - Validation: Required, non-empty, max 1000 characters
  - Example: "What are the key concepts in neural networks?"

- **expected_sources** (Array<String>)
  - Description: Expected source documents that should be retrieved (optional)
  - Validation: Optional, array of valid URLs or document IDs
  - Example: `["chapter-3-neural-networks.md", "section-3-1-introduction.md"]`

- **filters** (Object)
  - Description: Metadata filters to apply during validation
  - Structure: Same as filters in RetrievalQuery
  - Validation: Optional object with valid filter properties
  - Example: `{"chapter": "Chapter 3"}`

- **validation_criteria** (Object)
  - Description: Criteria to evaluate the validation results
  - Structure:
    - `min_precision` (Float, optional): Minimum precision required (0.0-1.0)
    - `max_latency_ms` (Float, optional): Maximum allowed latency in milliseconds
    - `require_expected_sources` (Boolean, optional): Whether expected sources must be present
  - Validation: Optional object with valid criteria
  - Example: `{"min_precision": 0.8, "max_latency_ms": 1500}`

### ValidationResponse
Contains the results of a validation request with quality metrics and evaluation.

- **query** (String)
  - Description: Original query that was validated
  - Validation: Required, non-empty
  - Example: "What are the key concepts in neural networks?"

- **retrieved_results** (Array<RetrievalResult>)
  - Description: Retrieved content chunks from the validation
  - Validation: Required, array of valid RetrievalResult objects
  - Example: `[{"id": "chunk-123", "content": "...", ...}, ...]`

- **semantic_alignment_score** (Float)
  - Description: Score measuring how well results align with query intent (0-1)
  - Validation: Required, between 0.0 and 1.0
  - Example: 0.91

- **precision_metric** (Float)
  - Description: Precision of the retrieved results (0-1)
  - Validation: Required, between 0.0 and 1.0
  - Example: 0.89

- **recall_metric** (Float)
  - Description: Recall of the retrieved results (0-1)
  - Validation: Optional, between 0.0 and 1.0
  - Example: 0.85

- **latency_ms** (Float)
  - Description: Time taken for retrieval in milliseconds
  - Validation: Required, positive number
  - Example: 420.5

- **validation_passed** (Boolean)
  - Description: Whether the validation met all specified criteria
  - Validation: Required, boolean value
  - Example: true

- **validation_details** (Object)
  - Description: Detailed information about the validation results
  - Structure:
    - `expected_sources_found` (Integer): Count of expected sources that were found
    - `total_expected_sources` (Integer): Total count of expected sources
    - `sources_match_ratio` (Float): Ratio of expected sources found (0-1)
    - `criteria_evaluations` (Object): Individual evaluation results for each criterion
  - Validation: Optional object with validation details
  - Example: `{"expected_sources_found": 2, "total_expected_sources": 2, "sources_match_ratio": 1.0, "criteria_evaluations": {"precision": true, "latency": true}}`

### ValidationMetrics
Aggregated metrics for evaluating the overall performance of the retrieval system.

- **validation_run_id** (String)
  - Description: Unique identifier for the validation run
  - Validation: Required, UUID format
  - Example: "validation-run-12345-abcde"

- **timestamp** (DateTime)
  - Description: When the validation was performed
  - Validation: Required, ISO 8601 format
  - Example: "2025-12-16T10:30:00Z"

- **query_types_tested** (Array<String>)
  - Description: Types of queries tested during validation
  - Validation: Required, non-empty array of query type strings
  - Example: `["general", "chapter-specific", "paragraph-level"]`

- **overall_precision** (Float)
  - Description: Average precision across all validation queries (0-1)
  - Validation: Required, between 0.0 and 1.0
  - Example: 0.88

- **overall_recall** (Float)
  - Description: Average recall across all validation queries (0-1)
  - Validation: Required, between 0.0 and 1.0
  - Example: 0.82

- **avg_latency_ms** (Float)
  - Description: Average latency across all validation queries
  - Validation: Required, positive number
  - Example: 380.2

- **success_rate** (Float)
  - Description: Percentage of queries that passed validation (0-1)
  - Validation: Required, between 0.0 and 1.0
  - Example: 0.95

- **total_queries_tested** (Integer)
  - Description: Total number of queries tested in the validation run
  - Validation: Required, positive integer
  - Example: 100

- **queries_passed** (Integer)
  - Description: Number of queries that passed validation
  - Validation: Required, positive integer, ≤ total_queries_tested
  - Example: 95

## Relationships

### Between Entities

1. **ValidationRequest** → **ValidationResponse**
   - One-to-one relationship
   - A validation request produces exactly one validation response

2. **ValidationResponse** → **RetrievalResult**
   - One-to-many relationship
   - A validation response contains multiple retrieval results

3. **ValidationMetrics** → **ValidationResponse**
   - One-to-many relationship
   - Validation metrics aggregate results from multiple validation responses

4. **RetrievalQuery** → **RetrievalResult**
   - One-to-many relationship
   - A retrieval query can return multiple results (top-k)

## Validation Rules

### Business Logic Validations

1. **RetrievalQuery Validation**:
   - Query text must be between 5 and 1000 characters
   - Top-k value must be between 1 and 20
   - Query embedding must have the correct vector dimension (1024)

2. **RetrievalResult Validation**:
   - Similarity and confidence scores must be between 0.0 and 1.0
   - Content must not exceed 5000 characters
   - Source URL must be a valid URL format

3. **ValidationRequest Validation**:
   - At least one of query or expected_sources must be provided
   - Validation criteria values must be within valid ranges

4. **ValidationResponse Validation**:
   - Validation passed flag must match the actual criteria evaluation
   - Precision and recall metrics must be between 0.0 and 1.0
   - Latency must be a positive value

### Data Integrity Validations

1. **Timestamp Consistency**:
   - All timestamps must be in ISO 8601 format
   - Retrieved timestamps must be within reasonable bounds

2. **Identifier Uniqueness**:
   - Result IDs must be unique within a single validation response
   - Validation run IDs must be globally unique

3. **Metric Calculations**:
   - Precision and recall values must be mathematically consistent
   - Success rate must equal queries_passed / total_queries_tested