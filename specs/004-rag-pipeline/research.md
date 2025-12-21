# Research Summary: RAG Pipeline Implementation

## Decision: GitHub Pages Deployment
**Rationale**: The book website needs to be deployed to GitHub Pages for content extraction. Based on the project structure, there's already a frontend directory with Docusaurus setup.
**Implementation**: Use the existing Docusaurus site in the `frontend/` directory and deploy it to GitHub Pages following standard Docusaurus deployment practices.

## Decision: Qdrant Cloud Configuration
**Rationale**: Qdrant Cloud provides a managed vector database service suitable for the project's needs without requiring self-hosted infrastructure.
**Implementation**:
- Use Qdrant Cloud free tier for development and testing
- Collection schema: vectors with dimension size matching Cohere embedding output (typically 1024 for multilingual models)
- Payload structure: include source URL, chapter, section, and original content

## Decision: Cohere API Integration
**Rationale**: Cohere provides high-quality embedding models that work well for technical content like robotics textbooks.
**Implementation**:
- Use Cohere's embed-multilingual-v3.0 model for broad language support
- Handle rate limits with exponential backoff retry strategy
- Implement batch processing to optimize API calls

## Decision: Content Extraction Approach
**Rationale**: The book content needs to be extracted from the deployed website in a structured way that preserves semantic meaning.
**Implementation**:
- Use Python requests and BeautifulSoup for web scraping
- Focus on extracting main content areas (typically in `<main>`, `<article>`, or content-specific divs)
- Preserve hierarchical structure (chapters, sections) in metadata
- Filter out navigation, headers, footers, and other non-content elements

## Decision: Content Chunking Strategy
**Rationale**: Content needs to be divided into semantically meaningful chunks that preserve context while being small enough for embedding models.
**Implementation**:
- Use section-based chunking (h1, h2, h3 tags as boundaries)
- Implement overlap strategy (10-20% overlap) to preserve context across chunks
- Set maximum chunk size to ~1000 tokens (approximately 500-800 words)
- Preserve code blocks and examples within single chunks when possible

## Alternatives Considered

### For Vector Database:
- **Pinecone**: More expensive than Qdrant, good alternative but Qdrant Cloud is sufficient
- **Weaviate**: Self-hosted option, but Qdrant Cloud provides managed service
- **Supabase Vector**: Good alternative but Qdrant has better embedding integration

### For Embedding Models:
- **OpenAI embeddings**: More expensive, Cohere provides good quality for technical content
- **Hugging Face models**: Would require self-hosting, Cohere API is more straightforward
- **Google embeddings**: Good alternative but Cohere has better documentation

### For Content Extraction:
- **Selenium**: More complex but handles JavaScript-rendered content
- **Scrapy**: More powerful but overkill for static Docusaurus site
- **Direct file access**: Would require access to source files, web scraping is more general

### For Chunking:
- **Token-based**: More precise but requires tokenization library
- **Character-based**: Simpler but may break semantic boundaries
- **Semantic-aware**: More complex but preserves meaning better

## Technical Specifications

### Qdrant Collection Schema
```json
{
  "collection_name": "book_chunks",
  "vector_size": 1024,
  "distance": "Cosine",
  "payload_schema": {
    "chunk_id": "keyword",
    "content": "text",
    "source_url": "keyword",
    "chapter": "keyword",
    "section": "keyword",
    "word_count": "integer",
    "chunk_index": "integer"
  }
}
```

### Cohere Embedding Parameters
- Model: `embed-multilingual-v3.0`
- Input type: `search_document` for content chunks
- Dimension: 1024 (default for this model)

### Content Extraction Strategy
- Target CSS selectors: `main.main-wrapper`, `article`, `.markdown`, `.doc-content`
- Exclude selectors: `.navbar`, `.footer`, `.menu`, `.pagination`
- Preserve formatting: Convert HTML to clean text while maintaining structure

## Architecture Decisions

### Pipeline Orchestration
The pipeline will be implemented as a Python script with distinct modules:
1. `extractor.py` - Content extraction from website
2. `chunker.py` - Content chunking with semantic boundaries
3. `embedder.py` - Cohere API integration for embeddings
4. `storage.py` - Qdrant Cloud integration
5. `pipeline.py` - Main orchestration with error handling

### Error Handling Strategy
- Retry with exponential backoff for API failures
- Graceful degradation when individual chunks fail
- Comprehensive logging for debugging
- Checkpoint system to resume from failures

### Validation Approach
- Content extraction: Verify text content > 100 characters per page
- Chunking: Ensure chunks are 100-800 words, preserve section boundaries
- Embeddings: Validate vector dimensions match expected size
- Storage: Confirm vectors are retrievable with semantic search