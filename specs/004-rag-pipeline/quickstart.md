# Quickstart Guide: RAG Pipeline

## Overview
This guide will help you set up and run the RAG (Retrieval-Augmented Generation) pipeline for the AI Robotics textbook. The pipeline extracts content from the deployed book website, chunks it semantically, generates embeddings using Cohere, and stores them in Qdrant Cloud for semantic search.

## Prerequisites

### Required Accounts & Services
1. **GitHub Account**: For deploying the book website to GitHub Pages
2. **Cohere API Key**: For generating text embeddings
3. **Qdrant Cloud Account**: For vector storage and retrieval

### Local Environment
- Python 3.8 or higher
- Git
- Access to the project repository

## Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd hackathon-01-AI-textbook
git checkout 004-rag-pipeline
```

### 2. Set Up Python Environment
```bash
cd backend  # or wherever the pipeline code will be located
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### 3. Install Additional Dependencies
```bash
pip install cohere qdrant-client beautifulsoup4 requests python-dotenv
```

### 4. Configure Environment Variables
Create a `.env` file in the backend directory:
```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_URL=your_qdrant_cluster_url
GITHUB_PAGES_URL=https://your-username.github.io/repository-name
```

## Running the Pipeline

### 1. Verify Website Deployment
First, ensure the book website is deployed and accessible:
```bash
curl -I https://your-username.github.io/repository-name
```

### 2. Run the Complete Pipeline
Execute the main pipeline script:
```bash
python run_rag_pipeline.py --website-url "https://your-username.github.io/repository-name"
```

### 3. Alternative: Run Pipeline Stages Separately

#### Stage 1: Content Extraction
```bash
python -m scripts.extract_content --url "https://your-username.github.io/repository-name" --output extracted_content.json
```

#### Stage 2: Content Chunking
```bash
python -m scripts.chunk_content --input extracted_content.json --output chunked_content.json
```

#### Stage 3: Embedding Generation
```bash
python -m scripts.generate_embeddings --input chunked_content.json --output embeddings.json
```

#### Stage 4: Vector Storage
```bash
python -m scripts.store_vectors --input embeddings.json
```

## Testing the Pipeline

### 1. Verify Content Extraction
Check that content was properly extracted:
```bash
python -m scripts.validate_extraction --input extracted_content.json
```

### 2. Test Vector Storage
Verify vectors are stored and retrievable:
```bash
python -m scripts.test_storage --query "sample query about robotics"
```

### 3. End-to-End Test
Run a complete test of the pipeline:
```bash
python -m tests.test_pipeline
```

## API Usage

Once the pipeline is complete, you can use the search API:

### Query for Similar Content
```bash
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the principles of humanoid robotics?",
    "top_k": 5
  }'
```

### Check Pipeline Status
```bash
curl http://localhost:8000/pipeline/status
```

## Expected Output

After running the complete pipeline, you should have:

1. **Deployment URL**: A working GitHub Pages site with the book content
2. **Embedding Generation Script**: `generate_embeddings.py` that can process content chunks
3. **Qdrant Collection Schema**: Configured collection named `book_chunks` with appropriate vector dimensions
4. **Sample Query Verification Output**: Results showing successful semantic search

## Troubleshooting

### Common Issues

#### API Rate Limits
- **Problem**: Cohere or Qdrant API rate limits exceeded
- **Solution**: Implement exponential backoff in your requests, or upgrade your API plan

#### Content Extraction Failures
- **Problem**: Unable to extract content from the website
- **Solution**: Check website structure, update CSS selectors, ensure the site is publicly accessible

#### Embedding Dimension Mismatch
- **Problem**: Stored vectors have wrong dimensions
- **Solution**: Verify the embedding model matches the Qdrant collection schema

### Logging
Check the pipeline logs for detailed error information:
```bash
tail -f logs/pipeline.log
```

## Next Steps

1. **Production Deployment**: Deploy the search API to a production environment
2. **Frontend Integration**: Connect the search API to the book website's UI
3. **Monitoring**: Set up monitoring for pipeline performance and errors
4. **Optimization**: Fine-tune chunking strategy and embedding parameters based on search quality