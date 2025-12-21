# Quickstart Guide: RAG Chatbot Integration

## Prerequisites

- Node.js 20+ for Docusaurus frontend
- Python 3.11+ for FastAPI backend
- OpenAI API key
- Qdrant vector database (local or cloud)
- Git for version control

## Environment Setup

### Backend Setup
1. Navigate to the backend directory:
   ```bash
   cd backend
   ```

2. Create a virtual environment and install dependencies:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   pip install -r rag_agent/requirements.txt
   ```

3. Set up environment variables in `.env`:
   ```env
   OPENAI_API_KEY=your_openai_api_key
   RETRIEVAL_API_BASE_URL=http://localhost:6333  # Qdrant endpoint
   RETRIEVAL_API_KEY=your_qdrant_api_key  # if applicable
   LOG_LEVEL=INFO
   ALLOWED_ORIGINS=http://localhost:3000,https://yourdomain.github.io
   ```

### Frontend Setup
1. Navigate to the Docusaurus directory:
   ```bash
   cd frontend/docusaurus
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Set up environment variables in `.env` (if needed):
   ```env
   REACT_APP_BACKEND_URL=http://localhost:8001  # Backend API URL
   ```

## Running the System Locally

### Backend (FastAPI)
1. Start the backend server:
   ```bash
   cd backend
   source venv/bin/activate
   python -m uvicorn rag_agent.main:app --host 0.0.0.0 --port 8001
   ```

2. The API will be available at `http://localhost:8001`
3. API documentation available at `http://localhost:8001/docs`

### Frontend (Docusaurus)
1. Start the development server:
   ```bash
   cd frontend/docusaurus
   npm start
   ```

2. The site will be available at `http://localhost:3000`

## Key Endpoints

### Backend API
- `GET /api/v1/health` - Health check
- `POST /api/v1/chat` - Chat with the RAG agent
- `POST /api/v1/sessions` - Create a new conversation session
- `GET /api/v1/sessions/{session_id}` - Get session details
- `POST /api/v1/sessions/{session_id}/reset` - Reset a session

### Example Chat Request
```json
{
  "query_text": "What are the key principles of reinforcement learning?",
  "context_mode": "retrieval_pipeline",
  "filters": {
    "chapter": "Chapter 3: Reinforcement Learning"
  },
  "conversation_id": "optional-session-id"
}
```

## Deployment

### Backend Deployment
The backend can be deployed to any cloud platform that supports Python applications (AWS, GCP, Azure, etc.). Containerization with Docker is recommended.

### Frontend Deployment
The Docusaurus frontend is designed for GitHub Pages deployment:

1. Build the static site:
   ```bash
   npm run build
   ```

2. Deploy to GitHub Pages using GitHub Actions or manual deployment

## Configuration Options

### Environment Variables
- `OPENAI_API_KEY`: Required OpenAI API key
- `RETRIEVAL_API_BASE_URL`: URL to Qdrant instance
- `RETRIEVAL_API_KEY`: Qdrant API key (if authentication enabled)
- `OPENAI_MODEL`: OpenAI model to use (default: gpt-4-turbo-preview)
- `REQUEST_TIMEOUT`: HTTP request timeout in seconds
- `MAX_RETRIES`: Maximum number of retry attempts for failed requests
- `ALLOWED_ORIGINS`: Comma-separated list of allowed origins for CORS

### API Configuration
The system supports different context modes:
- `retrieval_pipeline`: Use the RAG pipeline to answer questions
- `user_provided`: Use user-provided context to answer questions

## Testing the Integration

1. Start both backend and frontend
2. Access the Docusaurus site
3. Use the embedded chatbot to ask questions about the book
4. Verify that responses include proper source attribution
5. Test different query types:
   - General book questions
   - Chapter-specific queries
   - User-selected text–constrained questions

## Troubleshooting

### Common Issues
- **API Key Issues**: Verify all required API keys are set correctly
- **CORS Errors**: Ensure ALLOWED_ORIGINS includes your frontend domain
- **Vector Database Connection**: Verify Qdrant is running and accessible
- **Slow Responses**: Check embedding model performance and vector database indexing

### Health Checks
- Backend: `GET /api/v1/health`
- Frontend: Check browser console for errors
- Vector database: Verify connectivity to Qdrant instance