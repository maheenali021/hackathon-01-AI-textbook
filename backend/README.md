# RAG Agent API for AI Robotics Textbook

This is a RAG (Retrieval-Augmented Generation) agent API that answers questions about the AI Robotics textbook using Google Gemini and Qdrant vector database.

## API Endpoints

- `GET /` - Root endpoint with API information
- `POST /api/v1/agent/chat` - Chat endpoint for querying the RAG agent
- `POST /api/v1/agent/session` - Create conversation session
- `GET /api/v1/agent/session/{session_id}` - Get conversation session details
- `DELETE /api/v1/agent/session/{session_id}` - Clear conversation session
- `GET /api/v1/agent/health` - Health check endpoint

## Environment Variables

The following environment variables need to be set:

- `GEMINI_API_KEY` - Your Google Gemini API key
- `GEMINI_MODEL` - Google Gemini model to use (default: gemini-pro)
- `QDRANT_URL` - URL for Qdrant vector database
- `QDRANT_API_KEY` - API key for Qdrant database
- `QDRANT_COLLECTION_NAME` - Name of the collection in Qdrant (default: book_content)
- `RETRIEVAL_API_URL` - URL for the retrieval validation API

## Usage

1. Set the required environment variables
2. Start the server: `uvicorn app:app --reload`
3. Send POST requests to `/api/v1/agent/chat` with JSON payload:
   ```json
   {
     "query": "Your question here",
     "query_type": "general",
     "chapter_filter": null,
     "user_context": null,
     "conversation_id": null
   }
   ```

## Deployment

This application can be deployed to various platforms including:
- Hugging Face Spaces (using the app.py entry point)
- Cloud platforms like AWS, GCP, Azure
- Containerized environments with Docker