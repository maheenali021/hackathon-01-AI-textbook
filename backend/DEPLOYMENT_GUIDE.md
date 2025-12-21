# Hugging Face Deployment Guide for RAG Agent

This guide explains how to deploy your RAG Agent backend to Hugging Face Spaces.

## Prerequisites

1. A Hugging Face account (free at https://huggingface.co/)
2. A Google Gemini API key
3. Access to a Qdrant vector database with your book content indexed

## Steps to Deploy

### Option 1: Using Hugging Face Spaces (Recommended)

1. **Fork this repository** to your Hugging Face account
2. **Go to Hugging Face Spaces** and create a new Space
3. **Configure the Space:**
   - Choose "Docker" SDK
   - Select "GPU" or "CPU" based on your needs (CPU is sufficient for this application)
   - Choose visibility (Public/Private)

4. **Set Environment Variables** in the Space settings:
   - `GEMINI_API_KEY`: Your Google Gemini API key
   - `GEMINI_MODEL`: Google Gemini model to use (e.g., gemini-pro)
   - `QDRANT_URL`: URL for your Qdrant vector database
   - `QDRANT_API_KEY`: API key for your Qdrant database
   - `QDRANT_COLLECTION_NAME`: Name of your collection (default: book_content)
   - `RETRIEVAL_API_URL`: URL for your retrieval validation API

5. **Add the required files** to your repository:
   - `app.py` (entry point)
   - `requirements.txt` (dependencies)
   - `Dockerfile` (container configuration)
   - `README.md` (instructions)

6. **Start the Space** - Hugging Face will build and deploy your application

### Option 2: Using Hugging Face Inference API

1. **Upload your model files** to a Hugging Face Model Hub repository
2. **Create a custom inference endpoint** using the app.py as entry point
3. **Configure the environment variables** as mentioned above
4. **Deploy the endpoint**

## Required Environment Variables

For the application to work correctly, you need to set these environment variables:

- `GEMINI_API_KEY` (required): Your Google Gemini API key
- `GEMINI_MODEL` (optional): Google Gemini model to use (default: gemini-pro)
- `QDRANT_URL` (required): URL for your Qdrant vector database
- `QDRANT_API_KEY` (required): API key for your Qdrant database
- `QDRANT_COLLECTION_NAME` (optional): Name of your collection (default: book_content)
- `RETRIEVAL_API_URL` (optional): URL for retrieval validation API (default: http://localhost:8000/api/v1/retrieval/validate)

## API Endpoints

Once deployed, your API will be available at:
- `{your-space-url}/` - Root endpoint
- `{your-space-url}/api/v1/agent/chat` - Chat endpoint
- `{your-space-url}/api/v1/agent/session` - Session management
- `{your-space-url}/api/v1/agent/health` - Health check

## Testing Your Deployment

After deployment, you can test the API using curl or any HTTP client:

```bash
curl -X POST "{your-space-url}/api/v1/agent/chat" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is this book about?",
    "query_type": "general"
  }'
```

## Troubleshooting

1. **Environment Variables**: Make sure all required environment variables are set correctly
2. **Qdrant Connection**: Verify that your Qdrant database is accessible from the Hugging Face environment
3. **API Keys**: Ensure your Google Gemini API key is valid and has sufficient quota
4. **Logs**: Check the Space logs for any error messages

## Scaling Considerations

- The application is designed to handle multiple concurrent requests
- Monitor your Google Gemini API usage to ensure you don't exceed quotas
- Consider implementing caching for frequently asked questions
- Monitor response times and scale resources as needed

## Security Notes

- Never expose your API keys in client-side code
- Use HTTPS for all API communications
- Validate and sanitize all user inputs
- Monitor API usage for potential abuse