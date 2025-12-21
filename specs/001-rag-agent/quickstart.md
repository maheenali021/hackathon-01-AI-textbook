# Quickstart: RAG Agent Backend – OpenAI Agents SDK + FastAPI

**Feature**: RAG Agent Backend – OpenAI Agents SDK + FastAPI
**Date**: 2025-12-16
**Branch**: 001-rag-agent

## Overview

This quickstart guide helps you set up and run the RAG Agent Backend that connects OpenAI Agents SDK with the validated retrieval pipeline to answer questions about the AI robotics textbook.

## Prerequisites

- Python 3.9+
- OpenAI API key
- Access to the validated retrieval pipeline from Spec-2
- (Optional) MCP server for accessing latest OpenAI SDK documentation

## Setup

### 1. Clone and Navigate to Backend Directory

```bash
cd backend
```

### 2. Create and Activate Virtual Environment

```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies

```bash
pip install -r requirements.txt
```

For the RAG Agent specifically:
```bash
pip install openai fastapi uvicorn pydantic python-dotenv
```

### 4. Set Up Environment Variables

Create a `.env` file in the backend directory:

```env
OPENAI_API_KEY=your_openai_api_key_here
OPENAI_ASSISTANT_ID=your_assistant_id_if_pre_existing

# Configuration for connecting to retrieval pipeline
RETRIEVAL_API_BASE_URL=http://localhost:8000/api/v1/retrieval
COHERE_API_KEY=your_cohere_api_key
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_URL=your_qdrant_url

# Server configuration
HOST=0.0.0.0
PORT=8001
LOG_LEVEL=info
```

## Running the Service

### 1. Start the Agent Service

```bash
cd backend/rag_agent
uvicorn main:app --host 0.0.0.0 --port 8001 --reload
```

The API will be available at `http://localhost:8001`.

### 2. Verify the Service is Running

```bash
curl http://localhost:8001/health
```

## API Usage Examples

### 1. Send a Message to the Agent

```bash
curl -X POST http://localhost:8001/chat \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer your-api-key" \
  -d '{
    "query_text": "What are the key principles of reinforcement learning?",
    "context_mode": "retrieval_pipeline",
    "filters": {
      "chapter": "Chapter 3"
    }
  }'
```

### 2. Use User-Provided Context Mode

```bash
curl -X POST http://localhost:8001/chat \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer your-api-key" \
  -d '{
    "query_text": "Based on this text, how does Q-learning work?",
    "context_mode": "user_provided",
    "user_context": "Q-learning is a model-free reinforcement learning algorithm. It learns the value of an action in a particular state."
  }'
```

### 3. Create a New Conversation Session

```bash
curl -X POST http://localhost:8001/chat/session \
  -H "Authorization: Bearer your-api-key"
```

## Key Components

### Agent Structure
```
backend/rag_agent/
├── main.py                 # FastAPI app entry point
├── agents/
│   ├── book_agent.py       # OpenAI Agent implementation
│   └── tools/
│       ├── retrieval_tool.py   # Tool for calling retrieval pipeline
│       └── context_tool.py     # Tool for user-provided context mode
├── api/
│   ├── chat_endpoints.py     # FastAPI chat endpoints
│   └── models/
│       ├── request_models.py   # Request schemas
│       └── response_models.py  # Response schemas
└── services/
    ├── agent_service.py      # Agent orchestration service
    └── validation_service.py # Response validation service
```

### Core Files to Understand

1. **`agents/book_agent.py`** - The main agent implementation using OpenAI Assistants API
2. **`agents/tools/retrieval_tool.py`** - Custom tool that connects to the retrieval pipeline
3. **`api/chat_endpoints.py`** - FastAPI endpoints for chat interactions
4. **`services/validation_service.py`** - Service that ensures responses are grounded in content

## Development Workflow

### 1. Testing the Agent

Run the tests:
```bash
cd backend/rag_agent
python -m pytest tests/ -v
```

### 2. API Documentation

View the automatically generated API documentation at:
- `http://localhost:8001/docs` (Swagger UI)
- `http://localhost:8001/redoc` (ReDoc)

### 3. MCP Server Integration

During development, use the MCP server to access the latest OpenAI Agents SDK documentation:
```bash
# The MCP server can be used to validate SDK usage against latest documentation
# Implementation details will depend on your MCP server setup
```

## Configuration Options

### Environment Variables

- `OPENAI_API_KEY`: Your OpenAI API key (required)
- `OPENAI_ASSISTANT_ID`: Pre-existing assistant ID (optional, new one will be created if not provided)
- `RETRIEVAL_API_BASE_URL`: Base URL for the retrieval pipeline API
- `HOST`: Host to bind the service to (default: 0.0.0.0)
- `PORT`: Port to run the service on (default: 8001)
- `LOG_LEVEL`: Logging level (default: info)

## Troubleshooting

### Common Issues

1. **OpenAI API Connection Errors**: Verify your API key is correct and has sufficient permissions
2. **Retrieval Pipeline Connection Errors**: Ensure the retrieval pipeline from Spec-2 is running and accessible
3. **Hallucination Prevention**: If the agent is generating content not in the source, check the validation service configuration

### Enable Debug Logging

Add `LOG_LEVEL=debug` to your `.env` file to get more detailed logs.

## Next Steps

1. Review the API contracts in `specs/001-rag-agent/contracts/agent-api.yaml`
2. Examine the data models in `specs/001-rag-agent/data-model.md`
3. Look at the research findings in `specs/001-rag-agent/research.md`
4. Run the full test suite to ensure everything works correctly