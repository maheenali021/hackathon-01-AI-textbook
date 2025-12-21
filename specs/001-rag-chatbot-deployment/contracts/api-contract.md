# API Contract: RAG Chatbot Backend

## Overview
This document defines the API contract between the Docusaurus frontend and the FastAPI RAG backend for the AI robotics textbook chatbot.

## Base URL
`https://your-backend-domain.com/api/v1`

## Common Headers
- `Content-Type: application/json`
- `Accept: application/json`

## Authentication
API keys should be passed in the Authorization header:
- `Authorization: Bearer {api_key}` (if required)

## Endpoints

### POST /chat
Submit a query to the RAG agent and receive a response.

#### Request
```json
{
  "query_text": "What are the key principles of reinforcement learning?",
  "context_mode": "retrieval_pipeline",
  "user_context": "Optional context when context_mode is 'user_provided'",
  "filters": {
    "chapter": "Chapter 3: Reinforcement Learning",
    "section": "3.1 Basic Concepts"
  },
  "conversation_id": "optional-session-id",
  "thread_id": "optional-openai-thread-id"
}
```

#### Response (200 OK)
```json
{
  "response_text": "Reinforcement learning is a type of machine learning where an agent learns to make decisions by taking actions in an environment to maximize cumulative reward...",
  "sources": [
    {
      "content_id": "chunk_12345",
      "source_url": "https://book.example.com/chapter3",
      "chapter": "Chapter 3: Reinforcement Learning",
      "section": "3.1 Basic Concepts"
    }
  ],
  "confidence_score": 0.85,
  "grounding_validation": {
    "is_fully_grounded": true,
    "unsupported_claims": []
  },
  "conversation_id": "session-abc123",
  "timestamp": "2023-12-16T10:30:00.000Z"
}
```

#### Error Response (400 Bad Request)
```json
{
  "error": "VALIDATION_ERROR",
  "message": "Query text is required and must be between 1 and 1000 characters",
  "details": {
    "field": "query_text"
  }
}
```

### POST /sessions
Create a new conversation session.

#### Request
```json
{
  "context_mode": "retrieval_pipeline",
  "user_id": "optional-user-id",
  "initial_filters": {
    "chapter": "Chapter 3: Reinforcement Learning"
  }
}
```

#### Response (200 OK)
```json
{
  "session_id": "session-abc123",
  "openai_thread_id": "thread-def456",
  "created_at": "2023-12-16T10:30:00.000Z",
  "last_interaction": "2023-12-16T10:30:00.000Z",
  "context_mode": "retrieval_pipeline",
  "user_id": "user123",
  "active_filters": {
    "chapter": "Chapter 3: Reinforcement Learning"
  },
  "conversation_history": []
}
```

### GET /sessions/{session_id}
Get details of a conversation session.

#### Response (200 OK)
```json
{
  "session_id": "session-abc123",
  "openai_thread_id": "thread-def456",
  "created_at": "2023-12-16T10:30:00.000Z",
  "last_interaction": "2023-12-16T10:35:00.000Z",
  "context_mode": "retrieval_pipeline",
  "user_id": "user123",
  "active_filters": {
    "chapter": "Chapter 3: Reinforcement Learning"
  },
  "conversation_history": [
    {
      "role": "user",
      "content": "What is reinforcement learning?",
      "timestamp": "2023-12-16T10:30:00.000Z"
    },
    {
      "role": "assistant",
      "content": "Reinforcement learning is a type of machine learning...",
      "timestamp": "2023-12-16T10:30:05.000Z"
    }
  ]
}
```

### POST /sessions/{session_id}/reset
Reset a conversation session.

#### Request
```json
{
  "preserve_context_mode": true,
  "new_filters": {
    "chapter": "Chapter 4: Neural Networks"
  }
}
```

#### Response (200 OK)
```json
{
  "session_id": "session-abc123",
  "openai_thread_id": "thread-def456",
  "created_at": "2023-12-16T10:30:00.000Z",
  "last_interaction": "2023-12-16T10:40:00.000Z",
  "context_mode": "retrieval_pipeline",
  "user_id": "user123",
  "active_filters": {
    "chapter": "Chapter 4: Neural Networks"
  },
  "conversation_history": []
}
```

### GET /health
Health check endpoint.

#### Response (200 OK)
```json
{
  "status": "healthy",
  "service": "rag-agent-backend",
  "version": "1.0.0",
  "timestamp": "2023-12-16T10:30:00.000Z"
}
```

## Error Codes

| Code | Description |
|------|-------------|
| 400 | Bad Request - Validation error in request parameters |
| 401 | Unauthorized - Invalid or missing authentication |
| 404 | Not Found - Requested resource does not exist |
| 429 | Too Many Requests - Rate limit exceeded |
| 500 | Internal Server Error - Server-side error |
| 503 | Service Unavailable - Downstream service unavailable |

## Data Models

### AgentRequest
- `query_text`: string (1-1000 characters) - The user's question or query
- `context_mode`: enum ["retrieval_pipeline", "user_provided"] - Whether to use the retrieval pipeline or user-provided context
- `user_context`: string (0-10000 characters, optional) - Text provided by user for context-only mode
- `filters`: object (optional) - Metadata filters (e.g., chapter, section) for retrieval pipeline
- `conversation_id`: string (optional) - ID for maintaining conversation state
- `thread_id`: string (optional) - OpenAI thread ID for assistant memory

### AgentResponse
- `response_text`: string (max 10000 characters) - The agent's response to the user
- `sources`: array of SourceAttribution - List of sources used in the response
- `confidence_score`: number (0.0-1.0) - Agent's confidence in the response
- `grounding_validation`: GroundingValidation - Information about response grounding
- `conversation_id`: string - ID for the conversation thread
- `timestamp`: string (ISO format) - When response was generated

### SourceAttribution
- `content_id`: string - ID of the content chunk used
- `source_url`: string - URL of the source
- `chapter`: string - Chapter where content was found
- `section`: string (optional) - Section where content was found

### GroundingValidation
- `is_fully_grounded`: boolean - Whether response is fully grounded in retrieved content
- `unsupported_claims`: array of strings - Any unsupported claims in the response