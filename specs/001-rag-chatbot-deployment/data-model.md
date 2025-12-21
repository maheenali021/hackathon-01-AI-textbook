# Data Model: RAG Chatbot Integration

## Core Entities

### Book Content
- **ContentChunk**
  - id: string (unique identifier)
  - text: string (the actual content)
  - chapter: string (chapter title/identifier)
  - section: string (optional section identifier)
  - source_url: string (URL to the original content location)
  - embedding: float[] (vector representation for similarity search)
  - metadata: object (additional attributes like page numbers, tags)

### User Interaction
- **UserQuery**
  - id: string (unique identifier)
  - query_text: string (the user's question)
  - context_mode: enum ("retrieval_pipeline" | "user_provided")
  - user_context: string (optional user-provided context)
  - filters: object (metadata filters like chapter, section)
  - conversation_id: string (session identifier)
  - timestamp: string (ISO date format)

- **ChatbotResponse**
  - id: string (unique identifier)
  - response_text: string (the AI-generated response)
  - sources: SourceAttribution[] (list of sources used)
  - confidence_score: float (0-1 confidence in response accuracy)
  - grounding_validation: GroundingValidation (validation information)
  - conversation_id: string (session identifier)
  - timestamp: string (ISO date format)

- **SourceAttribution**
  - content_id: string (ID of the content chunk used)
  - source_url: string (URL of the source)
  - chapter: string (chapter where content was found)
  - section: string (optional section where content was found)

- **GroundingValidation**
  - is_fully_grounded: boolean (whether response is fully grounded in retrieved content)
  - unsupported_claims: string[] (any unsupported claims in the response)

### Conversation Management
- **ConversationSession**
  - session_id: string (unique session identifier)
  - openai_thread_id: string (OpenAI's thread ID for assistant memory)
  - created_at: string (ISO date format)
  - last_interaction: string (ISO date format)
  - context_mode: enum ("retrieval_pipeline" | "user_provided")
  - user_id: string (optional user identifier)
  - active_filters: object (active filters applied to the session)
  - conversation_history: ConversationMessage[] (history of exchanges)

- **ConversationMessage**
  - role: enum ("user" | "assistant")
  - content: string (the message content)
  - timestamp: string (ISO date format)

## API Models

### Request Models
- **AgentRequest**
  - query_text: string (the user's question or query)
  - context_mode: enum ("retrieval_pipeline" | "user_provided")
  - user_context: string (optional text for context-only mode)
  - filters: object (metadata filters for retrieval pipeline)
  - conversation_id: string (optional ID for maintaining conversation state)
  - thread_id: string (optional OpenAI thread ID for assistant memory)

- **CreateSessionRequest**
  - context_mode: enum ("retrieval_pipeline" | "user_provided")
  - user_id: string (optional user ID)
  - initial_filters: object (optional initial filters to apply)

- **ResetSessionRequest**
  - preserve_context_mode: boolean (whether to preserve current context mode after reset)
  - new_filters: object (optional new filters to apply after reset)

### Response Models
- **AgentResponse**
  - response_text: string (the agent's response to the user)
  - sources: SourceAttribution[] (list of sources used in the response)
  - confidence_score: float (agent's confidence in the response, 0-1)
  - grounding_validation: GroundingValidation (information about response grounding)
  - conversation_id: string (ID for the conversation thread)
  - timestamp: string (when response was generated)

- **ConversationSession**
  - session_id: string (unique identifier for the conversation session)
  - openai_thread_id: string (OpenAI's thread ID for assistant memory)
  - created_at: string (timestamp when session was created)
  - last_interaction: string (timestamp of last interaction)
  - context_mode: enum ("retrieval_pipeline" | "user_provided")
  - user_id: string (optional ID of the user)
  - active_filters: object (active filters applied to the session)
  - conversation_history: ConversationMessage[] (history of exchanges)

- **ErrorResponse**
  - error: string (error code)
  - message: string (human-readable error message)
  - details: object (optional additional error details)

- **HealthResponse**
  - status: string (health status)
  - service: string (service name)
  - version: string (service version)
  - timestamp: string (when the health check was performed)

## Vector Database Schema

### Qdrant Collection: book_content
- Point ID: string (unique identifier for each chunk)
- Vector: float[] (embedding vector)
- Payload:
  - text: string (the content text)
  - chapter: string (chapter identifier)
  - section: string (section identifier)
  - source_url: string (URL to original location)
  - content_id: string (content chunk ID)

## Relationships

- One ConversationSession contains many ConversationMessage entries
- One ChatbotResponse contains many SourceAttribution entries
- Many UserQuery entries link to one ConversationSession
- Many ContentChunk entries are searched during retrieval
- One GroundingValidation belongs to one ChatbotResponse