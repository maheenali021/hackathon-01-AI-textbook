# Data Model: RAG Agent Backend – OpenAI Agents SDK + FastAPI

**Feature**: RAG Agent Backend – OpenAI Agents SDK + FastAPI
**Date**: 2025-12-16
**Branch**: 001-rag-agent

## Overview

This document defines the data models for the RAG Agent Backend that connects OpenAI Agents SDK with the validated retrieval pipeline to answer questions about the AI robotics textbook.

## Entities

### Agent Request
**Description**: User input to the agent including query text and optional context/filters

**Fields**:
- `query_text` (string, required): The user's question or query
- `context_mode` (enum: "retrieval_pipeline" | "user_provided", default: "retrieval_pipeline"): Whether to use the retrieval pipeline or user-provided context
- `user_context` (string, optional): Text provided by user for context-only mode
- `filters` (object, optional): Metadata filters (e.g., chapter, section) for retrieval pipeline
- `conversation_id` (string, optional): ID for maintaining conversation state
- `thread_id` (string, optional): OpenAI thread ID for assistant memory

**Validation**:
- `query_text` must be 1-1000 characters
- If `context_mode` is "user_provided", `user_context` is required
- `filters` must follow metadata schema from retrieval pipeline

### Retrieved Content
**Description**: Book content chunks retrieved from the validated pipeline with metadata and source attribution

**Fields**:
- `content_chunks` (array of objects, required): Retrieved content segments
- `content_chunks[].id` (string): Unique identifier for the chunk
- `content_chunks[].text` (string): The actual content text
- `content_chunks[].source_url` (string): URL of the source document
- `content_chunks[].chapter` (string): Chapter title
- `content_chunks[].section` (string, optional): Section title
- `content_chunks[].similarity_score` (number): Relevance score (0-1)
- `retrieval_metadata` (object): Additional retrieval information
- `retrieval_metadata.query` (string): Original query
- `retrieval_metadata.timestamp` (string): When retrieval occurred
- `retrieval_metadata.source_count` (number): Number of sources retrieved

**Validation**:
- `content_chunks` array must have 1-20 items
- Each chunk text must be 10-5000 characters
- Similarity scores must be between 0 and 1

### Agent Response
**Description**: AI-generated response based on retrieved content with source attribution and confidence indicators

**Fields**:
- `response_text` (string, required): The agent's response to the user
- `sources` (array of objects, required): List of sources used in the response
- `sources[].content_id` (string): ID of the content chunk used
- `sources[].source_url` (string): URL of the source
- `sources[].chapter` (string): Chapter where content was found
- `sources[].section` (string, optional): Section where content was found
- `confidence_score` (number): Agent's confidence in the response (0-1)
- `grounding_validation` (object): Information about response grounding
- `grounding_validation.is_fully_ground` (boolean): Whether response is fully grounded
- `grounding_validation.unsupported_claims` (array of strings): Any unsupported claims
- `conversation_id` (string): ID for the conversation thread
- `timestamp` (string): When response was generated

**Validation**:
- `response_text` must be 1-10000 characters
- `confidence_score` must be between 0 and 1
- All claims in `response_text` must be supported by content in `sources`
- If `is_fully_grounded` is false, `unsupported_claims` must not be empty

### Conversation Session
**Description**: Stateful interaction context between user and agent for multi-turn conversations

**Fields**:
- `session_id` (string, required): Unique identifier for the conversation session
- `openai_thread_id` (string, required): OpenAI's thread ID for assistant memory
- `created_at` (string, required): Timestamp when session was created
- `last_interaction` (string, required): Timestamp of last interaction
- `user_id` (string, optional): ID of the user (for logged-in sessions)
- `context_mode` (enum, required): Current context mode for the session
- `active_filters` (object, optional): Active filters applied to the session
- `conversation_history` (array of objects): History of exchanges
- `conversation_history[].role` (enum: "user" | "assistant"): Role of the message
- `conversation_history[].content` (string): The message content
- `conversation_history[].timestamp` (string): When the message was exchanged

**Validation**:
- `session_id` must be unique
- `conversation_history` must not exceed 100 messages
- Session must be cleaned up after 24 hours of inactivity

## Relationships

### Agent Request → Retrieved Content
- An Agent Request triggers the retrieval of multiple Retrieved Content chunks
- The relationship is 1-to-many during the processing phase

### Retrieved Content → Agent Response
- Retrieved Content is used to generate an Agent Response
- Multiple content chunks may contribute to a single response
- The relationship is many-to-1

### Conversation Session → Agent Request/Response
- Conversation Session contains multiple Agent Requests and Agent Responses
- Each request-response pair is part of the session history
- The relationship is 1-to-many for both directions

## State Transitions

### Conversation Session States
1. **Created**: Session initialized, waiting for first user message
2. **Active**: Ongoing conversation with user
3. **Inactive**: No recent activity, preserved for a period
4. **Expired**: Session cleaned up due to inactivity

### Agent Response States
1. **Processing**: Agent is generating response
2. **Validated**: Response has been checked for grounding
3. **Delivered**: Response has been sent to user
4. **Flagged**: Response failed validation (for monitoring)

## Validation Rules

1. **Grounding Validation**: All content in Agent Response must be traceable to Retrieved Content
2. **Source Attribution**: Every fact in Agent Response must reference a source in Retrieved Content
3. **Context Mode Consistency**: Agent must respect the requested context mode (retrieval vs user-provided)
4. **Hallucination Prevention**: Agent Response must not contain information not present in Retrieved Content
5. **Confidence Threshold**: Responses with low confidence should be flagged or rejected