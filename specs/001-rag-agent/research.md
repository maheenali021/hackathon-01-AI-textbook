# Research: RAG Agent Backend – OpenAI Agents SDK + FastAPI

**Feature**: RAG Agent Backend – OpenAI Agents SDK + FastAPI
**Date**: 2025-12-16
**Branch**: 001-rag-agent

## Research Summary

This document captures research for implementing an agent-powered backend using the OpenAI Agents SDK and FastAPI to answer questions about the AI robotics textbook by invoking a validated retrieval pipeline.

## Decision: OpenAI Assistant vs OpenAI Agent Implementation

**Rationale**: The OpenAI Agents SDK includes the Assistants API which is the current recommended approach for creating AI agents with tool calling capabilities. The Assistants API provides built-in memory, tool calling, and conversation management.

**Alternatives considered**:
1. OpenAI Chat Completions API with manual function calling
2. OpenAI Assistants API (recommended approach)
3. Third-party agent frameworks (LangChain, CrewAI, etc.)

**Decision**: Use OpenAI Assistants API as it provides the best integration with tool calling capabilities required for the retrieval pipeline integration.

## Decision: FastAPI for Agent Endpoint Wrapper

**Rationale**: FastAPI provides excellent performance, automatic API documentation, and easy integration with async operations required for agent interactions. It aligns with the constitution requirement for API design.

**Alternatives considered**:
1. Flask (simpler but less performant)
2. FastAPI (selected for performance and documentation features)
3. Django (overkill for this use case)

**Decision**: Use FastAPI as specified in the feature requirements and constitution.

## Decision: Tool Integration Pattern for Retrieval Pipeline

**Rationale**: The agent needs to call the existing retrieval pipeline from Spec-2. A custom tool implementation will allow the agent to invoke the retrieval pipeline when needed.

**Implementation approach**:
1. Create a custom OpenAI tool that wraps the retrieval pipeline
2. The tool will accept query parameters and return retrieved content
3. The agent will use this tool when it needs to access book content

## Decision: Grounding and Hallucination Prevention Strategy

**Rationale**: Critical to ensure all responses are based on retrieved content only, as required by the constitution's "Grounded AI" principle.

**Implementation approach**:
1. Agent instructions will mandate using only retrieved content
2. Response validation will check for content grounding
3. Source attribution will be included in all responses
4. When no relevant content is found, the agent will acknowledge this

## Decision: User-Provided Context Mode Implementation

**Rationale**: Support for answering questions based only on user-selected text as specified in the requirements.

**Implementation approach**:
1. Separate tool for user-provided context mode
2. When activated, bypasses the retrieval pipeline
3. Agent uses only the provided text context
4. Maintains same grounding and attribution requirements

## Decision: MCP Server Integration for Documentation

**Rationale**: The feature requires using MCP server to fetch and reference up-to-date OpenAI Agents SDK documentation.

**Implementation approach**:
1. Use MCP server during development to access latest SDK documentation
2. Validate implementation against current best practices
3. Ensure compatibility with latest SDK versions

## Best Practices: OpenAI Assistants API

**Research findings**:
- Assistants maintain their own state and memory
- Tool calling allows assistants to execute custom functions
- Threads represent individual conversations
- Messages are exchanged within threads
- Proper error handling is essential for production use

## Best Practices: FastAPI Integration

**Research findings**:
- Use Pydantic models for request/response validation
- Implement proper async/await patterns for agent interactions
- Add comprehensive logging and monitoring
- Implement rate limiting for production deployment
- Use dependency injection for configuration management

## Best Practices: Retrieval-Augmented Generation

**Research findings**:
- Agent should only respond based on retrieved content
- Source attribution is critical for trust and verification
- Response validation should check for grounding
- Fallback strategies needed when retrieval fails
- Proper handling of citations and references

## Integration Patterns: Agent to Retrieval Pipeline

**Research findings**:
- Tool functions should handle the connection to the existing pipeline
- Error handling needed for pipeline failures
- Caching strategies for frequently accessed content
- Proper serialization of retrieval results for agent consumption
- Metadata preservation for source attribution

## MCP Server Integration

**Research findings**:
- MCP (Model Context Protocol) servers can provide real-time documentation access
- Can be used to validate SDK usage against latest specifications
- Should be used during development but not required in production
- Enables access to the most current OpenAI Agents SDK documentation