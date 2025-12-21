# Implementation Plan: RAG Agent Backend – OpenAI Agents SDK + FastAPI

**Branch**: `001-rag-agent` | **Date**: 2025-12-16 | **Spec**: [link](spec.md)
**Input**: Feature specification from `/specs/001-rag-agent/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of an agent-powered backend using the OpenAI Agents SDK and FastAPI that answers user questions about the AI robotics textbook by invoking a validated retrieval pipeline. The system creates an OpenAI Agent with tool calling capabilities to connect to the existing retrieval pipeline, ensuring responses are grounded in book content with source attribution and hallucination prevention.

## Technical Context

**Language/Version**: Python 3.9+
**Primary Dependencies**: OpenAI Agents SDK, FastAPI, Pydantic, uvicorn, OpenAPI tools
**Storage**: N/A (integrates with existing Qdrant Cloud and retrieval pipeline from Spec-2)
**Testing**: pytest for backend testing, OpenAPI contract testing
**Target Platform**: Linux server (cloud deployment)
**Project Type**: web (backend service)
**Performance Goals**: <10 seconds response time for 95% of queries, 90%+ success rate for general questions
**Constraints**: <10 seconds p95 response time, hallucination prevention (99%+ accuracy), proper source attribution (95%+ of responses)
**Scale/Scope**: Support concurrent users asking questions about the AI robotics textbook, handle both general and chapter-specific queries

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Gates determined based on constitution file:**

1. **Grounded AI Principle**: ✅ PASSED - Agent responses must be grounded strictly in retrieved book content with no hallucinations (FR-003, FR-009, SC-004)
2. **RAG Backend Integration**: ✅ PASSED - Implementation uses OpenAI Agents SDK with existing retrieval pipeline as required (FR-002, FR-007)
3. **API Design and Security**: ✅ PASSED - FastAPI backend with proper request/response schemas (FR-007)
4. **Technology Stack Compliance**: ✅ PASSED - Uses OpenAI Agents SDK and FastAPI as specified (FR-001, FR-007)
5. **Content-First Development**: ✅ PASSED - Focuses on accurate book content delivery (FR-003, FR-008)
6. **Clean Separation of Concerns**: ✅ PASSED - Backend service maintains clear boundaries while enabling integration (FR-010)

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-agent/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
└── rag_agent/
    ├── __init__.py
    ├── main.py                 # FastAPI app entry point
    ├── agents/
    │   ├── __init__.py
    │   ├── book_agent.py       # OpenAI Agent implementation
    │   └── tools/
    │       ├── __init__.py
    │       ├── retrieval_tool.py   # Tool for calling retrieval pipeline
    │       └── context_tool.py     # Tool for user-provided context mode
    ├── api/
    │   ├── __init__.py
    │   ├── chat_endpoints.py     # FastAPI chat endpoints
    │   └── models/
    │       ├── __init__.py
    │       ├── request_models.py   # Request schemas
    │       └── response_models.py  # Response schemas
    ├── services/
    │   ├── __init__.py
    │   ├── agent_service.py      # Agent orchestration service
    │   └── validation_service.py # Response validation service
    ├── utils/
    │   ├── __init__.py
    │   ├── config.py             # Configuration utilities
    │   ├── logging_config.py     # Logging setup
    │   └── helpers.py            # Helper functions
    ├── tests/
    │   ├── __init__.py
    │   ├── test_agents/
    │   │   ├── __init__.py
    │   │   ├── test_book_agent.py
    │   │   └── test_tools.py
    │   ├── test_api/
    │   │   ├── __init__.py
    │   │   ├── test_chat_endpoints.py
    │   │   └── test_models.py
    │   └── conftest.py
    └── requirements.txt
```

**Structure Decision**: Backend service structure selected to integrate with existing retrieval pipeline from Spec-2. The agent implementation uses OpenAI Agents SDK with custom tools to interface with the validated retrieval pipeline, and FastAPI endpoints provide the interface for agent interaction as required by the constitution and feature spec.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| N/A | N/A | N/A |
