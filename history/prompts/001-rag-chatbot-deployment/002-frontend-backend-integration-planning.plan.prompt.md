---
id: 002
title: "Frontend–Backend Integration Planning"
stage: plan
date_iso: 2025-12-16
surface: agent
model: claude-sonnet-4-5-20250929
feature: rag-chatbot-deployment
branch: 001-rag-chatbot-deployment
user: Claude
command: /sp.plan
labels: ["frontend", "backend", "integration", "rag", "chatbot", "api"]
links:
  spec: specs/001-rag-chatbot-deployment/spec.md
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - specs/001-rag-chatbot-deployment/plan.md
  - specs/001-rag-chatbot-deployment/research.md
  - specs/001-rag-chatbot-deployment/data-model.md
  - specs/001-rag-chatbot-deployment/contracts/chat-api.yaml
  - specs/001-rag-chatbot-deployment/quickstart.md
tests_yaml: |
  - API contract validation
  - Data model verification
  - Implementation plan review
prompt_text: |
  /sp.plan Frontend–Backend Integration

  Steps:
  1. Create a chat UI component inside the Docusaurus frontend.
  2. Capture user input and optional selected text.
  3. Send queries to FastAPI chat endpoint via HTTP.
  4. Handle agent responses and render them in the UI.
  5. Configure CORS for local development.
  6. Validate selected-text-only query flow.
  7. Test error handling and empty responses.
  8. Prepare configuration for production deployment.

  Deliverables:
  - Embedded chatbot UI component
  - Frontend-to-backend API integration
  - Selected-text capture logic
  - Local and production configuration
response_text: |
  Completed comprehensive implementation planning for RAG Chatbot integration:

  1. Updated implementation plan (plan.md) with technical context, constitution check, and project structure
  2. Enhanced research findings (research.md) covering architecture, technology stack, and API design
  3. Created detailed data model (data-model.md) with entities, relationships, and validation rules
  4. Developed OpenAPI contract (contracts/chat-api.yaml) for frontend-backend communication
  5. Created quickstart guide (quickstart.md) for easy setup and deployment

  All artifacts align with project constitution and requirements. The plan provides clear guidance for implementing the chatbot UI component, API integration, selected-text capture, and error handling as specified in the user requirements.
outcome: |
  Implementation plan completed with all required artifacts: plan, research, data model, API contracts, and quickstart guide. Ready for task generation phase.
evaluation: |
  The planning phase successfully delivered all required artifacts with proper alignment to the feature specification. The API contract is well-defined, data models are comprehensive, and the technical approach aligns with the project constitution. All implementation details are clearly specified for the development team.
---

# Implementation Plan: Frontend–Backend Integration – RAG Chatbot for Book Website

**Branch**: `001-rag-chatbot-deployment` | **Date**: 2025-12-16 | **Spec**: [specs/001-rag-chatbot-deployment/spec.md](specs/001-rag-chatbot-deployment/spec.md)
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Integration of RAG agent backend with Docusaurus frontend through an embedded chatbot UI component that enables users to query book content. The implementation includes HTTP communication between frontend and backend services, selected text capture functionality, and proper error handling for both local development and production deployment.

## Technical Context

**Language/Version**: JavaScript/TypeScript (frontend), Python 3.11 (backend)
**Primary Dependencies**: Docusaurus, FastAPI, React, OpenAI Agents SDK, Qdrant
**Storage**: Qdrant vector database (for RAG), potentially Neon Postgres for user data
**Testing**: Jest/React Testing Library (frontend), pytest (backend)
**Target Platform**: Web browser (frontend), Linux server (backend)
**Project Type**: web
**Performance Goals**: <10 seconds response time for 95% of requests, 99% uptime for communication
**Constraints**: <200ms p95 for frontend-backend communication, proper CORS handling for local/production
**Scale/Scope**: Supports general questions, chapter-specific queries, and selected text context queries

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- ✅ **Spec-Driven Development**: Following established spec from `/specs/001-rag-chatbot-deployment/spec.md`
- ✅ **Grounded AI**: Responses must be grounded in book content via RAG pipeline
- ✅ **Unified System Architecture**: Integration between frontend and backend services
- ✅ **Content-First Development**: Focusing on educational content quality
- ✅ **Docusaurus-Centric Architecture**: Using Docusaurus standards for frontend
- ✅ **RAG Backend Integration**: FastAPI-based RAG backend integration with Docusaurus frontend
- ✅ **Frontend-Backend Communication**: HTTP communication between frontend and FastAPI backend
- ✅ **Chatbot UI Integration**: Embedded chatbot UI within published book
- ✅ **API Design and Security**: Proper request/response schema and CORS configuration
- ✅ **Technology Stack Compliance**: Using Docusaurus (frontend), FastAPI (backend), OpenAI Agents, Qdrant
- ✅ **Clean Separation of Concerns**: Clear boundaries between frontend and backend components
- ✅ **Mobile-Responsive Design**: Ensuring responsive design for chatbot UI

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-chatbot-deployment/
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
├── rag_agent/
│   ├── __init__.py
│   ├── chat_agent.py
│   └── rag_retriever.py
├── run_server.py
├── pyproject.toml
└── .python-version

frontend/
├── src/
│   ├── components/
│   │   └── Chatbot/
│   │       ├── Chatbot.jsx
│   │       ├── ChatMessage.jsx
│   │       └── ChatInput.jsx
│   ├── pages/
│   └── services/
│       └── api.js
├── static/
└── package.json

# Shared configuration
docker-compose.yml
```

**Structure Decision**: Web application structure with separate backend and frontend directories to maintain clear separation of concerns while enabling required integration. The backend contains the RAG agent and API services, while the frontend contains the Docusaurus site with embedded chatbot components.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [None] | [No violations identified] | [All constitution checks passed] |