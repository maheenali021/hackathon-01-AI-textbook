---
ID: 001
TITLE: amend-constitution-rag-integration
STAGE: constitution
DATE_ISO: 2025-12-16
MODEL: sonnet-4-5-20250929
FEATURE: Frontend-Backend Integration for Embedded RAG Chatbot
BRANCH: 004-rag-pipeline
USER: Maheen
COMMAND: /sp.constitution "Frontend–Backend Integration for Embedded RAG Chatbot in Docusaurus Book"
LABELS: ["governance", "architecture", "integration", "rag", "chatbot"]
LINKS:
  SPEC: null
  TICKET: null
  ADR: null
  PR: null
FILES_YAML:
  - .specify/memory/constitution.md
TESTS_YAML:
  - "constitution validation"
PROMPT_TEXT: |
  /sp.constitution Frontend–Backend Integration for Embedded RAG Chatbot in Docusaurus Book

  Target audience:
  - Readers of the Physical AI & Humanoid Robotics book
  - Developers reviewing the project implementation
  - Evaluators assessing end-to-end RAG system integration

  Objective:
  Integrate the FastAPI-based RAG backend with the Docusaurus frontend to provide an embedded, interactive chatbot that can answer questions about the book's content in real time.

  Success criteria:
  - Frontend can successfully send user queries to the FastAPI backend
  - Backend returns RAG-based responses using OpenAI Agents SDK + Qdrant retrieval
  - Chatbot UI is accessible from the published book (floating button or embedded widget)
  - Supports:
    - Free-form user questions about the book
    - Questions based only on user-selected text
  - End-to-end local development workflow works without errors
  - Production-ready configuration for deployment (env variables, CORS, API routing)

  Functional scope:
  - Establish HTTP/WebSocket communication between frontend and FastAPI backend
  - Implement request/response schema for chat messages
  - Handle session or conversation state on the backend
  - Integrate chatbot UI component into Docusaurus site
  - Configure CORS and environment variables for local and deployed environments

  Constraints:
  - Backend: FastAPI + OpenAI Agents SDK + MCP server
  - Frontend: Docusaurus (React-based)
  - Communication: REST or streaming-compatible endpoints
  - Authentication: API-key–based (no user auth system required)
  - Deployment target: GitHub Pages (frontend), cloud-hosted backend
  - Code style: Modular, readable, aligned with Spec-Kit Plus conventions

  Not building:
  - User authentication or login system
  - Payment or usage metering
  - Admin dashboard or analytics
  - Full real-time multi-user chat system
  - Model fine-tuning or embedding regeneration
RESPONSE_TEXT: |
  Updated the project constitution to include requirements for Frontend-Backend Integration for the Embedded RAG Chatbot. Key changes include:

  1. Updated the "Clean Separation of Concerns" principle to allow RAG integration while maintaining logical separation
  2. Added new principles:
     - RAG Backend Integration: Specifies FastAPI + OpenAI Agents SDK + Qdrant integration
     - Frontend-Backend Communication: HTTP/WebSocket communication with proper schemas
     - Chatbot UI Integration: Accessible via floating button/widget with responsive design
     - API Design and Security: API-key authentication, CORS, rate limiting
  3. Updated Technical Requirements to include FastAPI backend, OpenAI Agents SDK, and Qdrant
  4. Updated Development Workflow to include backend component testing
  5. Version bumped from 1.0.0 to 1.1.0
  6. Updated governance section to reflect RAG chatbot integration scope

  The constitution now properly governs the development of both the educational content and the RAG chatbot integration as specified in the user requirements.
OUTCOME: Constitution successfully updated to include RAG chatbot integration requirements
EVALUATION:
  SUCCESS_CRITERIA_MET: true
  COMPLIANCE_CHECK: All required elements included per user specification
  IMPACT_ASSESSMENT: Low risk - only governance document updated, no breaking changes to existing functionality
---

# Constitution Update for RAG Chatbot Integration

## Summary

This PHR documents the amendment to the Physical AI & Humanoid Robotics Educational Book Constitution to include requirements for Frontend-Backend Integration for the Embedded RAG Chatbot.

## Changes Made

- Updated constitution version from 1.0.0 to 1.1.0
- Modified "Clean Separation of Concerns" principle to accommodate RAG integration
- Added 4 new principles: RAG Backend Integration, Frontend-Backend Communication, Chatbot UI Integration, and API Design and Security
- Updated Technical Requirements to include backend components
- Updated Development Workflow to include backend testing
- Updated Governance section to reflect expanded scope

## Validation

- All user-specified requirements have been incorporated into the constitution
- The document maintains consistency with existing principles
- Versioning follows semantic versioning (minor update for new functionality)
- All placeholders have been properly filled