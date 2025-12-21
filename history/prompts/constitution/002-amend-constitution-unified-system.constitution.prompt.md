---
ID: 002
TITLE: amend-constitution-unified-system
STAGE: constitution
DATE_ISO: 2025-12-16
MODEL: sonnet-4-5-20250929
FEATURE: Unified AI / Spec-Driven Book with Integrated RAG Chatbot
BRANCH: 004-rag-pipeline
USER: Maheen
COMMAND: /sp.constitution "Unified AI / Spec-Driven Book with Integrated RAG Chatbot"
LABELS: ["governance", "architecture", "integration", "rag", "chatbot", "auth", "personalization", "translation"]
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
  /sp.constitution

  Project: Unified AI / Spec-Driven Book with Integrated RAG Chatbot

  Purpose:
  Build a single, production-ready system that combines an AI-written technical book and an embedded Retrieval-Augmented Generation (RAG) chatbot, developed using Claude Code and Spec-Kit Plus.

  Core principles:
  - Spec-driven development (no implementation without an approved spec)
  - Grounded AI (all responses based strictly on book content)
  - Single unified system (book, chatbot, auth, personalization, translation)
  - Reproducibility and debuggability
  - Production-grade engineering practices

  Primary deliverables (Base: 100 points):
  1. AI / Spec-Driven Book Creation
     - Write the complete book using Claude Code
     - Use Spec-Kit Plus to drive specification, planning, tasks, and validation
     - Build the book with Docusaurus
     - Deploy the book to GitHub Pages

  2. Integrated RAG Chatbot
     - Embed a RAG chatbot inside the published book
     - Backend stack:
       - FastAPI
       - OpenAI Agents
       - Qdrant Cloud (Free Tier) for vector search
       - Neon Serverless Postgres for users, sessions, and chat history
     - Functional requirements:
       - Answer questions strictly from book content
       - Support answering based only on user-selected text
       - Maintain conversational context
       - Provide retrieval transparency for debugging

  Bonus objectives:
  3. Reusable Intelligence
     - Create and use Claude Code Subagents and Agent Skills
     - Subagents must be reusable across chapters
     - Clearly document agent responsibilities and usage

  4. Authentication & User Profiling
     - Implement Signup and Signin using https://www.better-auth.com/
     - During signup, collect:
       - Software background
       - Hardware / robotics background
     - Store user profiles securely
     - Use profile data to personalize content

  5. Chapter Personalization (+50)
     - Logged-in users can personalize chapters via a button at the start
     - Personalization adapts depth, explanations, and examples
     - Personalization must be powered by the agent + RAG pipeline

  6. Urdu Translation (+50)
     - Logged-in users can translate chapters into Urdu via a button
     - Translation must preserve technical accuracy
     - Translation must integrate seamlessly with the chapter UI

  Key standards:
  - All AI outputs must be grounded in retrieved book content
  - No hallucinations or unsupported claims
  - Clear separation of concerns between frontend, backend, and agents
  - Code must be readable, modular, and production-ready

  Constraints:
  - Frontend: Docusaurus + React
  - Backend: FastAPI
  - Agents: OpenAI Agents / ChatKit SDKs
  - Vector database: Qdrant Cloud Free Tier
  - Relational database: Neon Serverless Postgres
  - Authentication: Better-Auth
  - No technology substitutions without a documented spec decision

  Success criteria:
  - Book builds locally and deploys successfully to GitHub Pages
  - RAG chatbot answers book-only questions correctly
  - Selected-text question answering works
  - Authentication functions properly
RESPONSE_TEXT: |
  Major update to the project constitution to reflect the "Unified AI / Spec-Driven Book with Integrated RAG Chatbot" project. Key changes include:

  1. COMPLETE RESTRUCTURE: The entire constitution has been restructured to reflect the unified system approach
  2. NEW CORE PRINCIPLES ADDED:
     - Spec-Driven Development (NON-NEGOTIABLE)
     - Grounded AI
     - Unified System Architecture
     - Reusable Intelligence
     - Authentication and User Profiling
     - Chapter Personalization
     - Urdu Translation
     - Technology Stack Compliance

  3. UPDATED EXISTING PRINCIPLES:
     - RAG Backend Integration: Now includes Neon Postgres and Qdrant Cloud requirements
     - Frontend-Backend Communication: Now includes user data handling
     - Chatbot UI Integration: Now includes retrieval transparency
     - API Design and Security: Now includes user data security
     - Content-First Development: Updated to reflect new scope
     - Docusaurus-Centric Architecture: Updated to ensure GitHub Pages deployment

  4. VERSION BUMP: From 1.1.0 to 2.0.0 (MAJOR update due to fundamental project scope change)

  5. ENHANCED TECHNICAL REQUIREMENTS: Now includes production-grade practices, security requirements, and performance considerations

  6. ENHANCED GOVERNANCE: Added technology substitution constraint and enhanced validation requirements

  The constitution now properly governs the development of the unified system as specified in the user requirements, encompassing book, chatbot, auth, personalization, and translation in a single integrated system.
OUTCOME: Constitution successfully updated to reflect unified system requirements
EVALUATION:
  SUCCESS_CRITERIA_MET: true
  COMPLIANCE_CHECK: All required elements included per user specification
  IMPACT_ASSESSMENT: High impact - fundamental project scope change requiring major constitution update, but properly documented with versioning
---

# Constitution Update for Unified System

## Summary

This PHR documents the major amendment to the constitution to reflect the "Unified AI / Spec-Driven Book with Integrated RAG Chatbot" project, transforming it from a simple RAG chatbot integration to a comprehensive unified system with authentication, personalization, and translation capabilities.

## Changes Made

- Updated constitution version from 1.1.0 to 2.0.0 (major update)
- Complete restructuring to reflect unified system approach
- Added 7 new core principles covering all project requirements
- Updated all existing principles to align with new scope
- Enhanced technical requirements and governance sections
- Added technology stack compliance as a core principle

## Validation

- All user-specified requirements have been incorporated into the constitution
- The document maintains consistency with existing principles while expanding scope
- Versioning follows semantic versioning (major update for fundamental scope change)
- All placeholders have been properly filled