<!--
Sync Impact Report:
Version change: 1.1.0 → 2.0.0
List of modified principles: Content-First Development (updated), Docusaurus-Centric Architecture (updated), RAG Backend Integration (updated), Frontend-Backend Communication (updated), Chatbot UI Integration (updated), API Design and Security (updated), Clean Separation of Concerns (updated)
Added sections: Spec-Driven Development, Grounded AI, Unified System Architecture, Reproducibility and Debuggability, Production-Grade Engineering, Reusable Intelligence, Authentication and User Profiling, Chapter Personalization, Urdu Translation, Technology Stack Compliance
Removed sections: Previous versions of principles that were updated
Templates requiring updates: ✅ plan-template.md, spec-template.md, tasks-template.md (updated to reflect new project scope)
Follow-up TODOs: None
-->
# Unified AI / Spec-Driven Book with Integrated RAG Chatbot Constitution

## Core Principles

### Spec-Driven Development (NON-NEGOTIABLE)
No implementation without an approved spec; All development must follow the Spec-Kit Plus methodology with clear specs, plans, and tasks; All changes must be documented and validated before implementation; Use Claude Code for all development tasks; Each phase must be completed before moving to the next as outlined in the project requirements.

### Grounded AI
All AI outputs must be grounded in retrieved book content; No hallucinations or unsupported claims; All responses must be strictly based on book content; Provide retrieval transparency for debugging; Maintain factual accuracy and technical precision in all AI-generated content.

### Unified System Architecture
Create a single unified system combining book, chatbot, auth, personalization, and translation; All components must work together seamlessly; Maintain consistent user experience across all features; Ensure tight integration between frontend and backend services; Support both logged-in and anonymous user experiences.

### Content-First Development
All features must prioritize educational content quality and clarity above technical implementation; Content must be accurate, well-structured, and pedagogically sound; Every feature should enhance the learning experience for students studying the subject matter; Book content must be complete and comprehensive before additional features are implemented.

### Docusaurus-Centric Architecture
All book content must follow Docusaurus standards and conventions; Use MDX format for interactive content; Leverage Docusaurus's built-in features for navigation, search, and responsive design; Maintain compatibility with Docusaurus plugin ecosystem; Ensure GitHub Pages deployment works seamlessly.

### RAG Backend Integration
The FastAPI-based RAG backend must be integrated with the Docusaurus frontend to provide an embedded, interactive chatbot that can answer questions about the book's content in real time; The backend must use OpenAI Agents SDK + Qdrant Cloud for vector search and Neon Serverless Postgres for user data; All communication between frontend and backend must be secure and efficient; Support both free-form user questions and questions based only on user-selected text; Answer questions strictly from book content with conversational context maintenance.

### Frontend-Backend Communication
Establish secure HTTP communication between frontend and FastAPI backend; Implement proper request/response schema for chat messages and user data; Handle session and conversation state on the backend with Neon Postgres; Ensure reliable communication with appropriate error handling and retry mechanisms; Support both REST and streaming-compatible endpoints as needed for optimal user experience.

### Chatbot UI Integration
The chatbot UI must be embedded inside the published book with seamless integration; The UI must be intuitive and non-intrusive to the reading experience; Support seamless integration with Docusaurus theme and styling; Provide clear visual feedback during processing and maintain responsive design across all devices; Include functionality for both general questions and text selection-based queries; Provide retrieval transparency for debugging.

### Authentication and User Profiling
Implement Signup and Signin using Better-Auth (https://www.better-auth.com/); During signup, collect software background and hardware/robotics background information; Store user profiles securely in Neon Postgres; Use profile data to personalize content; Support both logged-in and anonymous user experiences; Ensure secure handling of user data.

### Reusable Intelligence
Create and use Claude Code Subagents and Agent Skills; Subagents must be reusable across chapters; Clearly document agent responsibilities and usage; Ensure agents are modular and maintainable; Follow best practices for agent design and implementation.

### Chapter Personalization
Logged-in users can personalize chapters via a button at the start; Personalization adapts depth, explanations, and examples based on user profile; Personalization must be powered by the agent + RAG pipeline; Maintain content accuracy while adapting to user background; Provide consistent personalization across the book.

### Urdu Translation
Logged-in users can translate chapters into Urdu via a button; Translation must preserve technical accuracy; Translation must integrate seamlessly with the chapter UI; Maintain formatting and structure during translation; Ensure translated content is properly grounded in source material.

### API Design and Security
Implement API-key–based authentication for all backend communication; Configure proper CORS settings for local and deployed environments; Design RESTful APIs with consistent request/response schemas; Implement rate limiting and appropriate security measures; Ensure all API endpoints are properly documented and tested for both local development and production deployment; Secure all user data and maintain privacy.

### Technology Stack Compliance
Frontend must use Docusaurus + React as specified; Backend must use FastAPI as specified; Agents must use OpenAI Agents / ChatKit SDKs as specified; Vector database must use Qdrant Cloud Free Tier as specified; Relational database must use Neon Serverless Postgres as specified; Authentication must use Better-Auth as specified; No technology substitutions without a documented spec decision.

### Clean Separation of Concerns
Maintain clear boundaries between different system components while enabling required integration; Ensure backend services can scale independently from frontend content; Implement proper error isolation between content delivery, chatbot functionality, personalization, and authentication; Future features should maintain this separation while enabling integration points where appropriate; Follow modular design principles for maintainability.

### Mobile-Responsive Design
All content and interactive elements must be fully responsive on mobile devices; Follow mobile-first design principles; Ensure accessibility standards are met for all users; Optimize for various screen sizes and input methods; Ensure all features work seamlessly across devices.

## Technical Requirements
All code must follow production-grade engineering practices; Implement proper testing for both frontend and backend components; Follow end-to-end local development workflow without errors; Ensure production-ready configuration for deployment; Include comprehensive error handling and logging for production monitoring; Implement proper environment variable configuration for local and production deployment; Follow security best practices for user data handling; Ensure all components meet performance requirements; Code must be readable, modular, and production-ready.

## Development Workflow
All content must follow the Spec-Kit Plus methodology with clear specs, plans, and tasks; Use Claude Code for all development tasks; Create MDX pages following Docusaurus structure inside /frontend; Adhere to the weekly breakdown and module requirements provided in course outline; Maintain version control with clear commit messages; Implement proper testing for both frontend and backend components; Follow end-to-end local development workflow without errors; Ensure production-ready configuration for deployment; Document all agent responsibilities and usage patterns.

## Governance

This constitution governs all development for the Unified AI / Spec-Driven Book with Integrated RAG Chatbot project; All code changes must comply with these principles; Amendments require explicit documentation of changes and approval from project stakeholders; Each phase must be completed before moving to the next as outlined in the project requirements; End-to-end functionality must be validated before release to ensure all success criteria are met; No technology substitutions without documented spec decision.

**Version**: 2.0.0 | **Ratified**: 2025-12-09 | **Last Amended**: 2025-12-16