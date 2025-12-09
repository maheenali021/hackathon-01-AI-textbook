# API Contracts

This directory contains API contracts for backend services that will be implemented in future phases.

## Current Phase Status
- No backend APIs are required for the current phase
- The educational book is a static site built with Docusaurus
- All content is delivered as static files

## Future Phase Considerations
When backend functionality is added in future phases, the following API contracts will be relevant:

### Content Management APIs
- GET /api/content/modules - Retrieve all modules
- GET /api/content/modules/{id} - Retrieve specific module
- GET /api/content/chapters/{id} - Retrieve specific chapter
- GET /api/content/assets - Retrieve multimedia assets

### User Progress APIs
- POST /api/user/progress - Save user progress
- GET /api/user/progress - Retrieve user progress
- PUT /api/user/progress/{id} - Update user progress

### Search APIs
- GET /api/search - Search across all content
- POST /api/search/advanced - Advanced search functionality

## Contract Format
API contracts will be specified using OpenAPI 3.0 specification format.