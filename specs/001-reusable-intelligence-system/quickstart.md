# Quickstart Guide: Reusable Intelligence System

## Overview
This guide helps you get started with the Reusable Intelligence System for the Physical AI & Humanoid Robotics book project. The system includes three Claude Code Subagents designed to assist with content creation, review, and summarization.

## Prerequisites
- Claude Code environment access
- Access to the book content and RAG pipeline
- Understanding of the Physical AI & Humanoid Robotics book domain

## Available Subagents

### 1. Chapter Writing Assistant
Helps create structured chapters for the book based on topics and requirements.

**Usage Example:**
```
Use the Chapter Writing Assistant to generate a draft for a chapter on "Introduction to Humanoid Robotics" with specific requirements for length, sections, and examples.
```

### 2. Content Review and Validation
Reviews content for technical accuracy and consistency with book standards.

**Usage Example:**
```
Submit existing content to the Content Reviewer to get feedback on technical accuracy, style consistency, and improvement suggestions.
```

### 3. Summarization and Structured Reasoning
Generates accurate summaries and provides structured reasoning based on book content.

**Usage Example:**
```
Use the Summarizer to create concise summaries of complex topics or to get structured reasoning for technical concepts.
```

## Integration with Workflows

### Writing Workflow
1. Use the Chapter Writing Assistant to generate initial drafts
2. Submit drafts to the Content Reviewer for validation
3. Refine content based on feedback
4. Finalize content for inclusion in the book

### RAG Chatbot Enhancement
1. The Summarization agent enhances chatbot responses with structured summaries
2. Content Reviewer ensures chatbot responses maintain technical accuracy
3. Chapter Writing Assistant can generate content to expand knowledge base

## Best Practices

### For Authors
- Always validate generated content with the Content Reviewer
- Use the Chapter Writing Assistant for initial drafts, not final content
- Review all generated content before publication
- Provide clear requirements when using subagents

### For Chatbot Enhancement
- Use summarization to improve response clarity
- Ensure all chatbot responses are grounded in book content
- Monitor technical accuracy of responses

## API Endpoints
- Chapter Writer: `POST /api/v1/subagents/chapter-writer`
- Content Reviewer: `POST /api/v1/subagents/content-reviewer`
- Summarizer: `POST /api/v1/subagents/summarizer`

## Expected Outcomes
- 50% reduction in manual content creation effort
- Improved technical accuracy of content
- Enhanced chatbot responses with structured reasoning
- Consistent style and quality across book chapters

## Troubleshooting
- If a subagent returns low accuracy scores, provide more context or specific requirements
- For better results, break down complex requests into smaller, focused tasks
- Always validate technical content with human experts before final publication