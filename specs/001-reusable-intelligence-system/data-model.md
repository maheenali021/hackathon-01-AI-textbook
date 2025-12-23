# Data Model: Reusable Intelligence System

## Entities

### Subagent
- **Definition**: A specialized Claude Code component that performs a specific task with reusable prompts
- **Attributes**:
  - id: string (unique identifier)
  - name: string (display name)
  - description: string (purpose and functionality)
  - type: enum (chapter_writer, content_reviewer, summarizer)
  - prompt_template: string (structured prompt for the agent)
  - created_at: datetime
  - updated_at: datetime

### AgentSkill
- **Definition**: A reusable function or capability that can be invoked during book writing or chatbot response generation
- **Attributes**:
  - id: string (unique identifier)
  - name: string (skill name)
  - description: string (what the skill does)
  - input_schema: object (structure of required inputs)
  - output_schema: object (structure of expected outputs)
  - category: enum (content_generation, validation, summarization)
  - created_at: datetime
  - updated_at: datetime

### PromptTemplate
- **Definition**: A structured template that ensures deterministic and reusable behavior across subagent invocations
- **Attributes**:
  - id: string (unique identifier)
  - name: string (template name)
  - description: string (what the template is for)
  - content: string (the actual prompt template)
  - variables: array (placeholders for dynamic content)
  - subagent_type: enum (chapter_writer, content_reviewer, summarizer)
  - created_at: datetime
  - updated_at: datetime

### AgentInteraction
- **Definition**: A record of an interaction with a subagent
- **Attributes**:
  - id: string (unique identifier)
  - subagent_id: string (which subagent was used)
  - input_data: object (what was provided to the agent)
  - output_data: object (what the agent returned)
  - timestamp: datetime
  - user_id: string (optional, who invoked the agent)
  - context: string (the purpose or use case)

## Relationships

- **Subagent** 1:M **AgentSkill** (one subagent can use multiple skills)
- **Subagent** 1:1 **PromptTemplate** (one subagent uses one primary template)
- **Subagent** 1:M **AgentInteraction** (one subagent can have many interactions)

## Validation Rules

### Subagent
- Name must be 3-50 characters
- Description must be 10-500 characters
- Type must be one of the defined enum values
- Prompt template must exist

### AgentSkill
- Name must be 3-50 characters
- Description must be 10-500 characters
- Input and output schemas must be valid JSON Schema
- Category must be one of the defined enum values

### PromptTemplate
- Name must be 3-50 characters
- Content must be 20-5000 characters
- Variables must match placeholders in content
- Subagent type must be one of the defined enum values

## State Transitions

### Subagent
- DRAFT → ACTIVE (when validated and ready for use)
- ACTIVE → ARCHIVED (when no longer needed)

### AgentSkill
- DRAFT → ACTIVE (when validated and ready for use)
- ACTIVE → ARCHIVED (when no longer needed)