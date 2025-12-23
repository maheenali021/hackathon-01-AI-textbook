# Summarization and Structured Reasoning Agent Contract

## Purpose
The Summarization Agent generates accurate summaries and provides structured reasoning based on book content to enhance the RAG chatbot's ability to assist users with complex technical queries.

## Endpoint
`POST /api/v1/subagents/summarizer`

## Request Schema
```json
{
  "type": "object",
  "required": ["content"],
  "properties": {
    "content": {
      "type": "string",
      "description": "The content to summarize or reason about",
      "minLength": 10,
      "maxLength": 50000
    },
    "options": {
      "type": "object",
      "properties": {
        "max_length": {
          "type": "integer",
          "minimum": 50,
          "maximum": 2000,
          "default": 500,
          "description": "Maximum length of the summary in words"
        },
        "style": {
          "type": "string",
          "enum": ["concise", "detailed", "technical", "layman"],
          "default": "concise",
          "description": "Style of the summary"
        },
        "include_reasoning": {
          "type": "boolean",
          "default": true,
          "description": "Whether to include structured reasoning"
        },
        "include_sources": {
          "type": "boolean",
          "default": true,
          "description": "Whether to include source attribution"
        },
        "target_audience": {
          "type": "string",
          "enum": ["beginner", "intermediate", "advanced"],
          "default": "intermediate",
          "description": "Target audience level for the summary"
        }
      }
    }
  }
}
```

## Response Schema
```json
{
  "type": "object",
  "required": ["success", "result"],
  "properties": {
    "success": {
      "type": "boolean",
      "description": "Whether the operation was successful"
    },
    "result": {
      "type": "object",
      "required": ["summary", "confidence_score"],
      "properties": {
        "summary": {
          "type": "string",
          "description": "The generated summary"
        },
        "reasoning": {
          "type": "string",
          "description": "Structured reasoning explaining the summary"
        },
        "confidence_score": {
          "type": "number",
          "minimum": 0,
          "maximum": 1,
          "description": "Confidence in the summary accuracy"
        },
        "sources": {
          "type": "array",
          "items": {
            "type": "string"
          },
          "description": "Sources referenced in the summary"
        },
        "key_points": {
          "type": "array",
          "items": {
            "type": "string"
          },
          "description": "Key points extracted from the content"
        },
        "technical_accuracy_score": {
          "type": "number",
          "minimum": 0,
          "maximum": 1,
          "description": "Technical accuracy of the summary"
        }
      }
    },
    "message": {
      "type": "string",
      "description": "Additional information or error message"
    }
  }
}
```

## Error Responses
- `400 Bad Request`: Invalid input parameters
- `500 Internal Server Error`: Agent processing error

## Examples

### Request
```json
{
  "content": "# Advanced Robotics Concepts\n\nHumanoid robots require sophisticated control systems...",
  "options": {
    "max_length": 300,
    "style": "technical",
    "include_reasoning": true,
    "include_sources": true,
    "target_audience": "advanced"
  }
}
```

### Response
```json
{
  "success": true,
  "result": {
    "summary": "Humanoid robots require sophisticated control systems that integrate multiple technologies...",
    "reasoning": "The summary focuses on control systems because they are fundamental to humanoid robot functionality...",
    "confidence_score": 0.89,
    "sources": [
      "Physical AI & Humanoid Robotics Book, Chapter 3",
      "Control Systems in Robotics"
    ],
    "key_points": [
      "Sophisticated control systems are essential",
      "Integration of multiple technologies required",
      "Balance and locomotion challenges"
    ],
    "technical_accuracy_score": 0.94
  }
}
```