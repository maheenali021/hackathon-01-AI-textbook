# Chapter Writing Agent Contract

## Purpose
The Chapter Writing Agent assists in creating structured content for the Physical AI & Humanoid Robotics book by generating chapters based on specified topics and requirements.

## Endpoint
`POST /api/v1/subagents/chapter-writer`

## Request Schema
```json
{
  "type": "object",
  "required": ["topic", "requirements"],
  "properties": {
    "topic": {
      "type": "string",
      "description": "The main topic for the chapter",
      "minLength": 5,
      "maxLength": 200
    },
    "requirements": {
      "type": "object",
      "description": "Specific requirements for the chapter",
      "properties": {
        "target_audience": {
          "type": "string",
          "enum": ["beginner", "intermediate", "advanced"],
          "description": "Target audience level for the chapter"
        },
        "length": {
          "type": "integer",
          "minimum": 500,
          "maximum": 10000,
          "description": "Target length in words"
        },
        "sections": {
          "type": "array",
          "items": {
            "type": "string"
          },
          "description": "Specific sections that should be covered"
        },
        "examples_needed": {
          "type": "boolean",
          "description": "Whether to include code examples"
        }
      }
    },
    "context": {
      "type": "string",
      "description": "Additional context or constraints",
      "maxLength": 1000
    }
  }
}
```

## Response Schema
```json
{
  "type": "object",
  "required": ["success", "chapter_draft"],
  "properties": {
    "success": {
      "type": "boolean",
      "description": "Whether the operation was successful"
    },
    "chapter_draft": {
      "type": "object",
      "required": ["title", "content", "sections", "metadata"],
      "properties": {
        "title": {
          "type": "string",
          "description": "Generated chapter title"
        },
        "content": {
          "type": "string",
          "description": "Full chapter content in markdown format"
        },
        "sections": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "title": {
                "type": "string"
              },
              "content": {
                "type": "string"
              }
            }
          }
        },
        "metadata": {
          "type": "object",
          "properties": {
            "word_count": {
              "type": "integer"
            },
            "estimated_reading_time": {
              "type": "integer",
              "description": "In minutes"
            },
            "technical_accuracy_score": {
              "type": "number",
              "minimum": 0,
              "maximum": 1,
              "description": "Estimated accuracy score"
            }
          }
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
  "topic": "Introduction to Humanoid Robotics",
  "requirements": {
    "target_audience": "intermediate",
    "length": 2000,
    "sections": ["Definition", "History", "Applications"],
    "examples_needed": true
  },
  "context": "Focus on modern humanoid robots like Atlas and ASIMO"
}
```

### Response
```json
{
  "success": true,
  "chapter_draft": {
    "title": "Introduction to Humanoid Robotics",
    "content": "# Introduction to Humanoid Robotics\n\nHumanoid robots are robots that resemble the human body structure...",
    "sections": [
      {
        "title": "Definition",
        "content": "Humanoid robots are robots that resemble the human body structure..."
      }
    ],
    "metadata": {
      "word_count": 2156,
      "estimated_reading_time": 9,
      "technical_accuracy_score": 0.92
    }
  }
}
```