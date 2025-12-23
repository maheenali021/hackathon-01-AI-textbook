# Content Review Agent Contract

## Purpose
The Content Review Agent reviews content for technical accuracy and consistency with the book's standards, validating technical concepts and suggesting improvements.

## Endpoint
`POST /api/v1/subagents/content-reviewer`

## Request Schema
```json
{
  "type": "object",
  "required": ["content"],
  "properties": {
    "content": {
      "type": "string",
      "description": "The content to review",
      "minLength": 10,
      "maxLength": 50000
    },
    "criteria": {
      "type": "object",
      "description": "Specific criteria for the review",
      "properties": {
        "technical_accuracy": {
          "type": "boolean",
          "default": true,
          "description": "Review for technical accuracy"
        },
        "style_consistency": {
          "type": "boolean",
          "default": true,
          "description": "Review for style consistency with book standards"
        },
        "target_audience": {
          "type": "string",
          "enum": ["beginner", "intermediate", "advanced"],
          "description": "Target audience level for appropriateness"
        },
        "completeness": {
          "type": "boolean",
          "default": true,
          "description": "Review for completeness of coverage"
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
  "required": ["success", "review_results"],
  "properties": {
    "success": {
      "type": "boolean",
      "description": "Whether the operation was successful"
    },
    "review_results": {
      "type": "object",
      "required": ["feedback", "accuracy_score", "suggestions"],
      "properties": {
        "feedback": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "type": {
                "type": "string",
                "enum": ["error", "warning", "suggestion", "positive"]
              },
              "category": {
                "type": "string",
                "enum": ["technical", "style", "structure", "accuracy", "completeness"]
              },
              "message": {
                "type": "string",
                "description": "Detailed feedback message"
              },
              "location": {
                "type": "string",
                "description": "Location in the content where issue occurs"
              },
              "severity": {
                "type": "string",
                "enum": ["critical", "high", "medium", "low"]
              }
            }
          }
        },
        "accuracy_score": {
          "type": "number",
          "minimum": 0,
          "maximum": 1,
          "description": "Overall technical accuracy score"
        },
        "suggestions": {
          "type": "array",
          "items": {
            "type": "string"
          },
          "description": "Specific improvement suggestions"
        },
        "summary": {
          "type": "object",
          "properties": {
            "technical_issues": {
              "type": "integer",
              "description": "Number of technical issues found"
            },
            "style_issues": {
              "type": "integer",
              "description": "Number of style issues found"
            },
            "suggestions_count": {
              "type": "integer",
              "description": "Number of suggestions provided"
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
  "content": "# Introduction to Robotics\n\nRobotics is a field that studies robots...",
  "criteria": {
    "technical_accuracy": true,
    "style_consistency": true,
    "target_audience": "intermediate",
    "completeness": true
  }
}
```

### Response
```json
{
  "success": true,
  "review_results": {
    "feedback": [
      {
        "type": "suggestion",
        "category": "technical",
        "message": "Consider adding more specific examples of robotic applications",
        "location": "Introduction paragraph",
        "severity": "medium"
      }
    ],
    "accuracy_score": 0.85,
    "suggestions": [
      "Add specific examples of robotic applications",
      "Include more technical specifications"
    ],
    "summary": {
      "technical_issues": 1,
      "style_issues": 0,
      "suggestions_count": 2
    }
  }
}
```