# Data Model: Physical AI & Humanoid Robotics Educational Book

## Overview
This document defines the data structures and relationships for the educational book content. Since this is primarily a content delivery system in the current phase, the data model focuses on the content organization rather than complex data relationships.

## Content Entities

### Module
- **Definition**: A major section of the book containing multiple chapters
- **Fields**:
  - id: string (unique identifier, e.g., "module1-ros2")
  - title: string (e.g., "The Robotic Nervous System (ROS 2)")
  - description: string (brief description of the module focus)
  - order: integer (sequence number for navigation)
  - chapters: array of Chapter objects
  - focus: string (primary learning objective)
- **Relationships**: Contains multiple Chapter entities
- **Validation**: Must have at least one chapter, unique ID across all modules

### Chapter
- **Definition**: A subsection within a module that covers specific topics and concepts
- **Fields**:
  - id: string (unique identifier, e.g., "module1-chapter1")
  - title: string (chapter title)
  - content: string (the main content in MDX format)
  - module_id: string (reference to parent Module)
  - order: integer (sequence within the module)
  - prerequisites: array of strings (other chapters that should be completed first)
  - learning_objectives: array of strings (what the user should learn)
  - duration_estimate: integer (estimated reading time in minutes)
- **Relationships**: Belongs to one Module, may reference other Chapter entities as prerequisites
- **Validation**: Must belong to an existing module, content must be valid MDX

### Content
- **Definition**: Educational material including text, code examples, diagrams, and interactive elements
- **Fields**:
  - id: string (unique identifier)
  - type: enum (text, code, diagram, image, video, interactive)
  - content: string or object (the actual content)
  - metadata: object (additional information like language for code blocks)
  - created_at: datetime
  - updated_at: datetime
- **Relationships**: Associated with a Chapter
- **Validation**: Content format must match the specified type

### Multimedia Asset
- **Definition**: Images, diagrams, videos, or GIFs that support the educational content
- **Fields**:
  - id: string (unique identifier)
  - filename: string (original file name)
  - filepath: string (relative path from static directory)
  - alt_text: string (accessibility description)
  - type: enum (image, video, gif, diagram)
  - size: integer (file size in bytes)
  - dimensions: object (width and height for images)
  - chapter_id: string (reference to associated Chapter)
  - usage_context: string (where in the chapter this asset is used)
- **Relationships**: Associated with a Chapter
- **Validation**: File must exist at specified path, alt text required for accessibility

## Navigation Entities

### NavigationItem
- **Definition**: Represents an item in the sidebar navigation
- **Fields**:
  - id: string (unique identifier)
  - title: string (display title)
  - href: string (relative path to content)
  - order: integer (position in navigation)
  - parent_id: string (optional, for nested navigation)
  - type: enum (module, chapter, section, link)
- **Relationships**: May have parent-child relationships with other NavigationItems
- **Validation**: Href must point to existing content

## Relationships

### Module-Chapters
- One-to-Many: One Module contains many Chapters
- Module (1) → (0..n) Chapters

### Chapter-Multimedia Assets
- One-to-Many: One Chapter can have many Multimedia Assets
- Chapter (1) → (0..n) Multimedia Assets

### Chapter-Content
- One-to-Many: One Chapter contains many Content items
- Chapter (1) → (0..n) Content

## State Transitions (if applicable)

### Content Approval Process (for future phases)
- Draft → Review → Approved → Published
- Each state has specific validation requirements and associated actions

## Validation Rules from Requirements

### Module Validation
- Each module must have a title and description
- Modules must be organized in the specified sequence (Introduction, Module 1-4, Capstone)
- Each module must contain at least one chapter

### Chapter Validation
- Each chapter must have content that follows MDX format
- Chapters must be linked to a valid module
- Chapter titles must be unique within a module
- All chapters must have learning objectives defined

### Multimedia Asset Validation
- All assets must meet accessibility standards (alt text for images)
- File paths must be valid and accessible
- Assets must be associated with at least one chapter

## Constraints

### Structural Constraints
- Content must be organized in the specified module structure
- Each module must follow the specified chapter breakdown from the requirements
- Navigation must reflect the module-chapter hierarchy

### Quality Constraints
- All content must be pedagogically sound
- Code examples must be accurate and tested
- Interactive elements must work across devices (responsive)

### Technical Constraints
- Content must be compatible with Docusaurus MDX format
- Assets must be optimized for web delivery
- All content must be accessible following WCAG 2.1 AA standards