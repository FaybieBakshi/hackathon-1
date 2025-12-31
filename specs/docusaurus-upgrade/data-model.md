# Data Model: Docusaurus Project Upgrade

## Content Structure

### Module Entity
- **Name**: Unique identifier for the module (e.g., "module-1", "module-2")
- **Title**: Display title for the module
- **Description**: Brief description of the module content
- **Chapters**: Array of chapter objects
- **Order**: Numeric position in the overall curriculum

### Chapter Entity
- **ID**: Unique identifier within the module
- **Title**: Display title for the chapter
- **Path**: URL path relative to module
- **Content**: Markdown content or reference
- **Prerequisites**: Array of prerequisite chapter IDs
- **LearningObjectives**: Array of learning objectives
- **ModuleID**: Reference to parent module

### Navigation Entity
- **Label**: Display text for navigation item
- **Type**: "category" or "doc"
- **Items**: Array of child navigation items (for categories)
- **DocID**: Reference to specific document (for doc type)
- **Collapsed**: Boolean for default collapsed state

## Content Relationships

### Module to Chapter
- One-to-Many relationship
- Module contains multiple chapters
- Chapters belong to single module

### Chapter to Prerequisites
- Many-to-Many relationship
- Chapters can have multiple prerequisite chapters
- Prerequisites can be required by multiple chapters

### Navigation Hierarchy
- Tree structure with parent-child relationships
- Categories contain other categories or documents
- Documents are leaf nodes in navigation tree

## Validation Rules

### Module Validation
- Name must be unique across all modules
- Title must not be empty
- Order must be a positive integer
- At least one chapter required per module

### Chapter Validation
- ID must be unique within module
- Title must not be empty
- Path must be valid URL format
- ModuleID must reference existing module

### Navigation Validation
- Labels must not be empty
- Type must be either "category" or "doc"
- DocID must reference existing document when type is "doc"

## State Transitions

### Content Migration States
1. **Original** - Content in current structure
2. **Migrated** - Content converted to new modular structure
3. **Validated** - Content tested and verified functional
4. **Deployed** - Content live in production

### Migration Workflow
- Original → Migrated: Content structure transformation
- Migrated → Validated: Testing and verification
- Validated → Deployed: Production deployment
- Any state → Original: Rollback capability

## File Organization Schema

### Module Directory Structure
```
docs/
├── module-1/
│   ├── introduction.md
│   ├── chapter-1.md
│   ├── chapter-2.md
│   └── conclusion.md
├── module-2/
│   ├── overview.md
│   ├── lesson-1.md
│   └── summary.md
└── tutorials/
    ├── getting-started.md
    └── advanced-topics.md
```

### Navigation Configuration Schema
```
Sidebar Configuration:
- Module 1 [Category]
  - Introduction [Doc]
  - Chapter 1 [Doc]
  - Chapter 2 [Doc]
- Module 2 [Category]
  - Overview [Doc]
  - Lesson 1 [Doc]
  - Summary [Doc]
- Tutorials [Category]
  - Getting Started [Doc]
  - Advanced Topics [Doc]
```