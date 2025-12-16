# Data Model: ROS2 Educational Module

## Entities

### Chapter
- **name**: String - The name of the chapter (e.g., "ROS 2 Fundamentals")
- **module**: String - The module this chapter belongs to (e.g., "Module 1")
- **content**: String - The main content of the chapter in Markdown format
- **learningObjectives**: Array<String> - List of learning objectives for the chapter
- **prerequisites**: Array<String> - Knowledge required before reading this chapter
- **concepts**: Array<String> - Key concepts covered in the chapter
- **examples**: Array<String> - Examples included in the chapter
- **summary**: String - Summary of the chapter content
- **nextChapter**: String - Link to the next chapter in the sequence

### Module
- **id**: String - The identifier for the module (e.g., "module-1")
- **title**: String - The title of the module (e.g., "The Robotic Nervous System")
- **description**: String - Description of the module content
- **chapters**: Array<Chapter> - List of chapters in this module
- **targetAudience**: String - Description of the target audience
- **prerequisites**: String - Prerequisites for the entire module

### NavigationItem
- **id**: String - Unique identifier for the navigation item
- **title**: String - Title to display in navigation
- **path**: String - URL path for the navigation item
- **children**: Array<NavigationItem> - Child navigation items (for sidebar structure)

## Validation Rules

### Chapter Validation
- **name** must not be empty
- **content** must be in valid Markdown format
- **learningObjectives** must contain at least one objective
- **prerequisites** must align with the target audience requirements
- **nextChapter** must be a valid reference to another chapter

### Module Validation
- **id** must follow the format "module-[number]"
- **chapters** must contain at least one chapter
- **chapters** must be ordered in learning progression sequence
- **targetAudience** must match the specification requirements

## State Transitions

### Chapter States
- **Draft**: Initial state when chapter is being created
- **Review**: Chapter is ready for review
- **Published**: Chapter is complete and published
- **Archived**: Chapter is no longer active (not applicable for this educational module)

## Relationships

### Module to Chapters
- One Module contains many Chapters
- Chapters are ordered within the Module according to learning progression
- Each Chapter belongs to exactly one Module

### Navigation Structure
- NavigationItems form a hierarchical structure
- Module and Chapter entities are referenced by NavigationItems
- NavigationItems define the sidebar structure for the documentation