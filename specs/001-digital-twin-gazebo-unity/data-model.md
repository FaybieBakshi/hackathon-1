# Data Model: Digital Twin (Gazebo & Unity) Documentation

## Content Entities

### Module
- **Name**: String (e.g., "Module 2: The Digital Twin")
- **Description**: Text (Overview of the module's purpose and learning objectives)
- **Prerequisites**: List of modules (e.g., "Module 1: ROS 2 basics")
- **Focus**: Text (Physics simulation and sensor modeling for humanoid robots)
- **Target Audience**: Text (Students with ROS 2 basics knowledge)

### Chapter
- **Title**: String (e.g., "Gazebo Fundamentals")
- **Description**: Text (Overview of chapter content and objectives)
- **Order**: Integer (Sequential order in the module)
- **Topics**: List of topics (Sub-topics within the chapter)
- **Learning Objectives**: List of objectives (What students should learn)

### Topic
- **Title**: String (e.g., "Physics Concepts")
- **Content**: Markdown text (Detailed explanation of the topic)
- **Examples**: List of examples (Practical examples to illustrate concepts)
- **Exercises**: List of exercises (Hands-on activities for students)
- **Assets**: List of assets (Images, diagrams, code snippets associated with topic)

### Asset
- **Type**: Enum (image, diagram, video, code, simulation)
- **Path**: String (Relative path to asset file)
- **Description**: Text (Brief description of what the asset shows/contains)
- **Associated Topics**: List of topic IDs (Topics that reference this asset)
- **Format**: String (File format, e.g., png, svg, mp4, py, urdf)

### Exercise
- **Title**: String (Name of the exercise)
- **Description**: Text (What the exercise aims to teach)
- **Steps**: List of steps (Sequential steps for the exercise)
- **Expected Outcome**: Text (What students should achieve)
- **Difficulty**: Enum (beginner, intermediate, advanced)
- **Estimated Time**: Integer (Time in minutes to complete)

## Relationships

- Module contains multiple Chapters
- Chapter contains multiple Topics
- Topic may reference multiple Assets
- Topic may include multiple Exercises
- Exercise may use multiple Assets

## Validation Rules

1. Module must have at least one Chapter
2. Chapter must have a unique title within the Module
3. Topic must belong to exactly one Chapter
4. Asset path must be valid and exist in the repository
5. Exercise difficulty must be one of the defined enum values
6. Learning objectives must be specific and measurable
7. Chapter order must be sequential without gaps
8. All referenced assets must exist in the assets directory

## State Transitions (if applicable)

- Content draft → Review → Approved → Published
- Exercise design → Testing → Validation → Available