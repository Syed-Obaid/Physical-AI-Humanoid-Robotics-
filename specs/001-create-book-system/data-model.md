# Data Model: AI/Spec-Driven Book Creation System

## Entities

### Book
- **name**: String (required) - The title of the book
- **description**: String (required) - A brief description of the book's purpose and content
- **modules**: [Module] (required) - The collection of modules that make up the book
- **authors**: [String] (optional) - List of book authors
- **version**: String (required) - The current version of the book following semantic versioning
- **createdDate**: DateTime (required) - When the book was first created
- **lastUpdated**: DateTime (required) - When the book was last modified
- **status**: Enum (draft, review, published) - The current status of the book
- **metadata**: Object - Additional metadata following Docusaurus requirements

### Module
- **id**: String (required) - Unique identifier for the module
- **title**: String (required) - The title of the module
- **description**: String (required) - A brief description of the module's content
- **chapters**: [Chapter] (required) - The collection of chapters in the module
- **order**: Integer (required) - The position of the module within the book (1-6 as per spec)
- **learningObjectives**: [String] (required) - List of learning objectives for the module
- **prerequisites**: [String] (optional) - Prerequisites for understanding this module
- **status**: Enum (draft, review, published) - The current status of the module

### Chapter
- **id**: String (required) - Unique identifier for the chapter
- **title**: String (required) - The title of the chapter
- **content**: String (required) - The main content of the chapter in Markdown/MDX
- **moduleId**: String (required) - Reference to the parent module
- **order**: Integer (required) - The position of the chapter within its module
- **learningObjectives**: [String] (required) - List of learning objectives for the chapter
- **prerequisites**: [String] (optional) - Prerequisites for understanding this chapter
- **examples**: [Example] (optional) - Associated code examples or exercises
- **references**: [Reference] (optional) - Academic or technical references used in the chapter
- **status**: Enum (draft, review, published) - The current status of the chapter
- **lastUpdated**: DateTime (required) - When the chapter was last modified
- **author**: String (optional) - The author who wrote this chapter

### Example
- **id**: String (required) - Unique identifier for the example
- **name**: String (required) - Descriptive name for the example
- **description**: String (required) - What the example demonstrates
- **language**: String (required) - Programming language or type of example
- **code**: String (required) - The actual code or example content
- **chapterId**: String (required) - Reference to the chapter this example belongs to
- **testScript**: String (optional) - Optional test script to validate the example
- **status**: Enum (draft, review, published) - The current status of the example

### Reference
- **id**: String (required) - Unique identifier for the reference
- **citation**: String (required) - Full citation in APA format
- **title**: String (required) - Title of the referenced work
- **authors**: [String] (required) - Authors of the referenced work
- **year**: Integer (required) - Year of publication
- **sourceType**: Enum (book, article, website, paper, documentation) - Type of source
- **url**: String (optional) - URL if available
- **chapterId**: String (required) - Reference to the chapter where this is cited
- **context**: String (required) - How this reference is used in the chapter

### Specification
- **id**: String (required) - Unique identifier for the specification
- **title**: String (required) - Title of the specification
- **type**: Enum (module, chapter, book) - The type of specification
- **targetId**: String (required) - ID of the entity being specified
- **content**: String (required) - The specification details in Markdown
- **status**: Enum (draft, approved, deprecated) - Current status of the specification
- **createdDate**: DateTime (required) - When the specification was created
- **reviewers**: [String] (optional) - List of reviewers for the specification

## Relationships

- A Book has many Modules (composition relationship)
- A Module belongs to one Book and has many Chapters (composition relationship)
- A Chapter belongs to one Module and has many Examples and References
- Examples and References belong to one Chapter
- Specifications can target Books, Modules, or Chapters

## Validation Rules

1. **Book Validation**:
   - Must have at least one module
   - Title must be between 5-100 characters
   - Version must follow semantic versioning format

2. **Module Validation**:
   - Must have between 1-10 chapters (based on spec requirements)
   - Must have a unique order value within the book
   - Must have 1-5 learning objectives
   - Title must be between 5-50 characters

3. **Chapter Validation**:
   - Must have between 1-3 learning objectives
   - Content must be valid Markdown/MDX
   - Must have a unique order value within the module
   - Should not exceed 2000 words (per Constitution Principle IV)

4. **Example Validation**:
   - Must be associated with a valid chapter
   - Code must be valid for the specified language
   - Must include a test script if language supports execution

5. **Reference Validation**:
   - Must follow APA format
   - Must be cited in the associated chapter
   - Source type must match the actual referenced material

## State Transitions

### Chapter States
- `draft` → `review` (when initial writing is complete)
- `review` → `draft` (when changes are requested)
- `review` → `published` (when approved by reviewers)
- `published` → `review` (when updates are needed)

### Module States
- `draft` → `review` (when all chapters are in review or published state)
- `review` → `draft` (when major changes are required)
- `review` → `published` (when all chapters are published)
- `published` → `review` (when updates are needed)

### Book States
- `draft` → `review` (when all modules are in review or published state)
- `review` → `draft` (when major changes are required)
- `review` → `published` (when all modules are published)
- `published` → `review` (when updates are needed)