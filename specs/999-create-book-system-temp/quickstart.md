# Quickstart Guide: AI/Spec-Driven Book Creation System

## Prerequisites

Before you begin, ensure you have the following installed on your Ubuntu system:
- Node.js (v18 or higher)
- Yarn package manager
- Git version control system
- Access to Claude Code with Gemini API

## Setting Up the Environment

1. Clone or create your book repository:
   ```bash
   git clone <your-repo-url>
   cd <your-repo-name>
   ```

2. Install Docusaurus dependencies:
   ```bash
   yarn install
   ```

3. Verify your Claude Code and Gemini API setup by testing the integration with a simple prompt.

## Creating Your First Module

1. Navigate to the docs directory:
   ```bash
   cd docs
   ```

2. Create a new module directory:
   ```bash
   mkdir module-1-introduction
   ```

3. Create your first chapter:
   ```bash
   touch module-1-introduction/chapter-1-what-book-about.md
   ```

4. Add content to your chapter file, following the required structure defined in the Constitution:
   - Learning Objectives
   - Prerequisites
   - Content (Introduction → Core Concepts → Examples → Applications)
   - Summary
   - Exercises
   - References

## Using Spec-Kit Plus for Chapter Creation

1. Define your chapter specification first using the Spec-Kit template:
   ```bash
   /sp.specify "Create chapter specification for [chapter topic]"
   ```

2. Generate your implementation plan:
   ```bash
   /sp.plan "Plan chapter on [topic]"
   ```

3. Break down your tasks:
   ```bash
   /sp.tasks "Create tasks for [chapter topic]"
   ```

## Generating Content with AI

1. With your chapter specification ready, use Claude Code to generate content based on your spec:
   ```bash
   # Example prompt to AI
   "Based on the specification in [spec file path], write the introduction section for [chapter name]"
   ```

2. Review and refine the generated content, ensuring it meets the Constitution requirements for accuracy and educational clarity.

## Building and Testing Your Book

1. To build your book locally:
   ```bash
   yarn build
   ```

2. To run in development mode with hot-reloading:
   ```bash
   yarn start
   ```

3. To run quality validation checks:
   ```bash
   # Check for broken links
   yarn lint-links
   
   # Validate build
   yarn build
   
   # Run accessibility checks
   # Implementation depends on specific tools chosen
   ```

## Deployment

1. Push your changes to the main branch:
   ```bash
   git add .
   git commit -m "Add new chapter content"
   git push origin main
   ```

2. GitHub Actions will automatically deploy your updated book to GitHub Pages.

## Quality Validation

Before publishing, ensure your content meets the following standards:
- Content accuracy and technical rigor (Constitution Principle I)
- Educational clarity and accessibility (Constitution Principle II)
- Consistency with terminology and standards (Constitution Principle III)
- Docusaurus structure and quality requirements (Constitution Principle IV)
- Code example quality standards (Constitution Principle V)
- Deployment and publishing standards (Constitution Principle VI)

For detailed guidance on any of these steps, refer to the corresponding chapters in your book-in-progress.