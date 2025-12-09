# AI/Spec-Driven Book Creation System

Build, Write & Deploy Books Using Docusaurus

## Overview

This project provides a complete system for creating books using a spec-driven approach with Docusaurus, GitHub Pages, Spec-Kit Plus, and Claude Code. The methodology ensures consistency, quality control, and reproducibility in book creation.

## Features

- **Spec-Driven Methodology**: Create detailed specifications before writing
- **AI-Assisted Writing**: Leverage Claude Code for content generation
- **Docusaurus Integration**: Professional documentation site generation
- **Automated Deployment**: GitHub Pages CI/CD workflow
- **Quality Gates**: Validation at every stage of creation
- **Modular Organization**: 6 comprehensive modules covering all aspects of book creation

## Getting Started

### Prerequisites

- Node.js (v18+)
- Yarn package manager
- Git version control system
- Access to Claude Code with Gemini API

### Installation

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. Install dependencies:
   ```bash
   yarn install
   ```

3. Start the development server:
   ```bash
   yarn start
   ```

4. Visit `http://localhost:3000` to view the book

### Structure

The book is organized into 6 modules:

1. **Introduction & Foundations**: Core concepts and tool overview
2. **Environment Setup**: Docusaurus, Spec-Kit, and Claude Code configuration
3. **Book Engineering**: Specifications, planning, and structure
4. **Writing with AI**: Spec-Kit and Claude Code integration
5. **Publishing & Advanced Techniques**: SEO, search, deployment
6. **Final Project**: Complete implementation example

## Usage

1. Follow the specification-driven approach:
   - Create specifications using `/sp.specify`
   - Plan implementation with `/sp.plan`
   - Generate tasks using `/sp.tasks`
   - Implement with guidance from the book content

2. Use Claude Code prompts from the examples directory to generate content

3. Validate your content using the quality gates

4. Deploy using GitHub Actions workflow

## Scripts

- `yarn start`: Start development server
- `yarn build`: Generate production build
- `yarn serve`: Serve built site locally
- `yarn validate`: Run all validation checks

## Contributing

1. Fork the repository
2. Create a feature branch
3. Add your content following the constitutional principles
4. Submit a pull request with proper validation

## License

[Specify license type if applicable]

## Support

For support, please open an issue in the GitHub repository.