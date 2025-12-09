---
id: 1
title: update-project-constitution
stage: constitution
date: 2025-12-09
model: Qwen
feature: none
branch: main
user: unknown
command: /sp.constitution
labels: ["constitution", "update", "governance"]
links: {"spec": null, "ticket": null, "adr": null, "pr": null}
files: ["./specify/memory/constitution.md", "./specify/templates/plan-template.md", "./specify/templates/spec-template.md", "./specify/templates/tasks-template.md"]
tests: []
outcome: "Successfully updated the project constitution from template to Physical AI Humanoid Robotics Textbook Constitution, and updated dependent templates to maintain consistency."
evaluation: "The constitution was successfully updated with all required principles, sections, and governance policies. Dependent templates were updated to align with the new constitution. Validation checks passed with no remaining placeholder tokens."
---

# User Input

```
---
description: Create or update the project constitution from interactive or provided principle inputs, ensuring all dependent templates stay in sync
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

You are updating the project constitution at `.specify/memory/constitution.md`. This file is a TEMPLATE containing placeholder tokens in square brackets (e.g. `[PROJECT_NAME]`, `[PRINCIPLE_1_NAME]`). Your job is to (a) collect/derive concrete values, (b) fill the template precisely, and (c) propagate any amendments across dependent artifacts.

Follow this execution flow:

1. Load the existing constitution template at `.specify/memory/constitution.md`.
   - Identify every placeholder token of the form `[ALL_CAPS_IDENTIFIER]`.
   **IMPORTANT**: The user might require less or more principles than the ones used in the template. If a number is specified, respect that - follow the general template. You will update the doc accordingly.

2. Collect/derive values for placeholders:
   - If user input (conversation) supplies a value, use it.
   - Otherwise infer from existing repo context (README, docs, prior constitution versions if embedded).
   - For governance dates: `RATIFICATION_DATE` is the original adoption date (if unknown ask or mark TODO), `LAST_AMENDED_DATE` is today if changes are made, otherwise keep previous.
   - `CONSTITUTION_VERSION` must increment according to semantic versioning rules:
     - MAJOR: Backward incompatible governance/principle removals or redefinitions.
     - MINOR: New principle/section added or materially expanded guidance.
     - PATCH: Clarifications, wording, typo fixes, non-semantic refinements.
   - If version bump type ambiguous, propose reasoning before finalizing.

3. Draft the updated constitution content:
   - Replace every placeholder with concrete text (no bracketed tokens left except intentionally retained template slots that the project has chosen not to define yet—explicitly justify any left).
   - Preserve heading hierarchy and comments can be removed once replaced unless they still add clarifying guidance.
   - Ensure each Principle section: succinct name line, paragraph (or bullet list) capturing non‑negotiable rules, explicit rationale if not obvious.
   - Ensure Governance section lists amendment procedure, versioning policy, and compliance review expectations.

4. Consistency propagation checklist (convert prior checklist into active validations):
   - Read `.specify/templates/plan-template.md` and ensure any "Constitution Check" or rules align with updated principles.
   - Read `.specify/templates/spec-template.md` for scope/requirements alignment—update if constitution adds/removes mandatory sections or constraints.
   - Read `.specify/templates/tasks-template.md` and ensure task categorization reflects new or removed principle-driven task types (e.g., observability, versioning, testing discipline).
   - Read each command file in `.specify/templates/commands/*.md` (including this one) to verify no outdated references (agent-specific names like CLAUDE only) remain when generic guidance is required.
   - Read any runtime guidance docs (e.g., `README.md`, `docs/quickstart.md`, or agent-specific guidance files if present). Update references to principles changed.

5. Produce a Sync Impact Report (prepend as an HTML comment at top of the constitution file after update):
   - Version change: old → new
   - List of modified principles (old title → new title if renamed)
   - Added sections
   - Removed sections
   - Templates requiring updates (✅ updated / ⚠ pending) with file paths
   - Follow-up TODOs if any placeholders intentionally deferred.

6. Validation before final output:
   - No remaining unexplained bracket tokens.
   - Version line matches report.
   - Dates ISO format YYYY-MM-DD.
   - Principles are declarative, testable, and free of vague language ("should" → replace with MUST/SHOULD rationale where appropriate).

7. Write the completed constitution back to `.specify/memory/constitution.md` (overwrite).

8. Output a final summary to the user with:
   - New version and bump rationale.
   - Any files flagged for manual follow-up.
   - Suggested commit message (e.g., `docs: amend constitution to vX.Y.Z (principle additions + governance update)`).

Formatting & Style Requirements:

- Use Markdown headings exactly as in the template (do not demote/promote levels).
- Wrap long rationale lines to keep readability (<100 chars ideally) but do not hard enforce with awkward breaks.
- Keep a single blank line between sections.
- Avoid trailing whitespace.

If the user supplies partial updates (e.g., only one principle revision), still perform validation and version decision steps.

If critical info missing (e.g., ratification date truly unknown), insert `TODO(<FIELD_NAME>): explanation` and include in the Sync Impact Report under deferred items.

Do not create a new template; always operate on the existing `.specify/memory/constitution.md` file.

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage:
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.


/sp.constitution <!--
SYNC IMPACT REPORT - Constitution v1.0.0

Version Change: [INITIAL] → 1.0.0 (Initial ratification)

Modified Principles:
  - [NEW] I. Content Accuracy & Technical Rigor
  - [NEW] II. Educational Clarity & Accessibility
  - [NEW] III. Consistency & Standards (NON-NEGOTIABLE)
  - [NEW] IV. Docusaurus Structure & Quality
  - [NEW] V. Code Example Quality
  - [NEW] VI. Deployment & Publishing Standards

Added Sections:
  - Content Development Workflow
  - Quality Gates & Review Process

Removed Sections: None (initial version)

Templates Requiring Updates:
  ✅ spec-template.md - Aligned (user stories for chapters/sections)
  ✅ plan-template.md - Aligned (constitution check gates added)
  ✅ tasks-template.md - Aligned (phases match content workflow)

Follow-up TODOs: None
-->

# Physical AI Humanoid Robotics Textbook Constitution

## Core Principles

### I. Content Accuracy & Technical Rigor

Every technical claim, formula, code example, and research reference MUST be accurate and verifiable.

**Rules**:
- All mathematical equations and physics principles MUST be validated against authoritative sources
- Code examples MUST be tested and functional (no pseudocode unless explicitly marked)
- Citations REQUIRED for research findings, algorithms, and external concepts using standard academic format
- Technical claims require either: (a) citation, (b) derivation/proof, or (c) experimental validation
- Version specifications REQUIRED for all software dependencies and APIs
- No speculative or unverified claims about hardware capabilities, safety limits, or performance

**Rationale**: Inaccurate technical content in educational material damages learner trust and creates downstream errors in student implementations. Robotics combines physics, mathematics, and software - errors compound across domains.

### II. Educational Clarity & Accessibility

Content MUST progress logically from fundamentals to advanced topics with clear learning pathways.

**Rules**:
- Each chapter/section MUST declare explicit prerequisites (prior chapters or external knowledge)
- Complex concepts MUST be introduced via: (1) motivation/context, (2) simple example, (3) formal definition, (4) practical application
- Target audience explicitly defined (undergraduate, graduate, practitioners - specify per section if mixed)
- Learning objectives MUST be measurable and stated at chapter start
- Include at least one worked example per major concept
- Diagrams REQUIRED for spatial concepts, system architectures, and multi-step processes
- Glossary terms linked on first use in each chapter

**Rationale**: Humanoid robotics spans mechanical, electrical, and software domains. Without careful scaffolding, readers get lost in abstraction gaps. Clear progression enables self-directed learning.

### III. Consistency & Standards (NON-NEGOTIABLE)

Uniform terminology, formatting, and structure across all content.

**Rules**:
- Terminology consistency enforced via `docs/glossary.md` (single source of truth)
- Code formatting: Language-specific standards (e.g., PEP 8 for Python, ROS 2 conventions for robotics code)
- Chapter structure MUST follow template:
  1. Learning Objectives
  2. Prerequisites
  3. Content (Introduction → Core Concepts → Examples → Applications)
  4. Summary
  5. Exercises (at least 3: conceptual, computational, implementation)
  6. References
- Voice: Second person for tutorials ("you will implement"), third person for theory ("the system computes")
- Notation: Mathematical symbols defined in `docs/notation.md` and used consistently
- Units: SI units unless domain-specific convention (e.g., degrees for joint angles, explicitly noted)

**Rationale**: Inconsistency creates cognitive load. Students spend mental energy reconciling notation/terminology instead of learning concepts. Standards enable scalable collaboration.

### IV. Docusaurus Structure & Quality

Documentation site MUST be navigable, searchable, and maintainable.

**Rules**:
- One concept per page (granular, linkable content - max 2000 words per page)
- Sidebar organization: Hierarchical by complexity (Fundamentals → Intermediate → Advanced → Specialized Topics)
- Metadata REQUIRED: `title`, `description`, `keywords`, `sidebar_position` in every `.md` frontmatter
- Internal links use relative paths: `[text](../path/file.md)` not absolute URLs
- Assets in `/static/img/[chapter-name]/` with descriptive names: `inverse-kinematics-diagram.svg` not `fig1.png`
- Alt text MANDATORY for accessibility (describe diagram content, not "image of robot")
- Search optimization: Keywords in headings, first paragraph, and metadata

**Rationale**: Docusaurus builds static sites - poor organization creates fragile cross-references. Discoverability depends on metadata and search. Accessibility is non-negotiable for educational content.

### V. Code Example Quality

All code MUST be runnable, well-documented, and pedagogically sound.

**Rules**:
- Language specification in fenced blocks: ```python not ```
- Complete examples (not fragments) unless explicitly marked as "excerpt from [full file path]"
- Comments explain WHY not WHAT (assume reader knows language syntax)
- Dependencies listed with versions: `# Requires: numpy>=1.24.0, robotics-toolbox-python==1.1.0`
- Repository structure: `/examples/[chapter-name]/[example-name]/` with README explaining purpose and usage
- Test coverage: Each example includes validation script or test case
- Safety warnings: Hardware-interacting code MUST include safety comments (e.g., "ensure e-stop accessible")
- Prefer standard libraries and widely-adopted packages (ROS 2, NumPy, PyTorch) over obscure dependencies

**Rationale**: Students learn by running and modifying code. Non-functional examples break trust. Robotics code can control physical systems - safety context is critical. Version pinning prevents "works on my machine" issues.

### VI. Deployment & Publishing Standards

Published content MUST build successfully, load quickly, and meet quality gates.

**Rules**:
- `main` branch: Production-ready content only (deployed to GitHub Pages)
- Feature branches: `chapter/[name]` or `fix/[issue]` with PR-based review
- Build gates (MUST pass before merge):
  - Docusaurus build completes without errors or warnings
  - Broken links checker passes (internal and external links)
  - Spell check passes (technical terms in dictionary)
  - Images optimized (< 500KB per file, use `svgo` for SVGs)
- Performance targets:
  - Initial page load: < 3 seconds (Lighthouse)
  - Largest Contentful Paint (LCP): < 2.5s
  - Cumulative Layout Shift (CLS): < 0.1
- SEO requirements:
  - Open Graph tags for social sharing
  - Sitemap generated automatically
  - robots.txt configured
- Versioning: Major book revisions tagged as releases (v1.0, v2.0)
- Redirects: Deprecated URLs MUST redirect to updated content (maintain in `docusaurus.config.js`)

**Rationale**: GitHub Pages deployment is automated - build failures block publishing. Slow page loads harm user experience on all devices. Broken links damage credibility. Performance and SEO determine discoverability.

## Content Development Workflow

### Spec-Driven Chapter Creation

**Process**:
1. **Specification** (`/sp.specify`): Define chapter scope, learning objectives, prerequisites, key concepts
2. **Planning** (`/sp.plan`): Outline structure, identify diagrams needed, plan code examples, research sources
3. **Task Breakdown** (`/sp.tasks`): Decompose into writing tasks (intro, concept sections, examples, exercises)
4. **Implementation**: Write content following constitution principles
5. **Review**: Technical accuracy review + peer review for clarity
6. **Publishing**: Build validation → PR → Merge to main → Auto-deploy

**Artifacts**:
- `specs/[chapter-name]/spec.md` - Chapter requirements and learning objectives
- `specs/[chapter-name]/plan.md` - Content structure and resource plan
- `specs/[chapter-name]/tasks.md` - Granular writing tasks
- `history/prompts/[chapter-name]/` - AI collaboration records (PHRs)
- `history/adr/` - Architectural decisions (e.g., framework choice, chapter organization)

**Content Types**:
- **Theory Chapters**: Mathematical foundations, algorithms, proofs
- **Implementation Chapters**: Code walkthroughs, system integration
- **Application Chapters**: Case studies, real-world examples
- **Reference Chapters**: API docs, hardware specs, datasets

### Architectural Decision Records (ADR)

**Triggers** (suggest ADR creation when ALL three conditions met):
- **Impact**: Long-term consequences (e.g., choosing ROS 2 vs. custom middleware, simulation framework)
- **Alternatives**: Multiple viable options with significant tradeoffs
- **Scope**: Decision affects multiple chapters or overall book structure

**Examples of ADR-worthy decisions**:
- Selection of robotics framework/ecosystem (ROS 2, Isaac Sim, etc.)
- Chapter organization strategy (by subsystem vs. by complexity)
- Code language choice (Python vs. C++ for examples)
- Simulation vs. hardware-first pedagogical approach

**Process**: Suggest via "📋 Architectural decision detected: [brief]. Document reasoning? Run `/sp.adr [title]`" - wait for user consent.

## Quality Gates & Review Process

### Pre-Merge Gates (NON-NEGOTIABLE)

All pull requests MUST pass:
1. **Build Validation**: Docusaurus build succeeds without warnings
2. **Link Check**: No broken internal/external links
3. **Technical Review**: Domain expert validates accuracy (formulas, code, claims)
4. **Peer Review**: Another contributor checks clarity and consistency
5. **Accessibility Check**: Alt text present, heading hierarchy correct, contrast ratios meet WCAG AA
6. **Performance Check**: Lighthouse score ≥ 90 for performance, accessibility, SEO

### Content Review Criteria

**Technical Accuracy**:
- [ ] Formulas validated against cited sources
- [ ] Code examples tested and functional
- [ ] Hardware specifications current and cited
- [ ] Safety considerations addressed for physical systems

**Educational Quality**:
- [ ] Learning objectives measurable and addressed
- [ ] Prerequisites clearly stated
- [ ] Progression from simple to complex
- [ ] Examples clarify concepts (not just demonstrate syntax)

**Consistency**:
- [ ] Terminology matches glossary
- [ ] Notation matches standards document
- [ ] Chapter structure follows template
- [ ] Code formatting follows style guide

**Production Quality**:
- [ ] Images optimized and have alt text
- [ ] Metadata complete
- [ ] Cross-references accurate
- [ ] No placeholder text (TODO, TBD, etc.)

### Review Roles

- **Technical Reviewer**: Validates accuracy (robotics domain expert)
- **Peer Reviewer**: Checks clarity and educational value (target audience perspective)
- **Editor**: Enforces consistency and standards
- **Maintainer**: Final approval and merge authority

## Governance

**Constitution Authority**: This document supersedes all other development practices. All PRs, content reviews, and planning decisions MUST verify compliance with these principles.

**Amendment Process**:
1. Proposed changes documented in issue with rationale
2. Impact analysis on existing content and workflow
3. Team discussion and approval
4. Migration plan for existing content (if needed)
5. Version increment following semantic versioning
6. Update dependent templates and documentation

**Compliance Verification**:
- All PRs include checklist mapping changes to constitution principles
- Quarterly audits of published content for consistency drift
- Template updates propagated within 1 week of constitution amendment
- Violations require documented justification (complexity tracking in plan.md)

**Complexity Justification**: Any deviation from constitution principles (e.g., non-standard notation for domain-specific reasons, external dependencies, non-SI units) MUST be documented in relevant plan.md with:
- What principle is violated
- Why deviation necessary (specific requirement)
- What simpler alternative was rejected and why

**Runtime Guidance**: Use `CLAUDE.md` for AI assistant behavior and workflow execution. Constitution defines WHAT we build; CLAUDE.md defines HOW we collaborate with AI agents.

**Version**: 1.0.0 | **Ratified**: 2025-11-29 | **Last Amended**: 2025-12-07
```

# Response

I've successfully updated the project constitution as requested. Here's a summary of the changes made:

1. **Constitution Update**:
   - Replaced the template placeholders with the Physical AI Humanoid Robotics Textbook Constitution
   - Added all 6 core principles with detailed rules and rationales
   - Included Content Development Workflow and Quality Gates & Review Process sections
   - Added Governance section with amendment procedures
   - Added an HTML sync impact report at the top of the file

2. **Template Updates**:
   - Updated plan-template.md to include constitution check gates aligned with the new principles
   - Updated spec-template.md to include constitution compliance requirements
   - Updated tasks-template.md to include constitution compliance tasks in the foundational phase

3. **Version Information**:
   - Set Version to 1.0.0
   - Set Ratification Date to 2025-11-29
   - Set Last Amended Date to 2025-12-09 (today)

All dependent templates have been updated to align with the new constitution, ensuring consistency across the project's documentation framework. The constitution now serves as the authoritative guide for the Physical AI Humanoid Robotics Textbook project with specific principles focused on content accuracy, educational clarity, consistency, Docusaurus structure, code example quality, and deployment standards.

The constitution is now ready for use and will guide all future development and content creation decisions for the project.