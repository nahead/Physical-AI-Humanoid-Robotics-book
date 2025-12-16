<!--
---
Sync Impact Report
---
- **Version Change**: Initial setup -> 1.0.0
- **Change Rationale**: Initial project constitution established from user-provided principles for the AI book creation project.
- **Modified Principles**:
    - Replaced template principles with 5 new core principles.
- **Added Sections**:
    - Key Standards
    - Project Constraints
    - Success Criteria
- **Removed Sections**:
    - Removed generic template sections.
- **Templates Requiring Updates**:
    - ✅ .specify/templates/plan-template.md
    - ✅ .specify/templates/spec-template.md
    - ✅ .specify/templates/tasks-template.md
- **Follow-up TODOs**:
    - None
-->
# AI/Spec-Driven Book Creation for Hackathon Constitution

## Core Principles

### 1. Specification-First Development
All chapters, pages, and content must originate from formal specs written in Spec-Kit Plus. No section is written without a prior spec.

### 2. Gemini AI–Native Workflow
Gemini CLI will handle writing, structuring, editing, optimization, and transformation of content. MCP + Spec-Kit Plus act as the governance layer.

### 3. Clarity & Developer-Focused Writing
Content must be easy for beginners and intermediate developers to understand. Every concept should be practical and actionable.

### 4. Technical Accuracy
All information must be factual, verifiable, and aligned with real documentation. Zero tolerance for hallucinations.

### 5. Consistency & Professional Quality
Tone, formatting, structure, and page layout must remain consistent.

## Key Standards

### Content Standards
- **Citation Style:** Simple inline links (official docs preferred)
- **Source Types Allowed:**
  - Official documentation
  - GitHub repositories
  - Reputable technical blogs
  - AI/Tools documentation (Gemini, MCP, Docusaurus, etc.)

### Writing Standards
- Readability target: Grade 8–10
- Simple vocabulary, short sentences
- Prefer step-by-step instructions
- All code examples must be functional
- No unnecessary jargon
- All examples must be reproducible by readers

### Structural Standards
- **Docusaurus v3** with MDX pages
- Auto-generated sidebar structure
- Responsive design with dark/light mode
- SEO metadata on all pages
- GitHub Pages deployment pipeline
- Build must pass without warnings

## Project Constraints
- **Chapters:** 8–12
- **Total Word Count:** 10,000–18,000 words
- **Each Chapter Must Include:**
  - Summary
  - Real, working code examples
  - A hands-on activity/task
  - A checklist
- **Mandatory Tools:**
  - Gemini CLI
  - Spec-Kit Plus CLI
  - MCP workflow
  - Docusaurus
  - GitHub + GitHub Actions

### Build Constraints
- Zero broken links
- No missing images or assets
- Code blocks must run without errors
- GitHub Actions must pass successfully

## Success Criteria
- All content generated and maintained via **Gemini CLI + Spec-Kit Plus** workflow.
- Entire book deploys correctly using Docusaurus on GitHub Pages.
- Navigation, sidebar, theming, and pages work flawlessly.
- All code examples are real, correct, and functional.
- Book is clear, readable, and beginner-friendly.
- Final hackathon submission must include:
  - GitHub repository
  - Live Docusaurus site
  - Optional PDF export

## Governance
This Constitution is the single source of truth for project principles and standards. All development, content creation, and reviews must adhere to it. Amendments require review and documentation to ensure project integrity.

**Version**: 1.0.0 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-05