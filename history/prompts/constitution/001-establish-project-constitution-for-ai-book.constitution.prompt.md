---
id: 001
title: Establish Project Constitution for AI Book
stage: constitution
date: 2025-12-05
surface: agent
model: gemini-pro
feature: none
branch: main
user: unknown
command: /sp.constitution
labels: [constitution, setup]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
 - .specify/templates/plan-template.md
tests:
 - none
---

## Prompt

/sp.constitution # /sp.constitution

## Project
AI/Spec-Driven Book Creation for Hackathon  
Using:  
- **Gemini CLI** (primary AI writing + editing engine)  
- **Spec-Kit Plus (MCP-based workflow)**  
- **Docusaurus v3** for building the book  
- **GitHub Pages** for final deployment  
- **AI-Native, Spec-Driven Development Workflow**

---

## Core Principles
1. **Specification-First Development**  
   - All chapters, pages, and content must originate from formal specs written in Spec-Kit Plus.  
   - No section is written without a prior spec.

2. **Gemini AI–Native Workflow**  
   - Gemini CLI will handle writing, structuring, editing, optimization, and transformation of content.  
   - MCP + Spec-Kit Plus act as the governance layer.

3. **Clarity & Developer-Focused Writing**  
   - Content must be easy for beginners and intermediate developers to understand.  
   - Every concept should be practical and actionable.

4. **Technical Accuracy**  
   - All information must be factual, verifiable, and aligned with real documentation.  
   - Zero tolerance for hallucinations.

5. **Consistency & Professional Quality**  
   - Tone, formatting, structure, and page layout must remain consistent.

---

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

---

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

---

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

## Response snapshot

I have updated the project constitution at `.specify/memory/constitution.md` with the content you provided.

- Set version to `1.0.0`.
- Added a sync impact report to the top of the file.
- Updated `.specify/templates/plan-template.md` to include a checklist based on the new constitution's core principles.

## Outcome

- ✅ Impact: High. Established the foundational principles and standards for the project.
- 🧪 Tests: None run.
- 📁 Files: 2 files modified (`.specify/memory/constitution.md`, `.specify/templates/plan-template.md`).
- 🔁 Next prompts: User can now proceed with other commands like `/sp.specify` or `/sp.plan`.
- 🧠 Reflection: The process of updating the constitution and propagating the changes to the plan template ensures that all future work will be aligned with the project's core principles.

## Evaluation notes (flywheel)

- Failure modes observed: `git` command failed, likely due to an uninitialized repository. Handled by using a default branch name.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment: n/a
