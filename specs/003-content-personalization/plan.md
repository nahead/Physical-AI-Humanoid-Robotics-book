# Implementation Plan: Content Personalization in Book Chapters

**Branch**: `003-content-personalization` | **Date**: 2025-12-08 | **Spec**: [specs/003-content-personalization/spec.md](specs/003-content-personalization/spec.md)
**Input**: Feature specification from `specs/003-content-personalization/spec.md`

## Summary

This plan outlines the implementation for a "Content Personalization" feature that allows logged-in users to dynamically adjust book chapter content based on their defined software and hardware backgrounds. A button at the start of each chapter will trigger the personalization, leveraging the Gemini API for content rephrasing and augmenting, and the existing Supabase user profiles for background data. The system will handle non-textual elements by attempting to provide simpler alternatives for beginners or more detailed explanations for advanced users.

## Technical Context

**Language/Version**: TypeScript/JavaScript (Next.js backend, React frontend for Docusaurus).  
**Primary Dependencies**: Google Generative AI SDK (Gemini API for personalization), existing Supabase Client (user profiles), Next.js (new API endpoint for personalization), React (button component, content display), Docusaurus.  
**Storage**: Existing Supabase Postgres (user `profiles` table to store background data). No new database structures are expected beyond what's already in the profile.  
**Testing**: Jest/React Testing Library (frontend components), Supertest/Jest (Next.js API route), Playwright/Cypress (E2E for user flow).  
**Target Platform**: Vercel (Next.js backend deployment), Web browser (Docusaurus frontend).
**Project Type**: Full-stack web feature.  
**Performance Goals**:
*   Personalized content should be displayed within 10 seconds of clicking the button for 90% of users (SC-002).
*   Reversion to original content should occur within 2 seconds for 99% of users (SC-004).
**Constraints**:
*   ONLY FREE tools (Gemini API, Supabase, Vercel).
*   Personalization must be based on `software_background` and `hardware_background` from user profile.
*   The system must gracefully handle Gemini API quota limits (FR-007).
*   **High Complexity**: Handling non-textual elements (code blocks, diagrams) by providing *simpler alternatives* (as per user's choice Option D) will be highly complex. This might initially be an MVP for text-only, with non-textual elements being preserved or having basic AI explanations.
**Scale/Scope**: Personalization for a single Docusaurus book, serving existing user base.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- [x] **1. Specification-First Development**: This plan originates from the `003-content-personalization` feature spec.
- [x] **2. Gemini AI–Native Workflow**: Gemini API is central to the content personalization logic.
- [x] **3. Clarity & Developer-Focused Writing**: The plan aims to provide clear steps for implementation.
- [x] **4. Technical Accuracy**: The plan relies on existing knowledge of the chosen tech stack.
- [x] **5. Consistency & Professional Quality**: The plan aligns with existing project architecture (Next.js backend, Docusaurus frontend, Supabase auth).

## Project Structure

### Documentation (this feature)

```text
specs/003-content-personalization/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # (Not applicable for this feature, uses existing profile)
├── quickstart.md        # (Not applicable for this feature, uses existing setup)
├── contracts/           # Phase 0/1 output (OpenAPI for personalization endpoint)
│   └── personalize.yaml
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
nextjs-backend/
├── src/
│   ├── pages/api/
│   │   └── personalize.ts       # New API endpoint for content personalization
│   └── lib/
│       └── personalizer.ts      # New service for Gemini interaction for personalization
docusaurus/
├── src/
│   ├── components/
│   │   └── PersonalizeButton/   # New React component for the personalization button
│   └── theme/
│       └── DocItem/
│           └── Layout/index.js  # Modified to integrate button and display personalized content
```

**Structure Decision**: The "Web application" option is retained, extending the existing `nextjs-backend` for the new API endpoint and `docusaurus` for the frontend integration. This maintains architectural consistency.

## Complexity Tracking

*No Constitution Check violations to justify at this point.*

## Phases & Research Approach

The implementation will follow a phased approach, with particular attention to the complexity of handling non-textual elements.

### Phase 0: Research & Backend API Definition

**Objective**: Deep dive into Gemini's capabilities for structured content processing and define the personalization API contract.

**Research Tasks**:
1.  **Gemini's Markdown/Code Processing**: Research Gemini's capabilities and best practices for:
    *   Rephrasing markdown text while preserving formatting (headings, lists).
    *   Simplifying/explaining code blocks.
    *   Modifying diagrams (e.g., explaining them).
    *   Identifying external links that should be preserved or augmented.
    *   **Outcome**: Document best prompt engineering strategies for Option D (simpler alternatives).
2.  **Content Extraction Strategies**: Evaluate robust ways to extract markdown content from Docusaurus pages, including frontmatter and structured text, for sending to the backend.
3.  **API Contract Definition**: Define the OpenAPI schema for the `/api/personalize` endpoint.

**Deliverables**:
*   `research.md` (for `003-content-personalization`): Document findings on Gemini's capabilities for structured content processing and recommended prompt strategies.
*   `contracts/personalize.yaml`: OpenAPI definition for the new `/api/personalize` endpoint.

### Phase 1: Backend Personalization Logic (Text-Only MVP)

**Objective**: Implement the core personalization backend API endpoint that processes text content using Gemini based on user background.

**Tasks**:
1.  Implement `personalizer.ts` service: This service will take original content, user's `software_background`, and `hardware_background` and generate personalized text using Gemini API.
2.  Create `/api/personalize` endpoint: This Next.js API route will:
    *   Receive original chapter content and user background.
    *   Call `personalizer.ts`.
    *   Return personalized content.
    *   Handle Gemini API quota limits (FR-007).

**Deliverables**:
*   `nextjs-backend/src/pages/api/personalize.ts`
*   `nextjs-backend/lib/personalizer.ts`

### Phase 2: Frontend Integration & Personalization Trigger (Text-Only MVP)

**Objective**: Create the UI button and integrate it into Docusaurus document pages to trigger and display text-only personalized content.

**Tasks**:
1.  Create `PersonalizeButton` React component: This component will handle clicking, loading states, and display.
2.  Modify `DocItem/Layout/index.js`:
    *   Render the `PersonalizeButton` at the start of the chapter (FR-001).
    *   Pass the current chapter content and user profile to the button.
    *   Display a loading indicator during personalization.
    *   Replace original content with personalized text on success (FR-004).
    *   Implement client-side logic for reverting content to original state (FR-005).
    *   Ensure button visibility/behavior for logged-in vs. anonymous users (FR-006).

**Deliverables**:
*   `docusaurus/src/components/PersonalizeButton/index.js`
*   Updated `docusaurus/src/theme/DocItem/Layout/index.js`

### Phase 3: Handling Non-Textual Elements (Advanced)

**Objective**: Extend the personalization logic to intelligently handle non-textual elements based on the user's background. This phase acknowledges the high complexity and might be an iterative refinement.

**Tasks**:
1.  **Content Pre-processing**: Develop logic (frontend or backend) to parse Docusaurus markdown, identify code blocks, diagrams, etc.
2.  **Gemini Prompt Enhancement**: Refine `personalizer.ts` to instruct Gemini on how to generate simpler alternatives or augmented explanations for these elements based on the user's background. This might require a structured input format for Gemini.
3.  **Frontend Rendering**: Develop robust frontend logic to render the personalized markdown, correctly re-inserting modified non-textual elements or their alternatives.

**Deliverables**:
*   Updated `nextjs-backend/lib/personalizer.ts`
*   Potentially new markdown parsing/rendering utilities.
*   Updated `docusaurus/src/theme/DocItem/Layout/index.js`

### Phase 4: Testing & CI/CD

**Objective**: Ensure the new feature is robust, well-tested, and deployable.

**Tasks**:
1.  Write unit and integration tests for `personalizer.ts` and `/api/personalize` endpoint.
2.  Write E2E tests for the personalization user flow (button click, content change, revert).
3.  Update Vercel deployment configuration for the new API endpoint.

**Deliverables**:
*   New test files.
*   Updated Vercel config.

## Evaluation and Validation
The feature will be evaluated against the Success Criteria, ensuring content personalization occurs within performance limits and is relevant to the user's background. User feedback will be crucial for SC-003. The handling of non-textual elements (Option D) will be a continuous refinement.