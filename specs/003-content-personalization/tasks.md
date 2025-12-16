# Tasks: Content Personalization in Book Chapters

**Input**: Design documents from `specs/003-content-personalization/`
**Prerequisites**: `plan.md` (required), `spec.md` (required for user stories)

**Tests**: Test tasks are included and recommended for a robust implementation.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `- [ ] [ID] [P?] [Story] Description with file path`

-   **[P]**: Can run in parallel (different files, no dependencies)
-   **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
-   Include exact file paths in descriptions

## Path Conventions

-   **Backend**: `nextjs-backend/`
-   **Frontend**: `docusaurus/`

## Phase 0: Research & Backend API Definition

**Purpose**: Deep dive into Gemini's capabilities for structured content processing and define the personalization API contract.

-   [ ] T001 Research Gemini's capabilities for rephrasing markdown text, simplifying code blocks, and identifying non-textual elements. Document findings in `specs/003-content-personalization/research.md`.
-   [ ] T002 Define the OpenAPI schema for the new `/api/personalize` endpoint in `specs/003-content-personalization/contracts/personalize.yaml`.

---

## Phase 1: Backend Personalization Logic (Text-Only MVP)

**Purpose**: Implement the core personalization backend API endpoint that processes text content using Gemini based on user background.

### Implementation for User Story 1 (Personalize Chapter Content)

-   [x] T003 [US1] Implement `nextjs-backend/lib/personalizer.ts` service: This service will take original content, user's `software_background`, and `hardware_background` and generate personalized text using Gemini API.
-   [x] T004 [US1] Create `/api/personalize` endpoint in `nextjs-backend/src/pages/api/personalize.ts`: This Next.js API route will receive original chapter content and user background, call `personalizer.ts`, and return personalized content.
-   [x] T005 [US1] Implement error handling in `nextjs-backend/src/pages/api/personalize.ts` for Gemini API quota limits (FR-007).

---

## Phase 2: Frontend Integration & Personalization Trigger (Text-Only MVP)

**Purpose**: Create the UI button and integrate it into Docusaurus document pages to trigger and display text-only personalized content.

### Implementation for User Story 1 (Personalize Chapter Content)

-   [ ] T006 [P] [US1] Create `docusaurus/src/components/PersonalizeButton/index.js` React component: This component will handle clicking, loading states, and display.
-   [ ] T007 [US1] Modify `docusaurus/src/theme/DocItem/Layout/index.js`:
    *   Render the `PersonalizeButton` at the start of the chapter (FR-001).
    *   Pass the current chapter content and user profile to the button.
    *   Display a loading indicator during personalization.
    *   Replace original content with personalized text on success (FR-004).
    *   Ensure button visibility/behavior for logged-in vs. anonymous users (FR-006).

### Implementation for User Story 2 (Revert Personalized Content)

-   [ ] T008 [US2] Modify `docusaurus/src/theme/DocItem/Layout/index.js` to implement client-side logic for reverting content to its original state (FR-005), possibly by making the personalization button a toggle.

---

## Phase 3: Handling Non-Textual Elements (Advanced)

**Purpose**: Extend the personalization logic to intelligently handle non-textual elements based on the user's background.

### Implementation for User Story 1 (Personalize Chapter Content)

-   [ ] T009 [US1] Develop content pre-processing logic (frontend or backend) to parse Docusaurus markdown, identify code blocks, diagrams, etc.
-   [ ] T010 [US1] Refine `nextjs-backend/lib/personalizer.ts` to instruct Gemini on how to generate simpler alternatives or augmented explanations for non-textual elements.
-   [ ] T011 [US1] Develop robust frontend logic in `docusaurus/src/theme/DocItem/Layout/index.js` to render the personalized markdown, correctly re-injecting modified non-textual elements or their alternatives.

---

## Phase 4: Testing & CI/CD

**Purpose**: Ensure the new feature is robust, well-tested, and deployable.

-   [ ] T012 Write unit and integration tests for `nextjs-backend/lib/personalizer.ts` and `/api/personalize` endpoint.
-   [ ] T013 Write E2E tests for the personalization user flow (button click, content change, revert).
-   [ ] T014 Update Vercel deployment configuration for the new `/api/personalize` endpoint.

---

## Dependencies & Execution Order

### Phase Dependencies
-   **Phase 0 (Research & API Definition)**: No dependencies.
-   **Phase 1 (Backend Personalization Logic)**: Depends on Phase 0 completion.
-   **Phase 2 (Frontend Integration)**: Depends on Phase 1 completion (specifically the `/api/personalize` endpoint).
-   **Phase 3 (Handling Non-Textual Elements)**: Depends on Phase 1 and 2 (building upon the text-only MVP).
-   **Phase 4 (Testing & CI/CD)**: Depends on all implementation phases.

### User Story Dependencies
-   **User Story 1 (P1 - Personalize Chapter Content)**: Implemented across Phases 1, 2, and 3.
-   **User Story 2 (P2 - Revert Personalized Content)**: Primarily in Phase 2.

### Parallel Opportunities
-   Phase 0 tasks can be worked on in parallel.
-   Once Phase 1 (`/api/personalize` endpoint) is complete, basic frontend integration (Phase 2) can begin in parallel with advanced backend work (Phase 3).

## Implementation Strategy

### Incremental Delivery (Text-Only MVP first)

1.  Complete Phase 0.
2.  Complete Phase 1: Get the backend `/api/personalize` endpoint working for text-only content.
3.  Complete Phase 2: Integrate the button into Docusaurus and display personalized text-only content. This will be the MVP.
4.  Optionally, proceed to Phase 3 for advanced handling of non-textual elements, recognizing its high complexity and potential for iterative refinement.
5.  Complete Phase 4 for testing and deployment.
