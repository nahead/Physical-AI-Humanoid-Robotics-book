# Tasks: RAG Chatbot with Gemini, Supabase, and Next.js

**Input**: Design documents from `specs/001-rag-chatbot/`
**Prerequisites**: `plan.md` (required), `spec.md` (required for user stories)

**Tests**: Test tasks are included and recommended for a robust implementation.

**Organization**: Tasks are grouped by phase, aligning with the updated implementation plan.

## Format: `- [ ] [ID] [P?] [Story] Description with file path`

-   **[P]**: Can run in parallel (different files, no dependencies)
-   **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
-   Include exact file paths in descriptions

## Path Conventions

-   **Backend**: `nextjs-backend/`
-   **Frontend**: `docusaurus/`

## Phase 1: Setup & Environment

**Purpose**: Project initialization and environment setup for all services.

-   [x] T001 Create `nextjs-backend/` directory for the Next.js application.
-   [x] T002 [P] Initialize Next.js project with TypeScript in `nextjs-backend/`.
-   [x] T003 [P] Install core dependencies (`@supabase/supabase-js`, `@qdrant/js-client-rest`, `@google/generative-ai`) in `nextjs-backend/`.
-   [x] T004 [P] Run `quickstart.md` steps to set up Supabase, Qdrant, and Gemini API keys.
-   [x] T005 Create `.env.local` file in `nextjs-backend/` and populate it with credentials from the quickstart guide.

---

## Phase 2: Foundational Backend (Authentication & DB Setup)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented. This phase covers the backend part of **User Story 4**.

**⚠️ CRITICAL**: No other work can begin until this phase is complete.

-   [x] T006 [US4] Initialize Supabase client in `nextjs-backend/lib/supabase.ts`.
-   [x] T007 [US4] Create database schema for `profiles`, `chat_sessions`, and `messages` in the Supabase dashboard using the SQL editor, based on `data-model.md`.
-   [x] T008 [P] [US4] Implement user registration endpoint in `nextjs-backend/pages/api/auth/register.ts`.
-   [x] T009 [P] [US4] Implement user login endpoint in `nextjs-backend/pages/api/auth/login.ts`.
-   [x] T010 [P] [US4] Implement user logout endpoint in `nextjs-backend/pages/api/auth/logout.ts`.
-   [x] T011 [P] Set up API route structure and a global error handler for `nextjs-backend/`.
-   [x] T012 Define TypeScript types for all API requests/responses in `nextjs-backend/types/index.ts`.

**Checkpoint**: Foundational backend with authentication is ready.

---

## Phase 3: User Story 1 - Document Indexing and General Chat

**Goal**: Enable indexing of book content and provide general chat functionality with optional translation.

**Independent Test**: Verify indexing endpoint populates Qdrant and the general chat returns cited, translated answers.

### Implementation for User Story 1

-   [x] T013 [P] [US1] Implement document loader to read markdown files from `docusaurus/docs/` in `nextjs-backend/lib/document_loader.ts`.
-   [ ] T014 [P] [US1] Implement text chunking strategy suitable for Gemini embeddings in `nextjs-backend/lib/chunker.ts`.
-   [x] T015 [P] [US1] Initialize Qdrant client in `nextjs-backend/lib/qdrant.ts`.
-   [x] T016 [US1] Initialize Gemini API client in `nextjs-backend/lib/gemini.ts`.
-   [x] T017 [US1] Implement Gemini embedding generation service in `nextjs-backend/lib/gemini_embeddings.ts`.
-   [x] T018 [US1] Implement indexing service to store chunks and metadata in Qdrant in `nextjs-backend/lib/indexer.ts` (depends on T013-T017).
-   [x] T019 [US1] Create `/index` API endpoint in `nextjs-backend/pages/api/index.ts` to trigger the indexing process.
-   [x] T020 [P] [US1] Implement retrieval service to query Qdrant for relevant chunks in `nextjs-backend/lib/retriever.ts`.
-   [x] T021 [P] [US1] Implement RAG context assembly and chat completion logic using Gemini API in `nextjs-backend/lib/gemini_chat.ts`.
-   [x] T022 [P] [US1] Implement Urdu translation service using Gemini API in `nextjs-backend/lib/gemini_translate.ts`.
-   [x] T023 [US1] Implement the main chat endpoint in `nextjs-backend/pages/api/chat.ts` (depends on T020-T022).

**Checkpoint**: Indexing and general chat APIs are functional.

---

## Phase 4: User Story 2 - Selected Text Q&A

**Goal**: Allow users to ask questions based on a highlighted portion of text.

**Independent Test**: Verify `/chat/selected` endpoint accurately responds using only the provided text.

### Implementation for User Story 2

-   [ ] T024 [US2] Implement the selected text chat endpoint in `nextjs-backend/pages/api/chat/selected.ts`, reusing services from Phase 3 where applicable.

**Checkpoint**: Selected text chat API is functional.

---

## Phase 5: User Story 3 - Integrated Chatbot UI

**Goal**: Embed a fully functional React Chatbot UI into the Docusaurus site.

**Independent Test**: Verify the UI loads, handles user input, displays responses, and supports auth and translation.

### Implementation for User Story 3

-   [ ] T025 [P] [US3] Create the basic React Chatbot UI component in `docusaurus/src/components/Chatbot/index.tsx`.
-   [x] T026 [P] [US3] Implement an API client in the React component to interact with the Next.js backend in `docusaurus/src/components/Chatbot/api.ts`.
-   [x] T027 [US3] Implement chat history display, input handling, and citation rendering in the `Chatbot` component.
-   [x] T028 [P] [US3] Implement language selection UI (for Urdu translation) in the `Chatbot` component.
-   [ ] T029 [US3] Integrate the `Chatbot` component into the Docusaurus layout (e.g., via swizzling `Layout`).
-   [x] T030 [US4] Implement frontend logic for Supabase authentication (login/logout) and session management within the `Chatbot` component and a shared context.
-   [x] T031 [US4] Add UI elements for personalization (e.g., displaying user info, saving chat preferences to Supabase).

**Checkpoint**: Chatbot is fully integrated and usable within the Docusaurus site.

---

## Phase 6: Testing, CI/CD & Deployment

**Purpose**: Ensure the entire system is robust, well-tested, and can be deployed reliably to Vercel and GitHub Pages.

-   [x] T032 [P] Write unit tests for Next.js API services in `nextjs-backend/tests/`.
-   [x] T033 [P] Write integration tests for Next.js API endpoints.
-   [x] T034 [P] Write unit tests for React components in `docusaurus/src/components/Chatbot/`.
-   [x] T035 Set up Vercel project and configure deployment for the `nextjs-backend/` application.
-   [x] T036 Set up GitHub Actions to deploy Docusaurus site to GitHub Pages in `.github/workflows/deploy-docusaurus.yml`.
-   [x] T037 Perform end-to-end testing of the full user flow.

---

## Dependencies & Execution Order

### Phase Dependencies
- **Phase 1 (Setup)**: Can start immediately.
- **Phase 2 (Foundational Backend)**: Depends on Phase 1 completion. BLOCKS all subsequent phases.
- **Phase 3 & 4 (US1 & US2 Backend)**: Depend on Phase 2 completion and can run largely in parallel.
- **Phase 5 (Frontend)**: Depends on Phase 2 for auth and the API contracts from Phase 3 & 4. Can start once auth endpoints are ready.
- **Phase 6 (Testing/Deployment)**: Depends on all implementation phases.

### Implementation Strategy

1.  **Complete Phase 1 & 2**: Set up all environments and get the core backend with authentication working.
2.  **Parallel Backend/Frontend Work**:
    *   **Backend Team**: Work on Phase 3 (Indexing & Chat) and Phase 4 (Selected Text) in parallel.
    *   **Frontend Team**: Work on Phase 5 (UI/Integration) using mocked API responses until the backend is ready.
3.  **Integration & Testing**: Once backend APIs are live, integrate them with the frontend, and begin comprehensive testing (Phase 6).
4.  **Deployment**: Deploy the Next.js backend to Vercel and the Docusaurus site to GitHub Pages.

This approach allows for parallel development streams and ensures a solid foundation before building the core RAG functionality.
