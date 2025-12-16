# Implementation Plan: RAG Chatbot for Physical AI Book

**Branch**: `001-rag-chatbot` | **Date**: 2025-12-06 | **Spec**: [specs/001-rag-chatbot/spec.md](specs/001-rag-chatbot/spec.md)
**Input**: Feature specification from `specs/001-rag-chatbot/spec.md` (updated with Gemini/Supabase/Next.js requirements)

## Summary

This plan outlines the architecture and phased implementation for building a Retrieval-Augmented Generation (RAG) chatbot integrated into the Docusaurus book. The chatbot will provide Q&A capabilities for book content, including targeted responses based on selected text, and support Urdu translation using Gemini. It will serve both anonymous and authenticated users, with personalization managed via Supabase. The technical approach involves a Next.js API backend, Qdrant for vector embeddings, Supabase (Postgres for user/session metadata, Auth for authentication), and a React-based chatbot UI embedded within Docusaurus. All tools will be free-tier compatible.

## Technical Context

**Language/Version**: TypeScript/JavaScript (Next.js backend, React frontend for Docusaurus), Python (for potential initial data processing scripts, though Next.js can handle most).  
**Primary Dependencies**: Next.js (API Routes, React), Supabase Client (Auth, PostgreSQL), Qdrant Client, Google Generative AI SDK (for Gemini API), Better-Auth (or custom auth implementation if Better-Auth is a concept rather than a library), Docusaurus.  
**Storage**: Qdrant Cloud Free tier (vector embeddings and metadata for document chunks), Supabase Postgres (user data, chat sessions, general application metadata).  
**Testing**: Jest/React Testing Library (frontend), Supertest/Jest (Next.js API routes), Playwright/Cypress (E2E).  
**Target Platform**: Vercel (for Next.js API backend deployment), Web browser (for Docusaurus frontend).
**Project Type**: Full-stack web application.  
**Performance Goals**:
*   Chat response time: < 5 seconds for 95% of queries.
*   Chatbot UI initialization: < 2 seconds on Docusaurus pages.
*   UI interactive elements response: < 500 milliseconds for 99% of user interactions.
*   User authentication (login/logout): < 3 seconds for 95% of attempts.
**Constraints**:
*   ONLY FREE tools: Gemini API, Qdrant Cloud Free Tier, Supabase Free Tier, Better-Auth (if library/concept), Vercel Free Tier.
*   Chatbot must answer using book content ONLY.
*   Embeddings generation and translation via Gemini API.
*   All secrets must be managed via environment variables.
*   No plaintext secrets in code.
*   Chatbot UI must be integrated seamlessly into the Docusaurus site.
*   Follow clean architecture and generate reusable code.
**Scale/Scope**: A single Docusaurus book, serving a moderate number of concurrent users. Focus on the specified RAG functionality, personalization, translation, and integration points.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- [x] **1. Specification-First Development**: This plan directly derives from the updated feature specification.
- [x] **2. Gemini AI–Native Workflow**: The Gemini CLI is being used to generate this plan and will be used for subsequent implementation tasks, with Gemini API for LLM interactions.
- [x] **3. Clarity & Developer-Focused Writing**: The plan aims for clarity and will guide the development team with actionable steps and technical details.
- [x] **4. Technical Accuracy**: The plan relies on established technologies and best practices (Next.js, React, Supabase, Qdrant, Gemini API) and will ensure accuracy through research and testing.
- [x] **5. Consistency & Professional Quality**: The plan prioritizes consistency in code style, architecture, and integration with the existing Docusaurus project.

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-chatbot/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
│   └── openapi.yaml
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
nextjs-backend/          # Next.js application for API routes
├── pages/api/           # API routes (indexing, chat, auth, translation)
├── lib/                 # Shared utilities, database connections, service logic
├── components/          # Reusable React components for backend (e.g., auth forms if Next.js handles them)
├── types/               # TypeScript types
├── .env.local           # Environment variables
└── package.json

docusaurus/
└── src/
    ├── components/
    │   └── Chatbot/     # React Chatbot component
    └── pages/
```

**Structure Decision**: A `nextjs-backend/` directory will house the Next.js application, leveraging its API routes for the backend functionalities. The Docusaurus `src/components/Chatbot/` will continue to contain the React UI. This structure provides a clean separation of concerns, optimizes for Vercel deployment, and supports the use of TypeScript across the full stack.

## Complexity Tracking

*No Constitution Check violations to justify at this point.*

## Phases & Research Approach

The implementation will follow a phased approach, incorporating concurrent research and decision documentation.

### Phase 0: Research & Environment Quickstart

**Objective**: Lay the groundwork for the project by making key technology decisions, setting up essential environment configurations, and creating quickstart guides for core services.

**Research Tasks**:
1.  **Gemini API Best Practices**:
    *   **Context**: Using Gemini API for embeddings and chat completions, including Urdu translation.
    *   **Outcome**: Document optimal usage patterns for Gemini embeddings (e.g., chunk size limits, input limits), chat completions, and translation capabilities. Identify relevant SDK functionalities.
2.  **Supabase Setup and Integration**:
    *   **Context**: Using Supabase for user data, session management, and Better-Auth.
    *   **Outcome**: Research Supabase project setup, database schema design for users/sessions, RLS policies, and integration with Next.js and Better-Auth.
3.  **Next.js API Routes Best Practices**:
    *   **Context**: Building the backend using Next.js API routes.
    *   **Outcome**: Document best practices for structuring API routes, handling environment variables, error management, and serverless function considerations for Vercel.
4.  **Qdrant Cloud Integration (Re-affirmation)**:
    *   **Context**: Qdrant remains the vector DB.
    *   **Outcome**: Briefly re-affirm Qdrant Cloud Free tier suitability and document specific Next.js/TypeScript client integration patterns.
5.  **Better-Auth Integration**:
    *   **Context**: Integrating Better-Auth for authentication.
    *   **Outcome**: Determine if "Better-Auth" refers to a specific library or a concept. If a library, research its Next.js and Supabase integration. If a concept, define a custom authentication strategy using Supabase Auth.
6.  **Quickstart Guides**:
    *   **Context**: Rapid setup for development.
    *   **Outcome**: Create `quickstart.md` with clear, step-by-step instructions for setting up Supabase, Qdrant Cloud, and obtaining Gemini API keys.

**Deliverables**:
*   `research.md`: Documenting decisions and best practices for Gemini, Supabase, Next.js API routes, Qdrant, and Better-Auth.
*   `quickstart.md`: Comprehensive guide for setting up Supabase, Qdrant, and Gemini API keys.

### Phase 1: Backend Setup (Next.js API & Supabase)

**Objective**: Establish the foundational Next.js backend structure, connect to Supabase, and integrate basic authentication.

**Tasks**:
1.  Initialize a new Next.js project (`nextjs-backend/`).
2.  Configure Next.js API routes structure.
3.  Set up Supabase project and client in `nextjs-backend/lib/supabase.ts`.
4.  Define database schema for users and chat sessions in Supabase.
5.  Implement user authentication (registration, login, session management) using Supabase Auth and/or Better-Auth within `nextjs-backend/pages/api/auth/*.ts`.
6.  Configure environment variables for Supabase and Gemini API keys.

**Deliverables**:
*   `nextjs-backend/` with basic Next.js structure.
*   `nextjs-backend/lib/supabase.ts` for Supabase client.
*   `nextjs-backend/pages/api/auth/*.ts` for authentication endpoints.
*   Updated `data-model.md` reflecting Supabase schema.
*   `contracts/openapi.yaml` updated for auth endpoints.

### Phase 2: Indexer & Chunker Implementation (Next.js & Gemini)

**Objective**: Develop the components responsible for processing Docusaurus content, generating Gemini embeddings, and storing them in Qdrant.

**Tasks**:
1.  Implement a document loading mechanism to read Docusaurus markdown files (can be an external script or a Next.js API route processing uploaded files).
2.  Develop a text chunking strategy suitable for Gemini embeddings.
3.  Integrate with Gemini API to generate vector embeddings for each chunk in `nextjs-backend/lib/gemini_embeddings.ts`.
4.  Implement Qdrant client initialization and configuration in `nextjs-backend/lib/qdrant.ts`.
5.  Develop an indexing service that stores chunks and their metadata (source document, page number, etc.) in Qdrant.
6.  Expose an indexing endpoint in Next.js API (`nextjs-backend/pages/api/index.ts`) to trigger the indexing process.

**Deliverables**:
*   `nextjs-backend/lib/gemini_embeddings.ts`
*   `nextjs-backend/lib/qdrant.ts`
*   `nextjs-backend/pages/api/index.ts`
*   Qdrant collection populated with sample document embeddings.

### Phase 3: Chat Endpoints & LLM Integration (Next.js & Gemini)

**Objective**: Implement the core RAG logic, including retrieval from Qdrant, response generation, and Urdu translation using Gemini, exposed via Next.js API endpoints.

**Tasks**:
1.  Implement a retrieval service that queries Qdrant with user input embeddings to find relevant document chunks in `nextjs-backend/lib/retriever.ts`.
2.  Integrate with Gemini API for chat completions and RAG context assembly in `nextjs-backend/lib/gemini_chat.ts`.
3.  Implement Urdu translation using Gemini API within the chat response flow or as a separate translation utility in `nextjs-backend/lib/gemini_translate.ts`.
4.  Develop the main chat endpoint (`nextjs-backend/pages/api/chat.ts`) that takes a user query and returns a cited response, potentially translated.
5.  Develop the selected text chat endpoint (`nextjs-backend/pages/api/chat/selected.ts`) that takes a user query and selected text, returning a response based only on the provided text.

**Deliverables**:
*   `nextjs-backend/lib/retriever.ts`
*   `nextjs-backend/lib/gemini_chat.ts`
*   `nextjs-backend/lib/gemini_translate.ts`
*   `nextjs-backend/pages/api/chat.ts`
*   `nextjs-backend/pages/api/chat/selected.ts`

### Phase 4: Frontend Chatbot UI & Docusaurus Integration

**Objective**: Build the interactive React chatbot component and integrate it seamlessly into the Docusaurus site, supporting personalization and translation.

**Tasks**:
1.  Develop a React component for the chatbot UI in `docusaurus/src/components/Chatbot/index.tsx`, including input field, chat history display, citation rendering, and a language selection for translation.
2.  Implement API client in the React component to interact with the Next.js API backend (`/api/chat`, `/api/chat/selected`, `/api/auth`).
3.  Integrate the React Chatbot component into the Docusaurus site layout.
4.  Implement logic for anonymous and authenticated user sessions, leveraging Supabase Auth on the frontend.
5.  Add UI elements for personalization features (e.g., displaying user name, saving chat history).
6.  Implement language selection for Urdu translation.

**Deliverables**:
*   `docusaurus/src/components/Chatbot/`: Complete React chatbot component.
*   Docusaurus configured to include the chatbot component.
*   Working chatbot UI for both anonymous and authenticated users, with translation capabilities.

### Phase 5: Testing, CI/CD & Deployment

**Objective**: Ensure the entire system is robust, well-tested, and can be deployed reliably to Vercel and GitHub Pages.

**Tasks**:
1.  Write comprehensive unit and integration tests for the Next.js API routes (Jest/Supertest).
2.  Write tests for React components (Jest/React Testing Library).
3.  Implement end-to-end tests for the full user flow (e.g., Playwright/Cypress).
4.  Configure Vercel deployment for the `nextjs-backend/` application.
5.  Set up GitHub Actions for Docusaurus frontend deployment to GitHub Pages.

**Deliverables**:
*   Comprehensive test suites.
*   Vercel project configured for `nextjs-backend/`.
*   GitHub Actions workflows for Docusaurus deployment.

## Evaluation and Validation
After all phases, the system will be evaluated against the Success Criteria from the updated specification. This includes quantitative metrics (e.g., response times, accuracy, translation quality) and qualitative assessments (e.g., UI usability, personalization effectiveness). A final review against the Constitution will ensure all principles are upheld.
