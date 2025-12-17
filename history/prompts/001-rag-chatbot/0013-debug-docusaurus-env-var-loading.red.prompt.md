---
id: 0013
title: Debug Docusaurus Env Var Loading
stage: red
date: 2025-12-06
surface: agent
model: gemini-1.5-pro
feature: 001-rag-chatbot
branch: 001-rag-chatbot
user: user
command: ui.js:1 Uncaught TypeError: Cannot read properties of null (reading 'contentWindow') at setReferences (ui.js:1:378) at start (ui.js:1:126) at window.onload (ui.js:1:3398)Understand this error installHook.js:1 Supabase URL or Anon Key is not available from Docusaurus customFields. iska ye matlab to nhi api valid nhi ha
labels: ["bug-fix", "frontend", "docusaurus", "environment-variables"]
links:
  spec: specs/001-rag-chatbot/spec.md
  plan: specs/001-rag-chatbot/plan.md
  tasks: specs/001-rag-chatbot/tasks.md
  ticket: null
  adr: null
  pr: null
files:
  - docusaurus/docusaurus.config.js
  - docusaurus/.env
  - docusaurus/src/lib/supabase.ts
  - docusaurus/src/contexts/AuthContext.tsx
  - docusaurus/src/components/Chatbot/index.tsx
  - docusaurus/src/components/AuthModal/index.tsx
tests: []
---

## Prompt

ui.js:1 Uncaught TypeError: Cannot read properties of null (reading 'contentWindow')
    at setReferences (ui.js:1:378)
    at start (ui.js:1:126)
    at window.onload (ui.js:1:3398)Understand this error
installHook.js:1 Supabase URL or Anon Key is not available from Docusaurus customFields.  iska ye matlab to nhi api valid nhi ha

## Response snapshot

The `Supabase URL or Anon Key is not available from Docusaurus customFields` error is a configuration loading issue, not an invalid API issue. It means the Docusaurus frontend isn't correctly receiving the Supabase URL and key from its configuration. This is likely due to the build cache not being cleared after updating the `.env` file to use `NEXT_PUBLIC_` prefixes.

The user has been instructed to:
1.  Go to the `docusaurus` directory.
2.  Run `npm run clear`.
3.  Run `npm run build`.
4.  Run `npm run start`.
5.  Confirm the exact content of `docusaurus.config.js` and if the `dotenv` require is at the top.

The `TypeError: Cannot read properties of null (reading 'contentWindow')` will be addressed once the primary configuration loading issue is resolved, as it might be a symptom of improper Docusaurus initialization.

## Outcome

- ✅ Impact: Instructions provided to resolve persistent environment variable loading issue in Docusaurus frontend.
- 🧪 Tests: Manual verification by user required after performing clean build and restart.
- 📁 Files: N/A (instructions to user).
- 🔁 Next prompts: User to confirm resolution of environment variable error after clean build.
- 🧠 Reflection: Emphasized the importance of clean build steps in frontend frameworks when dealing with environment variable changes.

## Evaluation notes (flywheel)

- Failure modes observed: Persistent environment variable loading error in Docusaurus despite code changes, likely due to caching.
- Graders run and results (PASS/FAIL): Not applicable.
- Prompt variant (if applicable): Debugging/troubleshooting.
- Next experiment (smallest change to try): User performing a clean build and restart.
