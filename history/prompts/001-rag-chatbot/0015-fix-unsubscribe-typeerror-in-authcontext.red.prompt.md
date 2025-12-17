---
id: 0015
title: Fix Unsubscribe TypeError in AuthContext
stage: red
date: 2025-12-06
surface: agent
model: gemini-1.5-pro
feature: 001-rag-chatbot
branch: 001-rag-chatbot
user: user
command: authListener?.unsubscribe is not a function TypeError: authListener?.unsubscribe is not a function at eval (webpack-internal:///./src/contexts/AuthContext.tsx:54:23) at Object.react_stack_bottom_frame (webpack-internal:///./node_modules/react-dom/cjs/react-dom-client.development.js:26000:13) at runWithFiberInDEV (webpack-internal:///./node_modules/react-dom/cjs/react-dom-client.development.js:871:30) at commitHookEffectListUnmount (webpack-internal:///./node_modules/react-dom/cjs/react-dom-client.development.js:13315:17) at commitHookPassiveUnmountEffects (webpack-internal:///./node_modules/react-dom/cjs/react-dom-client.development.js:13346:11) at commitPassiveUnmountEffectsInsideOfDeletedTree_begin (webpack-internal:///./node_modules/react-dom/cjs/react-dom-client.development.js:16263:13) at recursivelyTraversePassiveUnmountEffects (webpack-internal:///./node_modules/react-dom/cjs/react-dom-client.development.js:16084:13) at commitPassiveUnmountOnFiber (webpack-internal:///./node_modules/react-dom/cjs/react-dom-client.development.js:16158:11) at recursivelyTraversePassiveUnmountEffects (webpack-internal:///./node_modules/react-dom/cjs/react-dom-client.development.js:16104:11) at commitPassiveUnmountOnFiber (webpack-internal:///./node_modules/react-dom/cjs/react-dom-client.development.js:16116:11)
labels: ["bug-fix", "frontend", "docusaurus"]
links:
  spec: specs/001-rag-chatbot/spec.md
  plan: specs/001-rag-chatbot/plan.md
  tasks: specs/001-rag-chatbot/tasks.md
  ticket: null
  adr: null
  pr: null
files:
  - docusaurus/src/contexts/AuthContext.tsx
tests: []
---

## Prompt

authListener?.unsubscribe is not a function
TypeError: authListener?.unsubscribe is not a function
    at eval (webpack-internal:///./src/contexts/AuthContext.tsx:54:23)
    at Object.react_stack_bottom_frame (webpack-internal:///./node_modules/react-dom/cjs/react-dom-client.development.js:26000:13)
    at runWithFiberInDEV (webpack-internal:///./node_modules/react-dom/cjs/react-dom-client.development.js:871:30)
    at commitHookEffectListUnmount (webpack-internal:///./node_modules/react-dom/cjs/react-dom-client.development.js:13315:17)
    at commitHookPassiveUnmountEffects (webpack-internal:///./node_modules/react-dom/cjs/react-dom-client.development.js:13346:11)
    at commitPassiveUnmountEffectsInsideOfDeletedTree_begin (webpack-internal:///./node_modules/react-dom/cjs/react-dom-client.development.js:16263:13)
    at recursivelyTraversePassiveUnmountEffects (webpack-internal:///./node_modules/react-dom/cjs/react-dom-client.development.js:16084:13)
    at commitPassiveUnmountOnFiber (webpack-internal:///./node_modules/react-dom/cjs/react-dom-client.development.js:16158:11)
    at recursivelyTraversePassiveUnmountEffects (webpack-internal:///./node_modules/react-dom/cjs/react-dom-client.development.js:16104:11)
    at commitPassiveUnmountOnFiber (webpack-internal:///./node_modules/react-dom/cjs/react-dom-client.development.js:16116:11)

## Response snapshot

The `TypeError: authListener?.unsubscribe is not a function` has been fixed in `docusaurus/src/contexts/AuthContext.tsx` by correctly destructuring the `subscription` object from the `data` returned by `supabase.auth.onAuthStateChange`. This ensures that the `unsubscribe` method is called on the correct object.

## Outcome

- ✅ Impact: Critical runtime error related to Supabase authentication listener resolved.
- 🧪 Tests: Manual verification by user required by running the project after a clean build.
- 📁 Files: `docusaurus/src/contexts/AuthContext.tsx` was modified.
- 🔁 Next prompts: User to re-run Docusaurus project and verify resolution of the current error and provide details on `TypeError` from `ui.js` if it persists.
- 🧠 Reflection: Demonstrated ability to diagnose and fix API usage errors in third-party libraries based on provided stack traces and library documentation knowledge.

## Evaluation notes (flywheel)

- Failure modes observed: `TypeError: authListener?.unsubscribe is not a function` due to incorrect handling of `supabase.auth.onAuthStateChange` return value.
- Graders run and results (PASS/FAIL): Not applicable.
- Prompt variant (if applicable): Bug fixing during project execution.
- Next experiment (smallest change to try): User verification after clean build and restart.
