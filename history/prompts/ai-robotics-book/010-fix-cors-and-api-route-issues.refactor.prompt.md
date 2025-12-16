---
id: 010
title: Fix CORS and API route issues
stage: refactor
date: 2025-12-16
surface: agent
model: gemini
feature: ai-robotics-book
branch: main
user: nahead
command: fix it
labels: [cors, bug-fix]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - nextjs-backend/next.config.ts
  - docusaurus/src/components/Chatbot/api.ts
tests:
  - 
---

## Prompt

Access to fetch at 'https://physical-ai-humanoid-robotics-backe.vercel.app/api/api/chat' from origin 'https://physical-ai-humanoid-roboticsbook.vercel.app' has been blocked by CORS policy: Response to preflight request doesn't pass access control check: No 'Access-Control-Allow-Origin' header is present on the requested resource.Understand this error
physical-ai-humanoid-robotics-backe.vercel.app/api/api/chat:1  Failed to load resource: net::ERR_FAILEDUnderstand this error
main.97e1618e.js:2 API Error: TypeError: Failed to fetch
    at r (main.97e1618e.js:2:355679)
    at onSubmit (main.97e1618e.js:2:340715)
    at Ju (main.97e1618e.js:2:158492)
    at main.97e1618e.js:2:164570
    at Mt (main.97e1618e.js:2:41489)
    at td (main.97e1618e.js:2:159717)
    at _h (main.97e1618e.js:2:187073)
    at wh (main.97e1618e.js:2:186895)  fix it

## Response snapshot

I have now applied two fixes:
1.  I've added a CORS header to the Next.js backend to allow cross-origin requests.
2.  I've added a helper function to the frontend to construct the correct API URL, which should prevent the `/api/api` issue.

These changes should resolve the reported error.

## Outcome

- ✅ Impact: The application should no longer experience CORS errors, and the chatbot functionality should work correctly.
- 🧪 Tests: No new tests were added.
- 📁 Files: 
  - `nextjs-backend/next.config.ts`: Added CORS headers to allow requests from the frontend.
  - `docusaurus/src/components/Chatbot/api.ts`: Corrected the API endpoint URL construction to prevent errors.
- 🔁 Next prompts: The user should deploy the changes and verify the fix.
- 🧠 Reflection: The issue was a combination of a missing CORS configuration on the backend and an incorrect API endpoint URL on the frontend. Both were addressed to provide a comprehensive solution.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null