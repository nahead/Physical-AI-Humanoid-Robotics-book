# Research for Content Personalization in Book Chapters

## 1. Gemini's Capabilities for Structured Content Processing (Option D)

**Objective**: To understand how Gemini can be leveraged to implement "Option D: Provide simpler alternatives (e.g., simple code example for beginners)" for non-textual elements.

**Findings**:
*   **Rephrasing Markdown Text**: Gemini is highly capable of rephrasing and augmenting textual content based on specific instructions (e.g., "explain this concept to a beginner in software development"). It can understand context, tone, and target audience. Prompt engineering will be crucial here to maintain markdown formatting (headings, lists, links). Markdown output can be requested directly.
*   **Simplifying/Explaining Code Blocks**:
    *   **Challenge**: Directly "simplifying" or "rewriting" code *within* a code block using an LLM is risky. LLMs can introduce bugs or syntactic errors when altering code.
    *   **Approach (Textual Explanation)**: A safer approach is to treat code blocks as special textual elements. For beginners, Gemini could generate a *textual explanation* of the code, highlighting key lines, explaining the purpose, and providing simplified analogies. For advanced users, it could provide optimization tips or alternative patterns.
    *   **Alternative (Structured Output)**: For simpler code snippets, Gemini *might* be able to generate alternative code, but this requires robust validation and is prone to errors. This would be a high-risk, high-complexity path for an MVP.
*   **Modifying Diagrams/Images**:
    *   **Challenge**: LLMs are text-based. They cannot directly "modify" an image or diagram.
    *   **Approach (Textual Explanation)**: Gemini can provide textual explanations of diagrams, simplifying complex concepts for beginners or adding detail for advanced users. This text can be displayed alongside the original diagram.
    *   **Alternative (Diagram Generation)**: Extremely complex, requires image generation capabilities and integration, which is beyond the scope of a free-tier project and the current feature.
*   **Identifying External Links**:
    *   **Approach**: Gemini can analyze the text around links. For beginners, it could add context to *why* a link is important. For advanced users, it could suggest alternative resources. The links themselves should be preserved.
*   **Handling Structured Markdown**: Prompting Gemini to maintain specific markdown formatting (e.g., `## Heading`, `- List item`, `` `code` ``) is achievable with careful prompt engineering.

**Recommended Prompt Engineering Strategy (for Text & Explanations - MVP for Option D):**
The prompt to Gemini will include:
1.  The original content (chunked markdown text).
2.  The user's `software_background` and `hardware_background` levels.
3.  Clear instructions: "Explain this content to a [software_background] and [hardware_background] user. Simplify complex terms, provide analogies. For code blocks, explain their purpose in simple terms for beginners, or provide deeper insights for advanced users, rather than rewriting the code itself. Maintain original markdown formatting (headings, lists, code blocks, links) for structure, only rephrasing the narrative text. For diagrams/images, provide a textual description/explanation relevant to the user's level."

## 2. Content Extraction Strategies

**Objective**: To robustly extract content from Docusaurus pages for sending to the backend.

**Findings**:
*   **Frontend Extraction (Recommended for MVP)**: The `DocItem/Layout` component receives `props.children` which is the rendered React content of the Markdown/MDX page. To send this to the backend for personalization, we need to convert it back to a clean text representation. Using a helper function (like `extractTextFromChildren` already implemented) to get the *visible text* is a start. However, to preserve markdown structure for Gemini, we'd ideally get the *original markdown source*. Docusaurus usually renders markdown, so getting the exact markdown from the *rendered* children can be tricky.
    *   **Refinement**: Instead of extracting from `props.children` on the frontend, which is already React elements, it might be more robust to:
        1.  Make the `personalize` API endpoint accept a chapter ID/path.
        2.  Have the backend (Next.js) read the original Markdown file directly from the Docusaurus project (requires backend access to Docusaurus files, or the Markdown content to be sent from the frontend).
        3.  The frontend will need to send the current *original* content of the chapter for personalization.
*   **Sending Original Markdown (Proposed)**: The `DocItem/Layout` should have access to the original markdown of the document being displayed. We will assume for the MVP that `props.children` can be converted into a sufficiently structured text for Gemini, or that we can get the original Markdown content via a different Docusaurus API or by directly sending the raw markdown from the frontend after reading it. For now, `extractTextFromChildren` is a placeholder for obtaining "original text".

## 3. API Contract Definition for `/api/personalize`

**Endpoint**: `/api/personalize`
**Method**: `POST`

**Request Body (JSON)**:
```json
{
  "originalContent": "string",       // Original markdown content of the chapter
  "softwareBackground": "string",    // e.g., "beginner", "intermediate", "advanced"
  "hardwareBackground": "string"     // e.g., "beginner", "intermediate", "advanced"
}
```

**Response Body (JSON)**:
```json
{
  "personalizedContent": "string"    // Markdown content, rephrased by Gemini
}
```

**Error Responses**:
*   `400 Bad Request`: Missing required fields in request.
*   `500 Internal Server Error`: Backend processing error, Gemini API error (e.g., quota exceeded).

## Conclusion (for Option D - non-textual elements)

Implementing "Option D" (providing simpler alternatives for non-textual elements) is extremely complex. For an MVP, the `personalizer.ts` service will focus on:
*   **Text only**: Rephrasing the general markdown text content.
*   **Code Blocks**: Generating *textual explanations* of code blocks suitable for the user's level, rather than modifying the code itself.
*   **Diagrams/Images**: Generating textual explanations for diagrams/images.
The frontend will then need to intelligently replace/augment the original content with these explanations. Direct modification/generation of non-textual assets is deferred due to complexity.
