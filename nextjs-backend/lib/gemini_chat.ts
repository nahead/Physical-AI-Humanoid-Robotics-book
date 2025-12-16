// nextjs-backend/lib/gemini_chat.ts
import { genAI } from './gemini';
import { RetrievedChunk } from './retriever';
import { Citation } from '../types'; // Import Citation type

const CHAT_MODEL_NAME = "gemini-2.5-flash";


/**
 * Generates a chat response using Gemini Pro, incorporating retrieved context chunks.
 * Also extracts potential citations from the context.
 * @param query The user's chat query.
 * @param contextChunks An array of relevant text chunks retrieved from the vector DB.
 * @returns A promise that resolves to the generated chat response and extracted citations.
 */
export async function generateChatResponse(
  query: string,
  contextChunks: RetrievedChunk[]
): Promise<{ response: string; citations: Citation[] }> {
  try {
    const model = genAI.getGenerativeModel({ model: CHAT_MODEL_NAME });

    // Assemble context for the LLM
    const context = contextChunks.map((chunk, index) =>
      `[CONTEXT PART ${index + 1}]\n${chunk.chunkText}`
    ).join('\n\n');

    // Extract unique citations from the contextChunks to pass back
    const citations: Citation[] = contextChunks.map(chunk => ({
      source: chunk.metadata.title || chunk.metadata.sourceFile,
      context: chunk.chunkText,
      url: chunk.metadata.sourceFile ? `/docs/${chunk.metadata.sourceFile}` : undefined, // Assuming Docusaurus URL structure
    }));

    const chat = model.startChat({
      history: [
        {
          role: "user",
          parts: [{ text: `You are a helpful assistant that answers questions based on the provided book content. If the answer is not in the provided text, respond with "I cannot answer that question based on the provided information." Do not invent information.\n\n${context}` }],
        },
        {
          role: "model",
          parts: [{ text: "Okay, I understand. I will answer strictly based on the provided context." }],
        },
      ],
      generationConfig: {
        maxOutputTokens: 500, // Limit response length
      },
    });

    const result = await chat.sendMessage(query);
    const response = result.response.text();

    return { response, citations };
  } catch (error) {
    console.error(`Error generating chat response for query: "${query.substring(0, 50)}"...`, error);
    throw new Error(`Failed to generate chat response: ${error instanceof Error ? error.message : String(error)}`);
  }
}
