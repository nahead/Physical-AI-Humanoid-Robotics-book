// nextjs-backend/lib/ai-skills/gemini-rag-chat.ts
import { genAI } from '../gemini';
import { RetrievedChunk } from '../retriever'; // Assuming common types
import { Citation } from '../../types';

const CHAT_MODEL_NAME = "gemini-1.5-flash";

interface GeminiRagChatInput {
  query: string;
  contextChunks: RetrievedChunk[];
}

interface GeminiRagChatOutput {
  response: string;
  citations: Citation[];
}

/**
 * Implements the RAG chat skill using Gemini Pro.
 * @param input The input for the RAG chat skill.
 * @returns The generated chat response and extracted citations.
 */
export async function geminiRagChatSkill(input: GeminiRagChatInput): Promise<GeminiRagChatOutput> {
  const { query, contextChunks } = input;

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
      url: chunk.metadata.sourceFile ? `/docs/${chunk.metadata.sourceFile}` : undefined,
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
        maxOutputTokens: 500,
      },
    });

    const result = await chat.sendMessage(query);
    const response = result.response.text();

    return { response, citations };
  } catch (error) {
    console.error(`Error executing gemini-rag-chat skill for query: "${query.substring(0, 50)}..."`, error);
    throw new Error(`gemini-rag-chat skill failed: ${error instanceof Error ? error.message : String(error)}`);
  }
}
