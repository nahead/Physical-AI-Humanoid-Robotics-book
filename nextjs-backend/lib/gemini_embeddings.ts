// nextjs-backend/lib/gemini_embeddings.ts
import { genAI } from './gemini';

const EMBEDDING_MODEL_NAME = "text-embedding-004";

/**
 * Generates embeddings for a given text using the Gemini API.
 * @param text The text string to embed.
 * @returns A promise that resolves to an array of numbers representing the embedding.
 */
export async function generateEmbedding(text: string): Promise<number[]> {
  try {
    const model = genAI.getGenerativeModel({ model: EMBEDDING_MODEL_NAME });
    const result = await model.embedContent({
      content: { parts: [{ text: text }], role: "user" },
    });
    const embedding = result.embedding.values;
    return embedding;
  } catch (error) {
    console.error(`Error generating embedding for text: "${text.substring(0, 50)}..."`, error);
    throw new Error(`Failed to generate embedding: ${error instanceof Error ? error.message : String(error)}`);
  }
}

/**
 * Generates embeddings for multiple text entries in a batch.
 * @param texts An array of text strings to embed.
 * @returns A promise that resolves to an array of arrays of numbers, where each inner array is an embedding.
 */
export async function generateEmbeddingsBatch(texts: string[]): Promise<number[][]> {
  try {
    const model = genAI.getGenerativeModel({ model: EMBEDDING_MODEL_NAME });
    const { embeddings } = await model.batchEmbedContents({
      requests: texts.map(text => ({
        content: { parts: [{ text: text }], role: "user" },
      })),
    });
    return embeddings.map(e => e.values);
  } catch (error) {
    console.error(`Error generating embeddings for batch of ${texts.length} texts:`, error);
    throw new Error(`Failed to generate batch embeddings: ${error instanceof Error ? error.message : String(error)}`);
  }
}
