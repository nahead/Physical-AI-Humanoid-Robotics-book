// lib/indexer.ts
import { QdrantClient } from "@qdrant/js-client-rest";
import { generateEmbeddingsBatch } from "./gemini_embeddings";
import { chunkText } from "./chunker";

export const COLLECTION_NAME = process.env.QDRANT_COLLECTION_NAME || "book_chunks";
export const EMBEDDING_DIMENSION = 768;

export interface DocumentMetadata {
  sourceFile: string;
  title: string;
}

// Define Point type
export interface PointStruct {
  id: string | number;
  vector: number[];
  payload?: Record<string, any>;
}

// Index a document into Qdrant
export async function indexDocument(
  idOrText: string,
  content?: string,
  metadata?: DocumentMetadata
) {
  const client = new QdrantClient({ url: process.env.QDRANT_URL!, apiKey: process.env.QDRANT_API_KEY });

  // Ensure collection exists
  await client.createCollection(COLLECTION_NAME, {
    vectors: { size: EMBEDDING_DIMENSION, distance: "Cosine" },
  });

  const textToIndex = content || idOrText;
  const chunks = chunkText(textToIndex, 500);
  const embeddings = await generateEmbeddingsBatch(chunks);

  const points: PointStruct[] = chunks.map((chunk, idx) => ({
    id: metadata?.sourceFile ? `${metadata.sourceFile}_${idx}` : idx,
    vector: embeddings[idx],
    payload: { chunkText: chunk, ...(metadata || {}) },
  }));

  await client.upsert(COLLECTION_NAME, {
    wait: true,
    points,
  });

  return { indexed: points.length };
}

// Delete a document by ID
export async function deleteDocument(id: string) {
  const client = new QdrantClient({
    url: process.env.QDRANT_URL!,
    apiKey: process.env.QDRANT_API_KEY,
  });

  try {
    await client.delete(COLLECTION_NAME, {
      points: [id],
    });

    console.log(`Deleted document: ${id}`);
    return { success: true };
  } catch (error) {
    console.error("Delete failed:", error);
    throw error;
  }
}
