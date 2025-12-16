// lib/retriever.ts
import { qdrantClient } from './qdrant';
import { generateEmbedding } from './gemini_embeddings';
import { DocumentMetadata } from './indexer';

const COLLECTION_NAME = process.env.QDRANT_COLLECTION_NAME || 'book_chunks';
const TOP_K = 5;

export interface RetrievedChunk {
  chunkText: string;
  metadata: DocumentMetadata;
  score: number;
}

export async function retrieveChunks(query: string, k: number = TOP_K): Promise<RetrievedChunk[]> {
  try {
    const queryEmbedding = await generateEmbedding(query);

    // Qdrant search returns array directly
    const points = await qdrantClient.search(COLLECTION_NAME, {
      vector: queryEmbedding,
      limit: k,
      with_payload: true,
    });

    const retrieved: RetrievedChunk[] = points.map((point: any) => {
      const payload = point.payload as {
        chunkText: string;
        sourceFile: string;
        title: string;
      };

      return {
        chunkText: payload.chunkText,
        metadata: {
          sourceFile: payload.sourceFile,
          title: payload.title,
        },
        score: point.score,
      };
    });

    console.log(`Retrieved ${retrieved.length} chunks for query: "${query.substring(0, 50)}..."`);
    return retrieved;
  } catch (error) {
    console.error(`Error retrieving chunks for query: "${query.substring(0, 50)}..."`, error);
    throw new Error(`Failed to retrieve chunks: ${error instanceof Error ? error.message : String(error)}`);
  }
}
