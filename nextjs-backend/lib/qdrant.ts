// nextjs-backend/lib/qdrant.ts
import { QdrantClient } from '@qdrant/js-client-rest';

// Ensure that the environment variables are not undefined.
const qdrantHost = process.env.QDRANT_HOST;
const qdrantApiKey = process.env.QDRANT_API_KEY;

if (!qdrantHost) {
  throw new Error("Missing environment variable: QDRANT_HOST");
}
if (!qdrantApiKey) {
  throw new Error("Missing environment variable: QDRANT_API_KEY");
}

export const qdrantClient = new QdrantClient({
  url: qdrantHost,
  apiKey: qdrantApiKey,
});
