// scripts/index-docs.ts
// Run with: npm run index-docs

// Load env vars BEFORE any other imports
import 'dotenv/config';

import fs from 'fs';
import path from 'path';
import { QdrantClient } from '@qdrant/js-client-rest';
import { generateEmbeddingsBatch } from '../lib/gemini_embeddings';
import { chunkText } from '../lib/chunker';

const COLLECTION_NAME = process.env.QDRANT_COLLECTION_NAME || 'book_chunks';
const EMBEDDING_DIMENSION = 768;
const DOCS_PATH = path.join(__dirname, '../../docusaurus/docs');
 

async function getAllMarkdownFiles(dir: string): Promise<string[]> {
  const files: string[] = [];
  const entries = fs.readdirSync(dir, { withFileTypes: true });

  for (const entry of entries) {
    const fullPath = path.join(dir, entry.name);
    if (entry.isDirectory()) {
      files.push(...await getAllMarkdownFiles(fullPath));
    } else if (entry.name.endsWith('.md') || entry.name.endsWith('.mdx')) {
      files.push(fullPath);
    }
  }
  return files;
}

async function indexDocs() {
  const client = new QdrantClient({
    url: process.env.QDRANT_HOST!,
    apiKey: process.env.QDRANT_API_KEY!,
  });

  // Recreate collection
  try {
    await client.deleteCollection(COLLECTION_NAME);
    console.log(`Deleted existing collection: ${COLLECTION_NAME}`);
  } catch (e) {
    console.log('No existing collection to delete');
  }

  await client.createCollection(COLLECTION_NAME, {
    vectors: { size: EMBEDDING_DIMENSION, distance: 'Cosine' },
  });
  console.log(`Created collection: ${COLLECTION_NAME}`);

  const files = await getAllMarkdownFiles(DOCS_PATH);
  console.log(`Found ${files.length} markdown files`);

  let totalChunks = 0;

  for (const filePath of files) {
    const content = fs.readFileSync(filePath, 'utf-8');
    const relativePath = path.relative(DOCS_PATH, filePath);
    const title = relativePath.replace(/\.(md|mdx)$/, '').replace(/\//g, ' > ');

    const chunks = chunkText(content, 500);
    if (chunks.length === 0) continue;

    console.log(`Indexing ${relativePath}: ${chunks.length} chunks`);

    const embeddings = await generateEmbeddingsBatch(chunks);

    const points = chunks.map((chunk, idx) => ({
      id: crypto.randomUUID(),
      vector: embeddings[idx],
      payload: {
        chunkText: chunk,
        sourceFile: relativePath,
        title: title,
      },
    }));

    await client.upsert(COLLECTION_NAME, { wait: true, points });
    totalChunks += points.length;
  }

  console.log(`\nDone! Indexed ${totalChunks} chunks from ${files.length} files.`);
}

indexDocs().catch(console.error);
