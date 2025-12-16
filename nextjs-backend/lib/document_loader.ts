// lib/document_loader.ts
import fs from 'fs/promises';
import path from 'path';

export async function loadMarkdownDocument(filePath: string) {
  const content = await fs.readFile(filePath, 'utf-8');
  const fileName = path.basename(filePath);
  return { content, fileName };
}
