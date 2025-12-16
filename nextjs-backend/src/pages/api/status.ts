// nextjs-backend/pages/api/index.ts
import { NextApiRequest, NextApiResponse } from 'next';
import { indexDocument, DocumentMetadata, deleteDocument } from '../../../lib/indexer';

import { loadMarkdownDocument } from '../../../lib/document_loader';
import { handleApiError } from '../../../utils/errorHandler';
import fs from 'fs';
import path from 'path';

// This path should ideally be configurable or discovered dynamically
// For now, hardcode to a typical Docusaurus docs path relative to the Next.js backend's CWD
const DOCUSAURUS_DOCS_PATH = path.join(process.cwd(), '../../docusaurus/docs');

export default async function handler(req: NextApiRequest, res: NextApiResponse) {
  if (req.method !== 'POST') {
    res.setHeader('Allow', ['POST']);
    return res.status(405).json({ success: false, error: `Method ${req.method} Not Allowed` });
  }

  try {
    const { force_reindex } = req.body;

    // A simple way to find markdown files in the Docusaurus docs directory
    const getMarkdownFiles = async (dir: string, fileList: string[] = []): Promise<string[]> => {
      const files = await fs.promises.readdir(dir);
      for (const file of files) {
        const filePath = path.join(dir, file);
        const stat = await fs.promises.stat(filePath);
        if (stat.isDirectory()) {
          await getMarkdownFiles(filePath, fileList);
        } else if (file.endsWith('.md') || file.endsWith('.mdx')) {
          fileList.push(filePath);
        }
      }
      return fileList;
    };

    const markdownFiles = await getMarkdownFiles(DOCUSAURUS_DOCS_PATH);
    const results: Array<{ file: string; status: string; error?: string }> = [];

    for (const filePath of markdownFiles) {
      const documentId = path.relative(DOCUSAURUS_DOCS_PATH, filePath).replace(/\\/g, '/'); // Use relative path as ID
      
      // Basic metadata extraction (can be enhanced with frontmatter parsing)
      const metadata: DocumentMetadata = {
        sourceFile: documentId,
        title: path.basename(filePath, path.extname(filePath)),
      };

      try {
        if (force_reindex) {
          await deleteDocument(documentId);
          console.log(`Deleted existing index for ${documentId}`);
        }
        const { content } = await loadMarkdownDocument(filePath);
        await indexDocument(documentId, content, metadata);
        results.push({ file: documentId, status: 'indexed' });
      } catch (error: any) {
        console.error(`Failed to index ${documentId}:`, error);
        results.push({ file: documentId, status: 'failed', error: error.message });
      }
    }

    const failedCount = results.filter(r => r.status === 'failed').length;
    return res.status(200).json({
      success: true,
      message: `Indexing process completed. Total files: ${markdownFiles.length}, Successfully indexed: ${markdownFiles.length - failedCount}, Failed: ${failedCount}.`,
      results,
    });
  } catch (error) {
    handleApiError(res, error, 'Failed to trigger indexing process.');
  }
}
