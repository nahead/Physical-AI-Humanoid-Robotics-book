// nextjs-backend/lib/chunker.ts

/**
 * Splits text into chunks suitable for embeddings, attempting to preserve sentence boundaries.
 * @param text The input text to chunk.
 * @param chunkSize The approximate target size of each chunk (in characters).
 * @param chunkOverlap The number of characters to overlap between consecutive chunks.
 * @returns An array of text chunks.
 */
export function chunkText(text: string, chunkSize: number = 1000, chunkOverlap: number = 200): string[] {
  if (chunkSize <= chunkOverlap) {
    throw new Error("Chunk size must be greater than chunk overlap.");
  }
  
  const chunks: string[] = [];
  let currentPosition = 0;

  while (currentPosition < text.length) {
    let endPosition = Math.min(currentPosition + chunkSize, text.length);

    // Try to find a natural break (sentence or paragraph end) within the chunk
    // Search backwards from endPosition to currentPosition + overlap to find a period, newline, etc.
    let naturalBreak = -1;
    for (let i = endPosition - 1; i >= currentPosition + chunkOverlap; i--) {
      if (text[i] === '.' || text[i] === '\n' || text[i] === '!' || text[i] === '?') {
        naturalBreak = i + 1; // Include the punctuation in the chunk
        break;
      }
    }

    if (naturalBreak !== -1 && naturalBreak > currentPosition + chunkOverlap) {
      endPosition = naturalBreak;
    } else if (endPosition === text.length && currentPosition === 0) {
      // If it's the very first chunk and the whole text fits, or no natural break found, take the whole thing
      // This covers cases where text is smaller than chunk size
    } else if (endPosition < text.length) {
      // If we are cutting mid-sentence, try to cut at the next space to avoid broken words
      const nextSpace = text.indexOf(' ', endPosition);
      if (nextSpace !== -1 && nextSpace < currentPosition + chunkSize + 50) { // Limit how far we stretch
        endPosition = nextSpace;
      }
    }

    let chunk = text.substring(currentPosition, endPosition).trim();
    if (chunk) {
      chunks.push(chunk);
    }

    // Move currentPosition back by overlap for the next chunk
    currentPosition = endPosition - chunkOverlap;
    if (currentPosition < 0) currentPosition = 0; // Ensure it doesn't go below 0
    if (endPosition >= text.length) break; // If we've reached the end, stop
  }

  return chunks;
}
