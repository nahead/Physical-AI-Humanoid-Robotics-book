// nextjs-backend/lib/gemini.ts
import { GoogleGenerativeAI } from '@google/generative-ai';

const geminiApiKey = process.env.GEMINI_API_KEY;

if (!geminiApiKey) {
  throw new Error("Missing environment variable: GEMINI_API_KEY");
}

export const genAI = new GoogleGenerativeAI(geminiApiKey);

// You can expose different models if needed
// export const textModel = genAI.getGenerativeModel({ model: "gemini-pro" });
// export const embeddingModel = genAI.getGenerativeModel({ model: "embedding-001" });
