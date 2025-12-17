// nextjs-backend/pages/api/chat.ts
import { NextApiRequest, NextApiResponse } from 'next';
import corsMiddleware from '../../../lib/middleware';
import { retrieveChunks } from '../../../lib/retriever';
import { generateChatResponse } from '../../../lib/gemini_chat';
import { translateText } from '../../../lib/gemini_translate';
import { handleApiError } from '../../../utils/errorHandler';
import { ChatRequest, ChatResponse, Language } from '../../../types';
import { supabase } from '../../../lib/supabase';

export default async function handler(req: NextApiRequest, res: NextApiResponse<ChatResponse | { success: boolean; error: string }>) {
  // Run the CORS middleware
  await corsMiddleware(req, res);

  if (req.method !== 'POST') {
    res.setHeader('Allow', ['POST']);
    return res.status(405).json({ success: false, error: `Method ${req.method} Not Allowed` });
  }

  try {
    const { query, session_id, target_language } = req.body as ChatRequest;

    if (!query) {
      return res.status(400).json({ success: false, error: 'Query is required.' });
    }

    // Authenticate user if session_id is provided and valid
    // For now, this endpoint is public. User authentication for personalization
    // and session saving will be handled in subsequent tasks or directly on the client if anonymous.
    let userId: string | null = null;
    if (session_id) {
        // Here you would typically validate the session_id against Supabase
        // and fetch user_id if associated with an authenticated session.
        // For current scope, we treat session_id as an identifier but don't strictly link to auth.users here.
        // If authentication is required, middleware or specific session checks would be integrated.
    }

    // 1. Retrieve relevant chunks from Qdrant
    const retrievedChunks = await retrieveChunks(query);
    const context = retrievedChunks.map(chunk => chunk.chunkText).join('\n\n');

    // If no context, respond accordingly
    if (retrievedChunks.length === 0) {
      const noInfoResponse = 'I cannot answer that question as no relevant information was found in the book.';
      const translatedNoInfoResponse = target_language === 'ur' ? await translateText(noInfoResponse, 'ur') : undefined;
      return res.status(200).json({
        response_en: noInfoResponse,
        response_translated: translatedNoInfoResponse,
      });
    }

    // 2. Generate initial English response using Gemini with RAG context
    const { response: response_en, citations } = await generateChatResponse(query, retrievedChunks);

    let response_translated: string | undefined;
    if (target_language && target_language !== 'en') {
      response_translated = await translateText(response_en, target_language);
    }

    // 3. Save chat session and message (Optional for current task, but good practice for personalization)
    // This would typically involve using the Supabase client to insert into 'chat_sessions' and 'messages' tables.
    // For T023, we focus on the core RAG and translation, not persistence yet.

    return res.status(200).json({
      response_en,
      response_translated,
      citations,
    });

  } catch (error) {
    console.error('--- DETAILED ERROR ---', error);
    handleApiError(res, error, 'Failed to generate chat response.');
  }
}
