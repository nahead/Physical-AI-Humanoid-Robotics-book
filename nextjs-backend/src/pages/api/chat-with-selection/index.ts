// nextjs-backend/pages/api/chat/selected.ts
import { NextApiRequest, NextApiResponse } from 'next';
import { SelectedTextChatRequest, SelectedTextChatResponse } from '../../../../types';
import { RetrievedChunk } from '../../../../lib/retriever';
import { generateChatResponse } from '../../../../lib/gemini_chat';
import { translateText } from '../../../../lib/gemini_translate';
import { handleApiError } from '../../../../utils/errorHandler';

export default async function handler(req: NextApiRequest, res: NextApiResponse<SelectedTextChatResponse | { success: boolean; error: string }>) {
  if (req.method !== 'POST') {
    res.setHeader('Allow', ['POST']);
    return res.status(405).json({ success: false, error: `Method ${req.method} Not Allowed` });
  }

  try {
    const { query, selected_text, session_id, target_language } = req.body as SelectedTextChatRequest;

    if (!query || !selected_text) {
      return res.status(400).json({ success: false, error: 'Query and selected_text are required.' });
    }

    // For selected text chat, the context is strictly the provided selected_text
    // We create a mock RetrievedChunk to fit the generateChatResponse signature
    const contextChunk: RetrievedChunk[] = [{
      chunkText: selected_text,
      metadata: {
        sourceFile: 'selected-text', // Indicate the source is user-selected text
        title: 'Selected Text',
      },
      score: 1.0 // Perfect score as it's the direct context
    }];

    // Generate response using Gemini with ONLY the selected text as context
    // We explicitly set citations to empty as the source is just the selected text, not a document chunk
    const { response: response_en } = await generateChatResponse(query, contextChunk); // Citations not directly applicable here from chunks

    let response_translated: string | undefined;
    if (target_language && target_language !== 'en') {
      response_translated = await translateText(response_en, target_language);
    }
    
    // Note: Citations are not returned for selected text chat as the source is just the selected_text itself.
    // If we wanted to "cite" the selected text, the frontend would know what was selected.
    return res.status(200).json({
      response_en,
      response_translated,
    });

  } catch (error) {
    handleApiError(res, error, 'Failed to generate selected text chat response.');
  }
}
