// nextjs-backend/src/pages/api/translate.ts
import { NextApiRequest, NextApiResponse } from 'next';
import corsMiddleware from '../../../lib/middleware';
import { translateText } from '../../../lib/gemini_translate';
import { handleApiError } from '../../../utils/errorHandler';
import { TranslateRequest, TranslateResponse } from '../../../types';

export default async function handler(req: NextApiRequest, res: NextApiResponse<TranslateResponse | { success: boolean; error: string }>) {
  await corsMiddleware(req, res);

  if (req.method !== 'POST') {
    res.setHeader('Allow', ['POST']);
    return res.status(405).json({ success: false, error: `Method ${req.method} Not Allowed` });
  }

  try {
    const { text, target_language } = req.body as TranslateRequest;

    if (!text || !target_language) {
      return res.status(400).json({ success: false, error: 'text and target_language are required.' });
    }

    const translated_text = await translateText(text, target_language);

    return res.status(200).json({ translated_text });
  } catch (error) {
    handleApiError(res, error, 'Failed to translate text.');
  }
}
