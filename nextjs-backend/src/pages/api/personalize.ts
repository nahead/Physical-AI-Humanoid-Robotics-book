// nextjs-backend/src/pages/api/personalize.ts
import { NextApiRequest, NextApiResponse } from 'next';
import corsMiddleware from '../../../lib/middleware';
import { personalizeContent } from '../../../lib/personalizer';
import { handleApiError } from '../../../utils/errorHandler';
import { PersonalizationRequest, PersonalizationResponse } from '../../../types'; // Assuming types are defined here
import { supabase } from '../../../lib/supabase'; // To verify user session

export default async function handler(req: NextApiRequest, res: NextApiResponse<PersonalizationResponse | { success: boolean; error: string }>) {
  await corsMiddleware(req, res);

  if (req.method !== 'POST') {
    res.setHeader('Allow', ['POST']);
    return res.status(405).json({ success: false, error: `Method ${req.method} Not Allowed` });
  }

  try {
    // Authenticate user to ensure only logged-in users can personalize
    const { data: { session }, error: authError } = await supabase.auth.getSession();
    if (authError || !session) {
      return res.status(401).json({ success: false, error: 'Unauthorized: You must be logged in to personalize content.' });
    }

    const { originalContent, softwareBackground, hardwareBackground } = req.body as PersonalizationRequest;

    if (!originalContent || !softwareBackground || !hardwareBackground) {
      return res.status(400).json({ success: false, error: 'Missing required personalization parameters.' });
    }

    // Call the personalization service
    const personalizedContent = await personalizeContent(originalContent, softwareBackground, hardwareBackground);

    return res.status(200).json({ personalizedContent });
  } catch (error) {
    handleApiError(res, error, 'Failed to personalize content.');
  }
}
