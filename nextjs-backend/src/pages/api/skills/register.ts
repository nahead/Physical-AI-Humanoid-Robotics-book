// nextjs-backend/src/pages/api/skills/register.ts
import { NextApiRequest, NextApiResponse } from 'next';
import corsMiddleware from '../../../../lib/middleware'; // Adjust path as needed
import { registerSkill } from '../../../../lib/skill-registry';
import { handleApiError } from '../../../../utils/errorHandler';
import { AISkill } from '../../../../types'; // Adjust path as needed

export default async function handler(req: NextApiRequest, res: NextApiResponse) {
  await corsMiddleware(req, res);

  if (req.method !== 'POST') {
    res.setHeader('Allow', ['POST']);
    return res.status(405).json({ success: false, error: `Method ${req.method} Not Allowed` });
  }

  try {
    const newSkill = req.body as AISkill;

    // Basic validation (more robust validation should use inputSchema from the skill)
    if (!newSkill.id || !newSkill.description || !newSkill.modelName || !newSkill.version || !newSkill.type) {
      return res.status(400).json({ success: false, error: 'Missing required skill fields.' });
    }
    // TODO: Add JSON schema validation for newSkill.inputSchema and newSkill.outputSchema

    const registeredSkill = await registerSkill(newSkill);
    return res.status(200).json({ success: true, data: registeredSkill });
  } catch (error: any) {
    if (error.message.includes('duplicate key')) { // Supabase/Postgres specific error for duplicate ID
      return res.status(409).json({ success: false, error: `Skill with ID '${req.body.id}' already exists.` });
    }
    handleApiError(res, error, 'Failed to register skill.');
  }
}
