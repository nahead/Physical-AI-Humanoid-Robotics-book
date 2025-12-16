// nextjs-backend/src/pages/api/skills/invoke.ts
import { NextApiRequest, NextApiResponse } from 'next';
import corsMiddleware from '../../../../lib/middleware';
import { invokeSkill, SkillId } from '../../../../lib/skill-executor';
import { handleApiError } from '../../../../utils/errorHandler';
import { SkillInvocationRequest, SkillInvocationResponse } from '../../../../types';

// Define valid Skill IDs (must match SkillId type in skill-executor.ts)
const validSkillIds: SkillId[] = [
  'gemini-rag-chat',
  'gemini-translate',
  'gemini-personalize-content',
];

export default async function handler(
  req: NextApiRequest,
  res: NextApiResponse<SkillInvocationResponse | { success: boolean; error: string }>
) {
  await corsMiddleware(req, res);

  if (req.method !== 'POST') {
    res.setHeader('Allow', ['POST']);
    return res.status(405).json({ success: false, error: `Method ${req.method} Not Allowed` });
  }

  try {
    const { skillId, input } = req.body as SkillInvocationRequest;

    if (!skillId || !input) {
      return res.status(400).json({ success: false, error: 'skillId and input are required.' });
    }

    // ✅ Runtime validation for skillId
    if (!validSkillIds.includes(skillId as SkillId)) {
      return res.status(400).json({ success: false, error: `Invalid skillId '${skillId}'.` });
    }

    // ✅ Type assertion safe now
    const result = await invokeSkill(skillId as SkillId, input);
    return res.status(200).json(result);
  } catch (error) {
    handleApiError(
      res,
      error,
      `Failed to invoke skill '${(req.body as SkillInvocationRequest)?.skillId || 'unknown'}'.`
    );
  }
}
