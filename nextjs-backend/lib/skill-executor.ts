// nextjs-backend/lib/skill-executor.ts
import { getSkillById } from './skill-registry';
import { AISkill, SkillInvocationResponse } from '../types';
import { supabase } from './supabase';
import { generateChatResponse } from './gemini_chat';
import { translateText } from './gemini_translate';
import { personalizeContent } from './personalizer';

// 1️⃣ Define all skill IDs explicitly
export type SkillId = 
  | 'gemini-rag-chat'
  | 'gemini-translate'
  | 'gemini-personalize-content';

// 2️⃣ Type for skill input/output mapping
type SkillImplementer = {
  [K in SkillId]: (input: any) => Promise<any>;
};

// 3️⃣ Register skill implementations
const skillImplementations: SkillImplementer = {
  'gemini-rag-chat': async (input) => {
    const { query, contextChunks } = input;
    if (!query || !contextChunks) throw new Error('Invalid input for gemini-rag-chat skill.');
    return generateChatResponse(query, contextChunks);
  },
  'gemini-translate': async (input) => {
    const { text, targetLanguage } = input;
    if (!text || !targetLanguage) throw new Error('Invalid input for gemini-translate skill.');
    return translateText(text, targetLanguage);
  },
  'gemini-personalize-content': async (input) => {
    const { originalContent, softwareBackground, hardwareBackground } = input;
    if (!originalContent || !softwareBackground || !hardwareBackground)
      throw new Error('Invalid input for gemini-personalize-content skill.');
    return personalizeContent(originalContent, softwareBackground, hardwareBackground);
  },
};

// 4️⃣ Skill executor function
export async function invokeSkill(skillId: SkillId, input: any): Promise<SkillInvocationResponse> {
  // Validate skill exists in registry
  const skillMetadata = await getSkillById(skillId);
  if (!skillMetadata) {
    throw new Error(`Skill '${skillId}' not found in registry.`);
  }

  // Execute the skill
  const implementer = skillImplementations[skillId];
  if (!implementer) {
    throw new Error(`No implementation found for skill '${skillId}'.`);
  }

  try {
    const output = await implementer(input);
    return { output };
  } catch (error) {
    console.error(`Error executing skill '${skillId}':`, error);
    throw new Error(`Skill execution failed: ${error instanceof Error ? error.message : String(error)}`);
  }
}
