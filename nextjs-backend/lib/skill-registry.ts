// nextjs-backend/lib/skill-registry.ts
import { supabase } from './supabase';
import { AISkill } from '../types'; // Assuming AISkill type is defined in types/index.ts

const SKILLS_TABLE = 'ai_skills';

/**
 * Registers a new AI skill in the Supabase database.
 * @param skill The AISkill object to register.
 * @returns The registered AISkill or null if an error occurred.
 */
export async function registerSkill(skill: AISkill): Promise<AISkill | null> {
  const { data, error } = await supabase
    .from(SKILLS_TABLE)
    .insert(skill)
    .select()
    .single();

  if (error) {
    console.error('Error registering skill:', error);
    throw new Error(`Failed to register skill: ${error.message}`);
  }
  return data;
}

/**
 * Retrieves an AI skill by its ID from the Supabase database.
 * @param skillId The ID of the skill to retrieve.
 * @returns The AISkill object or null if not found.
 */
export async function getSkillById(skillId: string): Promise<AISkill | null> {
  const { data, error } = await supabase
    .from(SKILLS_TABLE)
    .select('*')
    .eq('id', skillId)
    .single();

  if (error && error.code !== 'PGRST116') { // PGRST116 means no rows found (which is not an error for getById)
    console.error('Error getting skill by ID:', error);
    throw new Error(`Failed to retrieve skill: ${error.message}`);
  }
  return data;
}

/**
 * Updates an existing AI skill in the Supabase database.
 * @param skillId The ID of the skill to update.
 * @param updates The partial AISkill object containing updates.
 * @returns The updated AISkill or null if not found/error.
 */
export async function updateSkill(skillId: string, updates: Partial<AISkill>): Promise<AISkill | null> {
  const { data, error } = await supabase
    .from(SKILLS_TABLE)
    .update(updates)
    .eq('id', skillId)
    .select()
    .single();

  if (error) {
    console.error('Error updating skill:', error);
    throw new Error(`Failed to update skill: ${error.message}`);
  }
  return data;
}

/**
 * Deletes an AI skill by its ID from the Supabase database.
 * @param skillId The ID of the skill to delete.
 * @returns True if deletion was successful, false otherwise.
 */
export async function deleteSkill(skillId: string): Promise<boolean> {
  const { error } = await supabase
    .from(SKILLS_TABLE)
    .delete()
    .eq('id', skillId);

  if (error) {
    console.error('Error deleting skill:', error);
    throw new Error(`Failed to delete skill: ${error.message}`);
  }
  return true;
}

/**
 * Lists all registered AI skills.
 * @returns An array of AISkill objects.
 */
export async function listSkills(): Promise<AISkill[]> {
  const { data, error } = await supabase
    .from(SKILLS_TABLE)
    .select('*');

  if (error) {
    console.error('Error listing skills:', error);
    throw new Error(`Failed to list skills: ${error.message}`);
  }
  return data || [];
}
