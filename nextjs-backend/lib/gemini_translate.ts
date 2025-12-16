// nextjs-backend/lib/gemini_translate.ts
import { genAI } from './gemini';
import { Language } from '../types';

const TRANSLATION_MODEL_NAME = "gemini-2.5-flash";

/**
 * Translates text to a target language using the Gemini API.
 * @param text The text to translate.
 * @param targetLanguage The language code to translate to (e.g., "ur" for Urdu).
 * @returns A promise that resolves to the translated text.
 */
export async function translateText(text: string, targetLanguage: Language): Promise<string> {
  if (targetLanguage === 'en') {
    return text; // No translation needed if target is English
  }
  
  try {
    const model = genAI.getGenerativeModel({ model: TRANSLATION_MODEL_NAME });
    
    const prompt = `Translate the following English text to ${targetLanguage === 'ur' ? 'Urdu' : targetLanguage}:

"${text}"`;

    const result = await model.generateContent(prompt);
    const response = result.response.text();

    return response.trim();
  } catch (error) {
    console.error(`Error translating text to ${targetLanguage}: "${text.substring(0, 50)}"...`, error);
    throw new Error(`Failed to translate text: ${error instanceof Error ? error.message : String(error)}`);
  }
}
