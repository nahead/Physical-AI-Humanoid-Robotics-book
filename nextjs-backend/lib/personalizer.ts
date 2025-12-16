// nextjs-backend/lib/personalizer.ts
import { genAI } from './gemini';
import { BackgroundLevel } from '../types'; // Assuming BackgroundLevel is defined in types/index.ts

const PERSONALIZATION_MODEL_NAME = "gemini-2.5-flash";

/**
 * Personalizes content using the Gemini API based on user's background.
 * @param originalContent The original markdown content of the chapter.
 * @param softwareBackground The user's software background level.
 * @param hardwareBackground The user's hardware background level.
 * @returns A promise that resolves to the personalized markdown content.
 */
export async function personalizeContent(
  originalContent: string,
  softwareBackground: BackgroundLevel,
  hardwareBackground: BackgroundLevel
): Promise<string> {
  try {
    const model = genAI.getGenerativeModel({ model: PERSONALIZATION_MODEL_NAME });

    const prompt = `You are an expert technical writer. Rewrite the following technical content for a user with a ${softwareBackground} software background and a ${hardwareBackground} hardware background. Focus on simplifying complex concepts for beginners, providing analogies, or giving deeper technical context for advanced users, as appropriate for their level.
    
    Maintain the original markdown formatting (headings, lists, code blocks, links, etc.) as much as possible. If there are code blocks, explain their purpose in simple terms for beginners, or provide deeper insights for advanced users, rather than rewriting the code itself. For diagrams or images, provide a textual description/explanation relevant to the user's level, without altering the image tags themselves.
    
    Original Content:
    ${originalContent}`;

    const result = await model.generateContent(prompt);
    const response = result.response.text();

    return response.trim();
  } catch (error) {
    console.error(`Error personalizing content for backgrounds (SW: ${softwareBackground}, HW: ${hardwareBackground}):`, error);
    throw new Error(`Failed to personalize content: ${error instanceof Error ? error.message : String(error)}`);
  }
}
