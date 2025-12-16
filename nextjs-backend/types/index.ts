// nextjs-backend/types/index.ts

export type Language = "en" | "ur";
export type BackgroundLevel = "beginner" | "intermediate" | "advanced";

// --- Schemas from OpenAPI ---

export interface Citation {
  source: string;
  context: string;
  url?: string;
}

export interface UserAuth {
  email: string;
  password: string;
}

export interface UserResponse {
  user_id: string; // uuid
  email: string; // email format
  message: string;
}

// --- Request Body Types ---

export interface IndexRequest {
  force_reindex?: boolean;
}

export interface ChatRequest {
  session_id?: string; // uuid
  query: string;
  target_language?: Language;
}

export interface SelectedTextChatRequest {
  session_id?: string; // uuid
  query: string;
  selected_text: string;
  target_language?: Language;
}

export interface PersonalizationRequest {
  originalContent: string;
  softwareBackground: BackgroundLevel;
  hardwareBackground: BackgroundLevel;
}

export interface TranslateRequest {
  text: string;
  target_language: Language;
}

// --- Response Payload Types ---

export interface IndexResponse {
  message: string;
}

export interface ChatResponse {
  response_en: string;
  response_translated?: string;
  citations?: Citation[];
}

export interface SelectedTextChatResponse {
  response_en: string;
  response_translated?: string;
}

export interface PersonalizationResponse {
  personalizedContent: string;
}

export interface TranslateResponse {
  translated_text: string;
}

export interface AISkill {
  id: string;
  description: string;
  inputSchema: object; // JSON Schema
  outputSchema: object; // JSON Schema
  promptTemplate?: string;
  modelName: string;
  version: string;
  type: string;
}

export interface SkillInvocationRequest {
  skillId: string;
  input: object;
}

export interface SkillInvocationResponse {
  output: object;
}

