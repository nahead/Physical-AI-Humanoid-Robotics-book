// docusaurus/src/components/Chatbot/api.ts

export type Language = "en" | "ur";
export type BackgroundLevel = "beginner" | "intermediate" | "advanced"; // Added for personalization

export interface Citation {
  source: string;
  context: string;
  url?: string;
}

export interface ChatRequest {
  session_id?: string;
  query: string;
  target_language?: Language;
}

export interface ChatResponse {
  response_en: string;
  response_translated?: string;
  citations?: Citation[];
}

export const fetchChatResponse = async (request: ChatRequest, apiBaseUrl: string): Promise<ChatResponse> => {
  const finalApiBaseUrl = apiBaseUrl || 'http://localhost:3001';

  try {
    const response = await fetch(`${finalApiBaseUrl}/api/chat`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(request),
    });

    if (!response.ok) {
      const errorData = await response.json();
      if (response.status === 500 && errorData.details?.includes("Quota exceeded")) {
        throw new Error("API quota has been exceeded. Please try again later.");
      }
      throw new Error(errorData.error || 'Failed to fetch chat response');
    }

    return await response.json();
  } catch (error) {
    console.error('API Error:', error);
    let errorMessage = 'An unexpected error occurred. Please try again.';
    if (error instanceof TypeError && error.message.includes('Failed to fetch')) {
      errorMessage = 'Could not connect to the backend. Please ensure it is running.';
    } else if (error instanceof Error) {
      errorMessage = error.message;
    }
    return {
      response_en: errorMessage,
      citations: [],
    };
  }
};

export interface TranslateRequest {
  text: string;
  target_language: Language;
}

export interface TranslateResponse {
  translated_text: string;
}

export const fetchTranslation = async (request: TranslateRequest, apiBaseUrl: string): Promise<TranslateResponse> => {
  const finalApiBaseUrl = apiBaseUrl || 'http://localhost:3001';

  try {
    const response = await fetch(`${finalApiBaseUrl}/api/translate`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(request),
    });

    if (!response.ok) {
      const errorData = await response.json();
      throw new Error(errorData.error || 'Failed to fetch translation');
    }

    return await response.json();
  } catch (error) {
    console.error('API Error:', error);
    return { translated_text: 'Error: Could not translate content.' };
  }
};

// New types and function for personalization
export interface PersonalizationRequest {
  originalContent: string;
  softwareBackground: BackgroundLevel;
  hardwareBackground: BackgroundLevel;
}

export interface PersonalizationResponse {
  personalizedContent: string;
}

export const fetchPersonalizedContent = async (request: PersonalizationRequest, apiBaseUrl: string): Promise<PersonalizationResponse> => {
  const finalApiBaseUrl = apiBaseUrl || 'http://localhost:3001';

  try {
    const response = await fetch(`${finalApiBaseUrl}/api/personalize`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(request),
    });

    if (!response.ok) {
      const errorData = await response.json();
      if (response.status === 500 && errorData.error?.includes("Quota exceeded")) {
        throw new Error("Personalization API quota has been exceeded. Please try again later.");
      }
      throw new Error(errorData.error || 'Failed to fetch personalized content');
    }

    return await response.json();
  } catch (error) {
    console.error('Personalization API Error:', error);
    let errorMessage = 'An unexpected error occurred during personalization. Please try again.';
    if (error instanceof TypeError && error.message.includes('Failed to fetch')) {
      errorMessage = 'Could not connect to the backend for personalization. Please ensure it is running.';
    } else if (error instanceof Error) {
      errorMessage = error.message;
    }
    return {
      personalizedContent: `Error: ${errorMessage}`,
    };
  }
};