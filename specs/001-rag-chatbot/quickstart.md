# Quickstart Guide: RAG Chatbot Environment Setup (Updated for Supabase, Qdrant, Gemini)

This guide provides step-by-step instructions for setting up the necessary free-tier environments for the RAG Chatbot using Supabase, Qdrant Cloud, and Google Gemini API.

## 1. Supabase Setup

Supabase provides a PostgreSQL database, authentication, and more.

### 1.1 Create a Supabase Project

1.  **Sign Up/Log In**: Go to [Supabase.com](https://supabase.com/) and sign up or log in to your account.
2.  **Create New Project**: Click "New project" in your dashboard.
3.  **Project Details**:
    *   **Organization**: Choose or create an organization.
    *   **Project Name**: Give your project a meaningful name (e.g., `rag-chatbot-supabase`).
    *   **Database Password**: Create a strong password and save it securely.
    *   **Region**: Select a region close to your deployment location (Vercel project).
    *   **Pricing Plan**: Select "Free".
4.  **Project Dashboard**: Once the project is created, navigate to its dashboard.

### 1.2 Get Supabase Project API Keys and URL

1.  From your Supabase project dashboard, go to "Settings" (gear icon) -> "API".
2.  **Project URL**: Copy the "Project URL" (e.g., `https://abcdefghijk.supabase.co`).
3.  **Project Anon Key**: Copy the "anon public" key (it starts with `eyJ...`).
    **IMPORTANT**: You will need these later as environment variables (`NEXT_PUBLIC_SUPABASE_URL`, `NEXT_PUBLIC_SUPABASE_ANON_KEY`).

### 1.3 Setup Database Schema

You will define your database schema (for `chat_sessions` and `messages` tables, and potentially `profiles` for custom user data) using SQL or Supabase's UI. This will be covered in Phase 1 of implementation.

## 2. Qdrant Cloud Free Tier Setup

Qdrant Cloud provides a managed vector database service.

### 2.1 Create a Qdrant Cloud Account and Cluster

1.  **Sign Up/Log In**: Go to [cloud.qdrant.io](https://cloud.qdrant.io/) and sign up or log in.
2.  **Create New Cluster**: Click on "Create New Cluster".
3.  **Cluster Details**:
    *   Choose a cluster name (e.g., `rag-chatbot-vectors`).
    *   Select the "Free" plan.
    *   Select a region.
4.  **API Key and Host**: After the cluster is provisioned, navigate to its details. You will find:
    *   **Host**: This will be a URL like `https://[your-cluster-id].qdrant.tech`.
    *   **API Key**: A long string (e.g., `******_your_api_key_******`).
    **IMPORTANT**: Copy both the Host and API Key. You will need them later as environment variables (`QDRANT_HOST`, `QDRANT_API_KEY`).

### 2.2 Create a Collection (Optional, can be done programmatically)

While you can create collections programmatically via the API, you can also do it through the UI for testing.
1.  In your cluster dashboard, go to "Collections".
2.  Click "Create Collection".
3.  Define the collection name (e.g., `book_chunks`), vector size (e.g., `768` for Gemini `embedding-001` or `1536` for `text-embedding-ada-002` if you were using OpenAI; verify Gemini embedding model output dimension), and distance metric (`Cosine`).

## 3. Google Gemini API Key

The Gemini API is used for both embeddings and chat completions, as well as translation.

### 3.1 Obtain a Gemini API Key

1.  Go to [Google AI Studio](https://aistudio.google.com/app/apikey) (or Google Cloud Console if preferred).
2.  Follow the instructions to create a new API Key.
    **IMPORTANT**: Copy this API Key. You will need it later as an environment variable (`GEMINI_API_KEY`).

## 4. Environment Variables

Create a `.env.local` file in your `nextjs-backend/` directory (for Next.js) and configure your Vercel project with the following variables:

```dotenv
# Supabase
NEXT_PUBLIC_SUPABASE_URL="https://abcdefghijk.supabase.co"
NEXT_PUBLIC_SUPABASE_ANON_KEY="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9..."
SUPABASE_SERVICE_ROLE_KEY="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9..." # Only needed for backend operations with elevated privileges

# Qdrant
QDRANT_HOST="https://[your-cluster-id].qdrant.tech"
QDRANT_API_KEY="******_your_api_key_******"

# Gemini
GEMINI_API_KEY="AIzaSyA..." # Your Google Gemini API Key
```

Remember to replace the bracketed placeholders with your actual credentials.