# Data Models for Physical AI & Humanoid Robotics Book

**Date**: 2025-12-05
**Feature**: [specs/002-ai-robotics-book/spec.md](specs/002-ai-robotics-book/spec.md)

This document outlines the data models for the core educational content and the bonus interactive features.

## Core Content Data Model

This model is implicitly defined by the Docusaurus file structure.

- **Book**: The root of the Docusaurus project.
  - **Attributes**: `title`, `tagline`
  - **Contains**: `Module[]`

- **Module**: A top-level directory within the `/docs` folder, representing a major part of the book.
  - **Attributes**: `title`, `order`
  - **Contains**: `Chapter[]`
  - **Example**: `/docs/module1-ros/`

- **Chapter**: A subdirectory or file within a Module.
  - **Attributes**: `title`, `order`
  - **Contains**: `SubChapter[]` or `Content`
  - **Example**: `/docs/module1-ros/01-introduction-to-ros/`

- **SubChapter**: A markdown file (`.md`/`.mdx`) within a Chapter.
  - **Attributes**: `title`
  - **Contains**: `Content` (Markdown, React components)
  - **Example**: `/docs/module1-ros/01-introduction-to-ros/01-nodes.md`

- **Content**: The text, code examples, images, and exercises within a SubChapter file.

## Bonus Features Data Model

These models are for the database-backed features (RAG Chatbot, Auth).

### User Authentication (`Better-Auth`)

- **User**:
  - `user_id` (Primary Key, UUID)
  - `email` (String, Unique)
  - `hashed_password` (String)
  - `created_at` (Timestamp)
  - `last_login` (Timestamp)
  - `personalization_prefs` (JSONB): Stores user preferences for content personalization (e.g., `{ "audience": "beginner" }`).

### RAG Chatbot (`FastAPI`, `Neon Postgres`, `Qdrant`)

- **ChatSession**: Represents a single conversation with a user.
  - `session_id` (Primary Key, UUID)
  - `user_id` (Foreign Key -> User, nullable for anonymous users)
  - `created_at` (Timestamp)
  - `history` (JSONB): Stores the conversation history as a list of messages. `[{ "role": "user", "content": "..." }, { "role": "assistant", "content": "..." }]`

- **VectorizedContent** (Managed by Qdrant): This is not a direct database model but represents the data structure in the Qdrant vector database.
  - `vector_id` (UUID)
  - `content_chunk` (String): The actual text chunk from the book.
  - `metadata` (Object):
    - `source_chapter`: The chapter the content came from.
    - `source_module`: The module the content came from.
  - `vector` (Array of Floats): The embedding vector.
