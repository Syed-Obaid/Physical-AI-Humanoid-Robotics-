# Data Model: Integrated RAG Chatbot for Book

## Overview
This document defines the data models for the RAG chatbot system, including entities, their attributes, relationships, and validation rules based on the feature requirements.

## Entity: Book Content

### Description
The text and associated metadata of the published digital book that serves as the knowledge base for the chatbot.

### Attributes
- `id` (string, required): Unique identifier for the book
- `title` (string, required): Title of the book
- `author` (string, required): Author of the book
- `content` (string, required): The full text content of the book
- `metadata` (object, optional): Additional book metadata (publishing date, ISBN, etc.)
- `chunks` (array of Chunk objects): Broken-down content chunks for embedding
- `created_at` (datetime, required): Timestamp when the book record was created
- `updated_at` (datetime, required): Timestamp when the book record was last updated

### Validation Rules
- `id` must be a UUID or other unique identifier
- `title` must be between 1 and 500 characters
- `content` must not be empty
- `chunks` must have at least one entry if content is processed

## Entity: Chunk

### Description
A segment of book content that has been processed for vector embedding and retrieval.

### Attributes
- `id` (string, required): Unique identifier for the chunk
- `book_id` (string, required): Reference to the parent Book Content
- `text` (string, required): The actual text content of the chunk
- `position` (integer, required): Position of this chunk within the book
- `embedding` (array of float, required): Vector embedding of the text content
- `vector_id` (string, required): ID in the vector database (Qdrant)
- `created_at` (datetime, required): Timestamp when the chunk record was created

### Validation Rules
- `text` must be between 50 and 2000 characters
- `position` must be a non-negative integer
- `embedding` must have the expected dimension for the embedding model
- `book_id` must reference an existing Book Content

### Relationships
- One Book Content contains many Chunks (1 to many)

## Entity: User Query

### Description
The text input from the reader asking a question about the book content.

### Attributes
- `id` (string, required): Unique identifier for the query
- `session_id` (string, required): Reference to the Chat Session
- `query_text` (string, required): The text of the user's question
- `processed` (boolean, required): Whether the query has been processed
- `created_at` (datetime, required): Timestamp when the query was received
- `processed_at` (datetime, optional): Timestamp when the query was processed

### Validation Rules
- `query_text` must be between 5 and 1000 characters
- `session_id` must reference an existing Chat Session
- `processed` defaults to false

### Relationships
- One Chat Session contains many User Queries (1 to many)

## Entity: Retrieved Context

### Description
Relevant passages from the book content that are used to generate the chatbot's response.

### Attributes
- `id` (string, required): Unique identifier for the retrieved context
- `query_id` (string, required): Reference to the User Query
- `chunk_id` (string, required): Reference to the Chunk that provided the context
- `similarity_score` (float, required): Similarity score from vector search (0.0-1.0)
- `context_text` (string, required): The actual text content retrieved
- `source_position` (integer, required): Position in the original book
- `retrieved_at` (datetime, required): Timestamp when context was retrieved

### Validation Rules
- `similarity_score` must be between 0.0 and 1.0
- `chunk_id` must reference an existing Chunk
- `query_id` must reference an existing User Query

### Relationships
- One User Query can have many Retrieved Context entries (1 to many)
- One Chunk can be referenced by many Retrieved Context entries (1 to many)

## Entity: Chat Session

### Description
The conversational context that maintains references between related user queries and system responses.

### Attributes
- `id` (string, required): Unique identifier for the session
- `session_token` (string, required): Anonymous token for session identification
- `book_id` (string, required): Reference to the Book Content being used
- `active` (boolean, required): Whether the session is currently active
- `created_at` (datetime, required): Timestamp when the session was created
- `last_interaction_at` (datetime, required): Timestamp of the last interaction
- `expires_at` (datetime, required): Timestamp when the session expires

### Validation Rules
- `session_token` must be a securely generated random string
- `book_id` must reference an existing Book Content
- `active` defaults to true
- `expires_at` must be in the future

## Entity: Response

### Description
The chatbot's answer to a user's query, generated based on retrieved context.

### Attributes
- `id` (string, required): Unique identifier for the response
- `query_id` (string, required): Reference to the User Query
- `response_text` (string, required): The text of the chatbot's response
- `sources` (array of strings, required): IDs of Retrieved Context used to generate the response
- `generated_at` (datetime, required): Timestamp when the response was generated
- `has_citations` (boolean, required): Whether the response includes citations to book content

### Validation Rules
- `response_text` must be between 10 and 5000 characters
- `query_id` must reference an existing User Query
- `sources` must contain valid Retrieved Context IDs or be empty
- Each source in `sources` must reference an existing Retrieved Context

### Relationships
- One User Query maps to one Response (1 to 1)

## State Transitions

### Chat Session
- Created → Active (when user starts a conversation)
- Active → Inactive (when user leaves the book or session times out)
- Inactive → Expired (after a set time period)
- Expired → Deleted (automatic cleanup)

### User Query
- Received → Processing (when query is being processed)
- Processing → Processed (when response is ready)
- Processed → Archived (if retention policy allows)

## Indexing Strategy

### Book Content
- Primary index on `id`

### Chunk
- Primary index on `id`
- Foreign key index on `book_id`
- Index on `vector_id` for fast vector database lookups

### Chat Session
- Primary index on `id`
- Index on `session_token` for quick lookups
- Index on `expires_at` for efficient cleanup

### User Query
- Primary index on `id`
- Foreign key index on `session_id`
- Index on `created_at` for chronological access
