---
title: RAG Chatbot API
emoji: 📚
colorFrom: blue
colorTo: purple
sdk: docker
pinned: false
license: mit
app_port: 7860
---

# RAG Chatbot API

A Retrieval-Augmented Generation (RAG) chatbot API for interactive digital books. This API allows you to upload books, create chat sessions, and ask questions about the book content using AI.

## Features

- 📚 **Book Management**: Upload and manage book content
- 💬 **Interactive Chat**: Create chat sessions and ask questions about books
- 🔍 **Vector Search**: Uses Qdrant for efficient semantic search
- 🤖 **AI-Powered**: Leverages Cohere for embeddings and text generation
- 🚀 **FastAPI**: High-performance async API framework

## API Endpoints

### Root
- `GET /` - API information

### Books
- `GET /v1/books` - List all books
- `POST /v1/books` - Create a new book
- `GET /v1/books/{book_id}` - Get specific book
- `PUT /v1/books/{book_id}` - Update a book

### Chat
- `POST /v1/chat/sessions` - Create a chat session
- `POST /v1/chat/sessions/{session_id}/messages` - Send a message
- `GET /v1/chat/sessions/{session_id}` - Get session details

## Environment Variables

You need to configure the following environment variables in your Hugging Face Space settings:

```bash
# Cohere API (Required)
COHERE_API_KEY=your-cohere-api-key

# Qdrant Vector Database (Required)
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_URL=https://your-cluster-url.gcp.cloud.qdrant.io

# Optional Settings
COHERE_MODEL=command-r
COHERE_EMBED_MODEL=embed-english-v3.0
SESSION_EXPIRY_MINUTES=60
```

## Setup Instructions

1. **Get API Keys**:
   - Cohere API: [https://cohere.com/](https://cohere.com/)
   - Qdrant Cloud: [https://cloud.qdrant.io/](https://cloud.qdrant.io/)

2. **Configure Secrets**:
   - Go to your Hugging Face Space settings
   - Add the required environment variables as secrets

3. **Deploy**:
   - Push your code to the Hugging Face Space repository
   - The Space will automatically build and deploy

## Usage Example

```python
import requests

BASE_URL = "https://your-space-name.hf.space"

# Create a book
book_data = {
    "title": "My Book",
    "author": "Author Name",
    "content": "Long book content here...",
    "metadata": {}
}
response = requests.post(f"{BASE_URL}/v1/books", json=book_data)
book_id = response.json()["id"]

# Create a chat session
session_data = {"book_id": book_id}
response = requests.post(f"{BASE_URL}/v1/chat/sessions", json=session_data)
session_id = response.json()["session_token"]

# Ask a question
message_data = {"message": "What is this book about?"}
response = requests.post(
    f"{BASE_URL}/v1/chat/sessions/{session_id}/messages",
    json=message_data
)
print(response.json()["response_text"])
```

## API Documentation

Once deployed, you can access the interactive API documentation at:
- Swagger UI: `https://your-space-name.hf.space/docs`
- ReDoc: `https://your-space-name.hf.space/redoc`

## License

MIT License - See LICENSE file for details

## Support

For issues and questions, please open an issue in the repository.
