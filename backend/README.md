---
title: Robotics Book Chatbot API
emoji: 🤖
colorFrom: blue
colorTo: purple
sdk: docker
app_port: 7860
pinned: false
license: mit
---

# Robotics Book Chatbot API

A production-ready Retrieval-Augmented Generation (RAG) chatbot API built with FastAPI, Cohere, and Qdrant.

## Features

- **Book Management**: Upload and manage robotics books/documents
- **Vector Search**: Semantic search using Cohere embeddings and Qdrant
- **Chat Sessions**: Interactive Q&A sessions about your documents
- **AI-Powered**: Uses Cohere's command-r model for intelligent responses
- **RESTful API**: Full OpenAPI/Swagger documentation

## API Endpoints

### Health Check
- `GET /` - API health check

### Books
- `POST /v1/books` - Create a new book
- `GET /v1/books` - List all books
- `GET /v1/books/{book_id}` - Get specific book
- `PUT /v1/books/{book_id}` - Update book

### Chat
- `POST /v1/chat/sessions` - Create chat session
- `POST /v1/chat/sessions/{session_token}/messages` - Send message
- `GET /v1/chat/sessions/{session_token}` - Get session info

## Quick Start

### Create a Book
```bash
curl -X POST "https://obaid987-robotics-book-chatbot-api.hf.space/v1/books" \
  -H "Content-Type: application/json" \
  -d '{
    "title": "Robotics Fundamentals",
    "author": "Tech Author",
    "content": "Robotics combines mechanical engineering, electrical engineering, and computer science...",
    "metadata": {"genre": "robotics"}
  }'
```

### Create Chat Session
```bash
curl -X POST "https://obaid987-robotics-book-chatbot-api.hf.space/v1/chat/sessions" \
  -H "Content-Type: application/json" \
  -d '{"book_id": "YOUR_BOOK_ID"}'
```

### Ask Questions
```bash
curl -X POST "https://obaid987-robotics-book-chatbot-api.hf.space/v1/chat/sessions/YOUR_SESSION_TOKEN/messages" \
  -H "Content-Type: application/json" \
  -d '{"message": "What are the key concepts in robotics?"}'
```

## API Documentation

Once deployed, visit:
- **Swagger UI**: https://obaid987-robotics-book-chatbot-api.hf.space/docs
- **ReDoc**: https://obaid987-robotics-book-chatbot-api.hf.space/redoc

## Configuration

Required environment variables (set in Space Settings → Repository secrets):

- `COHERE_API_KEY` - Your Cohere API key
- `QDRANT_API_KEY` - Your Qdrant API key
- `QDRANT_URL` - Your Qdrant cluster URL

Optional:
- `COHERE_MODEL` - Default: command-r
- `COHERE_EMBED_MODEL` - Default: embed-english-v3.0

## Technology Stack

- **FastAPI** - Modern Python web framework
- **Cohere** - LLM and embeddings
- **Qdrant** - Vector database
- **Pydantic** - Data validation
- **Uvicorn** - ASGI server
- **Docker** - Containerization

## License

MIT
