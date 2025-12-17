#!/bin/bash

# Quick Deployment Script for Hugging Face Spaces
# This script automates the deployment process

set -e

echo "🚀 Deploying RAG Chatbot API to Hugging Face Spaces..."
echo ""

# Configuration
SPACE_URL="https://huggingface.co/spaces/SYED-OBAID/rag-chatbot-api"
SPACE_DIR="hf-space-deploy"

# Clean up previous deployment folder
if [ -d "$SPACE_DIR" ]; then
    echo "📁 Cleaning up previous deployment folder..."
    rm -rf "$SPACE_DIR"
fi

# Clone the Space
echo "📥 Cloning Hugging Face Space..."
git clone "$SPACE_URL" "$SPACE_DIR"

# Copy files
echo "📋 Copying files..."
cd "$SPACE_DIR"

# Copy essential files
cp -r ../src .
cp ../Dockerfile .
cp ../requirements.txt .

# Create README.md
cat > README.md << 'EOF'
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

# RAG Chatbot API 📚

A powerful Retrieval-Augmented Generation (RAG) chatbot API for interactive digital books. Upload books, create chat sessions, and ask questions about book content using AI.

## 🚀 Features

- **Book Management**: Upload and manage book content
- **Interactive Chat**: Create chat sessions and ask questions about books
- **Vector Search**: Efficient semantic search using Qdrant
- **AI-Powered**: Leverages Cohere for embeddings and text generation
- **FastAPI**: High-performance async API framework

## 📡 API Endpoints

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

## 🔧 Configuration

The following environment variables must be configured as Hugging Face Secrets:

- `COHERE_API_KEY` - Cohere API key for embeddings and generation
- `QDRANT_API_KEY` - Qdrant vector database API key
- `QDRANT_URL` - Qdrant cluster URL
- `COHERE_MODEL` - Generation model (default: command-r)
- `COHERE_EMBED_MODEL` - Embedding model (default: embed-english-v3.0)

## 📖 API Documentation

Once deployed, access the interactive API documentation:
- **Swagger UI**: https://syed-obaid-rag-chatbot-api.hf.space/docs
- **ReDoc**: https://syed-obaid-rag-chatbot-api.hf.space/redoc

## 💡 Usage Example

\`\`\`python
import requests

BASE_URL = "https://syed-obaid-rag-chatbot-api.hf.space"

# Create a book
book_data = {
    "title": "Introduction to AI",
    "author": "John Doe",
    "content": "Artificial Intelligence is transforming the world...",
    "metadata": {"genre": "technology", "year": 2024}
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
\`\`\`

## 📄 License

MIT License

---

Built with ❤️ using FastAPI, Cohere, and Qdrant
EOF

# Git operations
echo "📝 Committing changes..."
git config user.email "syed-obaid@users.noreply.huggingface.co"
git config user.name "SYED-OBAID"
git add .
git commit -m "Deploy RAG Chatbot API to Hugging Face Spaces

- Add FastAPI backend with book management
- Implement RAG chat sessions with Cohere
- Integrate Qdrant vector database for semantic search
- Configure Docker for HF Spaces deployment
- Add complete API documentation"

# Push to Hugging Face
echo "🚀 Pushing to Hugging Face..."
git push origin main

echo ""
echo "✅ Deployment complete!"
echo ""
echo "📍 Next Steps:"
echo "1. Go to: https://huggingface.co/spaces/SYED-OBAID/rag-chatbot-api/settings"
echo "2. Add the following secrets:"
echo "   - COHERE_API_KEY"
echo "   - QDRANT_API_KEY"
echo "   - QDRANT_URL"
echo ""
echo "3. Monitor the build: https://huggingface.co/spaces/SYED-OBAID/rag-chatbot-api"
echo "4. Your API will be live at: https://syed-obaid-rag-chatbot-api.hf.space"
echo ""
echo "🎉 Happy deploying!"
