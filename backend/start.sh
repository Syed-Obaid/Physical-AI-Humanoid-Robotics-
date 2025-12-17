#!/bin/bash

# Startup script for RAG Chatbot API

echo "🚀 Starting RAG Chatbot API..."

# Check if required environment variables are set
if [ -z "$COHERE_API_KEY" ]; then
    echo "⚠️  Warning: COHERE_API_KEY is not set. LLM features will not work."
fi

if [ -z "$QDRANT_API_KEY" ]; then
    echo "⚠️  Warning: QDRANT_API_KEY is not set. Vector search will use in-memory mode."
fi

# Set default port if not specified
PORT="${PORT:-7860}"

echo "📡 Starting server on port $PORT..."

# Start the FastAPI application
exec uvicorn src.api.main:app --host 0.0.0.0 --port $PORT
