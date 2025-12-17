# Quickstart Guide: Integrated RAG Chatbot for Book

## Overview

This guide provides step-by-step instructions to set up, configure, and run the RAG chatbot for digital books. The system uses Cohere API for language processing, Qdrant Cloud for vector storage, and FastAPI for the backend API.

## Prerequisites

- Python 3.11 or higher
- Node.js 16 or higher (for frontend components)
- Access to Cohere API (API key)
- Qdrant Cloud account and cluster credentials
- Access to Neon Serverless Postgres
- Docker (optional, for containerized deployment)

## Environment Setup

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd <repository-directory>
   ```

2. Create a virtual environment and install Python dependencies:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   pip install -r backend/requirements.txt
   ```

3. Install frontend dependencies:
   ```bash
   cd frontend
   npm install
   ```

4. Create environment variables file `.env` in the backend directory:
   ```env
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_URL=https://your-cluster-url.gcp.cloud.qdrant.io
   QDRANT_PORT=6666
   DATABASE_URL=your_neon_postgres_connection_string
   SECRET_KEY=your_secret_key_for_session_tokens
   ```

## Backend Setup

1. Navigate to the backend directory:
   ```bash
   cd backend
   ```

2. Set up the database:
   ```bash
   # Run database migrations
   python -m alembic upgrade head
   ```

3. Initialize the vector database (Qdrant):
   ```bash
   # Run the script to create Qdrant collections
   python scripts/init_qdrant.py
   ```

4. Start the FastAPI backend server:
   ```bash
   uvicorn src.api.main:app --reload --port 8000
   ```

## Frontend Integration

1. Navigate to the frontend directory:
   ```bash
   cd frontend
   ```

2. Build the chatbot widget:
   ```bash
   npm run build
   ```

3. To run the development server:
   ```bash
   npm run dev
   ```

## Using the Chatbot

API endpoints for interacting with the chatbot:
- POST /chat/sessions - Create a new chat session
- POST /chat/sessions/{sessionId}/messages - Send a message in a chat session
- GET /chat/sessions/{sessionId} - Get chat session details

The response will contain:
- The chatbot's answer based on the book content
- Citations to specific parts of the book used to generate the response
- Confidence score for the response

## Embedding in Digital Books

To embed the chatbot in a digital book:

1. Include the JavaScript widget in your HTML:
   ```html
   <script src="/path/to/chatbot-widget.js"></script>
   <div id="chatbot-container"></div>
   <script>
     RAGChatbot.init({
       containerId: "chatbot-container",
       bookId: "your-book-id",
       apiUrl: "http://localhost:8000/v1"
     });
   </script>
   ```

2. The widget will automatically:
- Create a chat session when loaded
- Handle user input and display responses
- Manage session tokens and context

## Running Tests

1. Backend tests:
```bash
cd backend
pytest tests/
```

2. Frontend tests:
```bash
cd frontend
npm run test
```

## Configuration Options

- Adjust response timeout by modifying the `RESPONSE_TIMEOUT` environment variable
- Configure rate limiting by setting `RATE_LIMIT_REQUESTS` and `RATE_LIMIT_WINDOW`
- Set session expiry time with `SESSION_EXPIRY_MINUTES`

## Troubleshooting

- If Cohere API calls fail, verify your API key and check rate limits
- If vector search is slow, ensure your Qdrant cluster has adequate resources
- For session management issues, check that Redis is properly configured and running
