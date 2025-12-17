# RAG Chatbot for Digital Books

This project implements a Retrieval-Augmented Generation (RAG) chatbot embedded in digital books, allowing readers to ask questions about book content and receive accurate, contextually relevant answers.

## Architecture

The system consists of:
- **Backend**: FastAPI application handling API requests, vector database operations, and LLM interactions
- **Vector Database**: Qdrant Cloud for storing and retrieving document embeddings
- **LLM Service**: Cohere API for generating answers based on retrieved context
- **Frontend**: JavaScript widget for embedding in digital books

## Technologies Used

- Python 3.11+
- FastAPI
- Cohere API
- Qdrant Cloud
- PostgreSQL (Neon Serverless)
- JavaScript for frontend widget

## Setup

1. Install Python dependencies:
   ```bash
   cd backend
   pip install -r requirements.txt
   ```

2. Set up environment variables:
   ```bash
   cp .env.example .env
   # Edit .env with your API keys and configuration
   ```

3. Start the backend server:
   ```bash
   cd backend
   uvicorn src.api.main:app --reload --port 8000
   ```

## Adding a Book

To add a book to the system, use the API endpoint:

```bash
curl -X POST "http://localhost:8000/v1/books" \
  -H "Content-Type: application/json" \
  -d '{
    "title": "Introduction to Robotics",
    "author": "Sample Author",
    "content": "Full text content of the book goes here...",
    "metadata": {
      "isbn": "1234567890",
      "published": "2023-01-01"
    }
  }'
```

## Running the Demo

1. Start the backend server as described above
2. Open `examples/sample-book/index.html` in a web browser
3. The chatbot widget will be available in the bottom-right corner
4. Ask questions about the content of the sample book

## Frontend Integration

To embed the chatbot in your own digital book:

1. Include the JavaScript widget in your HTML:
   ```html
   <script src="/path/to/ChatbotWidget.js"></script>
   <div id="chatbot-container"></div>
   ```

2. Initialize the widget:
   ```javascript
   RAGChatbot.init({
     containerId: "chatbot-container",
     bookId: "your-book-id",
     apiUrl: "http://localhost:8000/v1"
   });
   ```

## API Endpoints

- `POST /v1/books` - Add a new book
- `GET /v1/books` - List available books
- `GET /v1/books/{bookId}` - Get book details
- `POST /v1/chat/sessions` - Create a new chat session
- `POST /v1/chat/sessions/{sessionId}/messages` - Send a message in a session
- `GET /v1/chat/sessions/{sessionId}` - Get session details

## How It Works

1. Books are processed and split into chunks, then embeddings are generated using Cohere
2. Embeddings are stored in Qdrant vector database with metadata
3. When a user asks a question, it is embedded and searched against the book's chunks
4. Relevant chunks are retrieved and used as context for the LLM
5. Cohere generates a response based on the question and context
6. The response is returned to the user with source citations

## Privacy Compliance

- User queries are not stored permanently
- Sessions expire after a set time period
- No user data is retained beyond session lifetime
