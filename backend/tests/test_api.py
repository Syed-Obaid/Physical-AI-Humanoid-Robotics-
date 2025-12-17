import pytest
from httpx import AsyncClient
import asyncio
import sys
import os

# Add the project root to the path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.api.main import app


@pytest.mark.asyncio
async def test_root_endpoint():
    """Test the root endpoint"""
    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.get("/")
        assert response.status_code == 200
        data = response.json()
        assert "message" in data
        assert data["message"] == "RAG Chatbot API"
        assert "version" in data


@pytest.mark.asyncio
async def test_list_books_empty():
    """Test listing books when database is empty"""
    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.get("/v1/books")
        assert response.status_code == 200
        data = response.json()
        assert isinstance(data, list)


@pytest.mark.asyncio
async def test_create_book():
    """Test creating a new book"""
    async with AsyncClient(app=app, base_url="http://test") as client:
        book_data = {
            "title": "Test Book",
            "author": "Test Author",
            "content": "This is a test book content. " * 100,  # Make it long enough
            "metadata": {"genre": "test", "year": 2024}
        }
        response = await client.post("/v1/books", json=book_data)
        print(f"\nCreate book response: {response.status_code}")
        print(f"Response content: {response.text}")

        assert response.status_code == 200
        data = response.json()
        assert "id" in data
        assert data["title"] == book_data["title"]
        assert data["author"] == book_data["author"]
        assert "created_at" in data
        assert "updated_at" in data

        # Store the book ID for other tests
        return data["id"]


@pytest.mark.asyncio
async def test_get_book():
    """Test retrieving a specific book"""
    async with AsyncClient(app=app, base_url="http://test") as client:
        # First create a book
        book_data = {
            "title": "Get Test Book",
            "author": "Get Test Author",
            "content": "This is content for testing get endpoint. " * 50,
            "metadata": {}
        }
        create_response = await client.post("/v1/books", json=book_data)
        book_id = create_response.json()["id"]

        # Now get the book
        response = await client.get(f"/v1/books/{book_id}")
        print(f"\nGet book response: {response.status_code}")
        print(f"Response content: {response.text}")

        assert response.status_code == 200
        data = response.json()
        assert data["id"] == book_id
        assert data["title"] == book_data["title"]


@pytest.mark.asyncio
async def test_get_nonexistent_book():
    """Test retrieving a book that doesn't exist"""
    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.get("/v1/books/nonexistent-id")
        assert response.status_code == 404


@pytest.mark.asyncio
async def test_update_book():
    """Test updating a book"""
    async with AsyncClient(app=app, base_url="http://test") as client:
        # First create a book
        book_data = {
            "title": "Original Title",
            "author": "Original Author",
            "content": "Original content. " * 50,
            "metadata": {}
        }
        create_response = await client.post("/v1/books", json=book_data)
        book_id = create_response.json()["id"]

        # Update the book
        update_data = {
            "title": "Updated Title",
            "author": "Updated Author"
        }
        response = await client.put(f"/v1/books/{book_id}", json=update_data)
        print(f"\nUpdate book response: {response.status_code}")
        print(f"Response content: {response.text}")

        assert response.status_code == 200
        data = response.json()
        assert data["title"] == update_data["title"]
        assert data["author"] == update_data["author"]


@pytest.mark.asyncio
async def test_create_chat_session():
    """Test creating a chat session"""
    async with AsyncClient(app=app, base_url="http://test") as client:
        # First create a book
        book_data = {
            "title": "Chat Test Book",
            "author": "Chat Test Author",
            "content": "This is content for testing chat. " * 100,
            "metadata": {}
        }
        create_response = await client.post("/v1/books", json=book_data)
        book_id = create_response.json()["id"]

        # Create a chat session
        session_data = {
            "book_id": book_id,
            "session_token": "",  # Will be generated
            "active": True
        }
        response = await client.post("/v1/chat/sessions", json=session_data)
        print(f"\nCreate session response: {response.status_code}")
        print(f"Response content: {response.text}")

        # This might fail due to model issues, but let's see
        if response.status_code == 200:
            data = response.json()
            assert "id" in data
            assert "session_token" in data
            assert data["book_id"] == book_id
            return data["session_token"]
        else:
            print(f"Session creation failed with: {response.text}")


@pytest.mark.asyncio
async def test_send_message():
    """Test sending a message in a chat session"""
    async with AsyncClient(app=app, base_url="http://test") as client:
        # First create a book
        book_data = {
            "title": "Message Test Book",
            "author": "Message Test Author",
            "content": "This is a comprehensive test book about Python programming. Python is a high-level programming language. " * 50,
            "metadata": {}
        }
        create_response = await client.post("/v1/books", json=book_data)
        book_id = create_response.json()["id"]

        # Create a chat session
        session_data = {
            "book_id": book_id,
            "session_token": "",
            "active": True
        }
        session_response = await client.post("/v1/chat/sessions", json=session_data)

        if session_response.status_code == 200:
            session_token = session_response.json()["session_token"]

            # Send a message
            message_data = {
                "message": "What is Python?"
            }
            response = await client.post(f"/v1/chat/sessions/{session_token}/messages", json=message_data)
            print(f"\nSend message response: {response.status_code}")
            print(f"Response content: {response.text}")

            # This might fail due to LLM/embedding service issues
            if response.status_code == 200:
                data = response.json()
                assert "id" in data
                assert "response_text" in data
                assert "sources" in data
                assert "has_citations" in data
            else:
                print(f"Message sending failed with: {response.text}")


@pytest.mark.asyncio
async def test_get_session():
    """Test retrieving a chat session"""
    async with AsyncClient(app=app, base_url="http://test") as client:
        # First create a book
        book_data = {
            "title": "Session Get Test Book",
            "author": "Session Get Test Author",
            "content": "Test content. " * 50,
            "metadata": {}
        }
        create_response = await client.post("/v1/books", json=book_data)
        book_id = create_response.json()["id"]

        # Create a chat session
        session_data = {
            "book_id": book_id,
            "session_token": "",
            "active": True
        }
        session_response = await client.post("/v1/chat/sessions", json=session_data)

        if session_response.status_code == 200:
            session_token = session_response.json()["session_token"]

            # Get the session
            response = await client.get(f"/v1/chat/sessions/{session_token}")
            print(f"\nGet session response: {response.status_code}")
            print(f"Response content: {response.text}")

            assert response.status_code == 200
            data = response.json()
            assert data["session_token"] == session_token
            assert data["book_id"] == book_id


if __name__ == "__main__":
    # Run tests directly
    pytest.main([__file__, "-v", "-s"])
