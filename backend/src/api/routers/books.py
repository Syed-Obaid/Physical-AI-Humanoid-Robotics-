from fastapi import APIRouter, HTTPException, BackgroundTasks
from typing import List
# from ..models.book import BookCreate, Book, BookUpdate
from src.models.book import BookCreate, Book, BookUpdate
from src.services.retrieval_service import RetrievalService
from uuid import uuid4
from datetime import datetime, UTC


router = APIRouter()
retrieval_service = RetrievalService()


# In-memory storage for books (in production, use a database)
books_db = {}


@router.get("/books", response_model=List[Book])
async def list_books():
    """List all available books"""
    try:
        return [Book(
            id=book_id,
            title=book_data["title"],
            author=book_data["author"],
            content=book_data["content"][:100] + "..." if len(book_data["content"]) > 100 else book_data["content"],  # Truncate content
            metadata=book_data.get("metadata", {}),
            created_at=book_data["created_at"],
            updated_at=book_data["updated_at"]
        ) for book_id, book_data in books_db.items()]
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error listing books: {str(e)}")


@router.post("/books", response_model=Book)
async def create_book(book_data: BookCreate, background_tasks: BackgroundTasks):
    """Add a new book to the system"""
    try:
        book_id = str(uuid4())
        now = datetime.now(UTC)

        # Store the book in memory
        book_record = {
            "id": book_id,
            "title": book_data.title,
            "author": book_data.author,
            "content": book_data.content,
            "metadata": book_data.metadata,
            "created_at": now,
            "updated_at": now
        }
        
        books_db[book_id] = book_record
        
        # Process the book content in the background
        # This involves chunking the text and creating embeddings
        background_tasks.add_task(retrieval_service.add_book_content, book_id, book_data.content)
        
        # Return the created book
        return Book(
            id=book_id,
            title=book_data.title,
            author=book_data.author,
            content=book_data.content[:100] + "..." if len(book_data.content) > 100 else book_data.content,  # Truncate content
            metadata=book_data.metadata,
            created_at=book_record["created_at"],
            updated_at=book_record["updated_at"]
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error creating book: {str(e)}")


@router.get("/books/{book_id}", response_model=Book)
async def get_book(book_id: str):
    """Get details about a specific book"""
    try:
        book = books_db.get(book_id)
        if not book:
            raise HTTPException(status_code=404, detail="Book not found")
        
        return Book(
            id=book["id"],
            title=book["title"],
            author=book["author"],
            content=book["content"][:200] + "..." if len(book["content"]) > 200 else book["content"],  # Truncate content
            metadata=book.get("metadata", {}),
            created_at=book["created_at"],
            updated_at=book["updated_at"]
        )
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving book: {str(e)}")


@router.put("/books/{book_id}", response_model=Book)
async def update_book(book_id: str, book_update: BookUpdate):
    """Update a book's information"""
    try:
        book = books_db.get(book_id)
        if not book:
            raise HTTPException(status_code=404, detail="Book not found")
        
        # Update the book with provided fields
        if book_update.title is not None:
            book["title"] = book_update.title
        if book_update.author is not None:
            book["author"] = book_update.author
        if book_update.content is not None:
            book["content"] = book_update.content
        if book_update.metadata is not None:
            book["metadata"] = book_update.metadata

        book["updated_at"] = datetime.now(UTC)
        
        return Book(
            id=book["id"],
            title=book["title"],
            author=book["author"],
            content=book["content"][:200] + "..." if len(book["content"]) > 200 else book["content"],  # Truncate content
            metadata=book["metadata"],
            created_at=book["created_at"],
            updated_at=book["updated_at"]
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating book: {str(e)}")