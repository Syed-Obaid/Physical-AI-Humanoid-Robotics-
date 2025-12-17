from fastapi import APIRouter, HTTPException, Depends, BackgroundTasks
from typing import List
# from ..models.chat_session import ChatSessionCreate, ChatSession
# from ..models.user_query import UserQueryCreate
from src.models.chat_session import ChatSessionCreate, ChatSession
from src.models.user_query import UserQueryCreate
# from ..services.session_service import SessionService
from fastapi import APIRouter
from src.services.session_service import SessionService
from src.services.retrieval_service import RetrievalService
from src.services.llm_service import LLMService
from src.vector_db.vector_db_client import VectorDBClient
from pydantic import BaseModel
import uuid


router = APIRouter()
session_service = SessionService()
retrieval_service = RetrievalService()
llm_service = LLMService()


class ChatRequest(BaseModel):
    message: str


class ChatResponse(BaseModel):
    id: str
    response_text: str
    sources: List[str]
    has_citations: bool


@router.post("/chat/sessions", response_model=ChatSession)
async def create_chat_session(session_data: ChatSessionCreate):
    """Create a new chat session for interacting with a book"""
    try:
        session_token = session_service.create_session(session_data.book_id)
        session = session_service.get_session(session_token)
        
        # Convert to ChatSession model format
        return ChatSession(
            id=session["id"],
            session_token=session["session_token"],
            book_id=session["book_id"],
            active=session["active"],
            created_at=session["created_at"],
            last_interaction_at=session["last_interaction_at"],
            expires_at=session["expires_at"]
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error creating session: {str(e)}")


@router.post("/chat/sessions/{session_id}/messages", response_model=ChatResponse)
async def send_message(session_id: str, chat_request: ChatRequest):
    """Send a message in a chat session and receive a response"""
    try:
        # Verify the session exists
        session = session_service.get_session(session_id)
        if not session:
            raise HTTPException(status_code=404, detail="Session not found or expired")
        
        # Retrieve relevant context from the book
        context_chunks = retrieval_service.retrieve_relevant_context(
            query=chat_request.message,
            book_id=session["book_id"]
        )
        
        # Generate a response using the LLM
        response_text = llm_service.generate_response(
            query=chat_request.message,
            context_chunks=context_chunks
        )
        
        # Update the session with the new interaction
        session_service.update_session_context(
            session_token=session_id,
            query=chat_request.message,
            response=response_text
        )
        
        # Extract source information for citations
        sources = [chunk.get("vector_id", "") for chunk in context_chunks if chunk.get("vector_id")]
        
        # Create a unique ID for this response
        response_id = str(uuid.uuid4())
        
        # Return the response
        return ChatResponse(
            id=response_id,
            response_text=response_text,
            sources=sources,
            has_citations=len(sources) > 0
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing message: {str(e)}")


@router.get("/chat/sessions/{session_id}", response_model=ChatSession)
async def get_session(session_id: str):
    """Get details about a specific chat session"""
    try:
        session = session_service.get_session(session_id)
        if not session:
            raise HTTPException(status_code=404, detail="Session not found or expired")
        
        return ChatSession(
            id=session["id"],
            session_token=session["session_token"],
            book_id=session["book_id"],
            active=session["active"],
            created_at=session["created_at"],
            last_interaction_at=session["last_interaction_at"],
            expires_at=session["expires_at"]
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving session: {str(e)}")