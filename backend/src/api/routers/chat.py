from fastapi import APIRouter, HTTPException, Depends, BackgroundTasks
from typing import List
import logging
import traceback
import uuid

from src.models.chat_session import ChatSessionCreate, ChatSession
from src.models.user_query import UserQueryCreate
from src.services.session_service import SessionService
from src.services.retrieval_service import RetrievalService
from src.services.llm_service import LLMService
from src.vector_db.vector_db_client import VectorDBClient
from pydantic import BaseModel

logger = logging.getLogger(__name__)


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

    logger.info(f"Processing message for session {session_id}: {chat_request.message[:50]}...")

    # 1. Validate session
    try:
        session = session_service.get_session(session_id)
    except Exception as e:
        logger.error(f"Session lookup failed: {e}\n{traceback.format_exc()}")
        raise HTTPException(status_code=500, detail="Session service unavailable")

    if not session:
        logger.warning(f"Session not found or expired: {session_id}")
        raise HTTPException(status_code=404, detail="Session not found or expired")

    book_id = session.get("book_id")
    if not book_id:
        logger.error(f"Session {session_id} has no associated book_id")
        raise HTTPException(status_code=400, detail="Session has no associated book")

    logger.info(f"Session valid. Book ID: {book_id}")

    # 2. Retrieve context from vector database
    try:
        context_chunks = retrieval_service.retrieve_relevant_context(
            query=chat_request.message,
            book_id=book_id
        )
        logger.info(f"Retrieved {len(context_chunks)} context chunks")
    except Exception as e:
        logger.error(f"Retrieval failed for book {book_id}: {e}\n{traceback.format_exc()}")
        raise HTTPException(
            status_code=503,
            detail=f"Unable to search document: {str(e)}. Please check if the document was indexed."
        )

    # 3. Handle empty results gracefully
    if not context_chunks:
        logger.warning(f"No context found for query '{chat_request.message}' in book {book_id}")
        return ChatResponse(
            id=str(uuid.uuid4()),
            response_text="I couldn't find relevant information in the document for your query. This may mean the document hasn't been indexed yet, or try rephrasing your question.",
            sources=[],
            has_citations=False
        )

    # 4. Generate LLM response
    try:
        response_text = llm_service.generate_response(
            query=chat_request.message,
            context_chunks=context_chunks
        )
        logger.info(f"LLM response generated: {len(response_text)} chars")
    except Exception as e:
        logger.error(f"LLM generation failed: {e}\n{traceback.format_exc()}")
        raise HTTPException(
            status_code=503,
            detail=f"AI service error: {str(e)}. Please try again."
        )

    # 5. Update session context (non-critical, don't fail the request)
    try:
        session_service.update_session_context(
            session_token=session_id,
            query=chat_request.message,
            response=response_text
        )
    except Exception as e:
        logger.warning(f"Failed to update session context: {e}")
        # Don't raise - this is non-critical

    # 6. Extract sources and return response
    sources = [chunk.get("vector_id", "") for chunk in context_chunks if chunk.get("vector_id")]

    return ChatResponse(
        id=str(uuid.uuid4()),
        response_text=response_text,
        sources=sources,
        has_citations=len(sources) > 0
    )


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


class HealthStatus(BaseModel):
    status: str
    llm_service: bool
    llm_error: str | None
    vector_db: bool
    vector_db_error: str | None
    embedding_service: bool
    embedding_error: str | None


@router.get("/chat/health", response_model=HealthStatus)
async def health_check():
    """Check if all backend services are available and properly configured"""

    llm_ok = llm_service.client is not None
    vector_ok = retrieval_service.vector_db.client is not None
    embed_ok = retrieval_service.embedding_service.client is not None

    status = HealthStatus(
        status="healthy" if (llm_ok and vector_ok and embed_ok) else "degraded",
        llm_service=llm_ok,
        llm_error=llm_service.connection_error if not llm_ok else None,
        vector_db=vector_ok,
        vector_db_error=retrieval_service.vector_db.connection_error if not vector_ok else None,
        embedding_service=embed_ok,
        embedding_error=retrieval_service.embedding_service.connection_error if not embed_ok else None,
    )

    if not (llm_ok and vector_ok and embed_ok):
        logger.warning(f"Health check failed: {status.model_dump()}")
        raise HTTPException(status_code=503, detail=status.model_dump())

    return status