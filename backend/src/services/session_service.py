from datetime import datetime, timedelta, UTC
from typing import Optional, Dict, Any
import secrets
from src.config import settings



class SessionService:
    def __init__(self):
        # In a real implementation, this would connect to Redis or a database
        # For this demo, we'll use an in-memory store (not suitable for production)
        self.sessions = {}
    
    def create_session(self, book_id: str) -> str:
        """
        Create a new chat session
        
        Args:
            book_id: ID of the book to associate with the session
        
        Returns:
            Session token
        """
        # Generate a secure random token
        session_token = secrets.token_urlsafe(32)

        # Create session data
        now = datetime.now(UTC)
        session_data = {
            "id": session_token,
            "session_token": session_token,
            "book_id": book_id,
            "active": True,
            "created_at": now,
            "last_interaction_at": now,
            "expires_at": now + timedelta(minutes=settings.SESSION_EXPIRY_MINUTES),
            "context": []  # Store conversation history for context
        }
        
        # Store the session
        self.sessions[session_token] = session_data
        
        return session_token
    
    def get_session(self, session_token: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve a session by its token
        
        Args:
            session_token: Session token to look up
        
        Returns:
            Session data if found and valid, None otherwise
        """
        session = self.sessions.get(session_token)
        
        if session is None:
            return None

        # Check if session is expired
        if datetime.now(UTC) > session["expires_at"]:
            self.delete_session(session_token)
            return None
        
        return session
    
    def update_session_context(self, session_token: str, query: str, response: str):
        """
        Update the session's conversation context
        
        Args:
            session_token: Session token
            query: The user's query
            response: The system's response
        """
        session = self.get_session(session_token)
        if session:
            # Add the query-response pair to the context
            session["context"].append({
                "query": query,
                "response": response,
                "timestamp": datetime.now(UTC)
            })

            # Update the last interaction time
            session["last_interaction_at"] = datetime.now(UTC)
    
    def delete_session(self, session_token: str):
        """
        Delete a session
        
        Args:
            session_token: Session token to delete
        """
        if session_token in self.sessions:
            del self.sessions[session_token]
    
    def cleanup_expired_sessions(self):
        """
        Remove all expired sessions from memory
        """
        now = datetime.now(UTC)
        expired_tokens = []
        
        for token, session in self.sessions.items():
            if now > session["expires_at"]:
                expired_tokens.append(token)
        
        for token in expired_tokens:
            del self.sessions[token]
    
    def extend_session(self, session_token: str):
        """
        Extend session expiration time based on activity
        
        Args:
            session_token: Session token to extend
        """
        session = self.get_session(session_token)
        if session:
            # Extend the session expiry time
            session["expires_at"] = datetime.now(UTC) + timedelta(minutes=settings.SESSION_EXPIRY_MINUTES)
            session["last_interaction_at"] = datetime.now(UTC)