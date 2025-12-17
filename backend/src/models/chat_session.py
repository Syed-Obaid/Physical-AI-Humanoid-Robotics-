from datetime import datetime
from typing import Optional
from pydantic import BaseModel


class ChatSessionBase(BaseModel):
    session_token: str
    book_id: str
    active: bool = True


class ChatSessionCreate(BaseModel):
    book_id: str


class ChatSessionUpdate(BaseModel):
    active: Optional[bool] = None


class ChatSession(ChatSessionBase):
    id: str
    created_at: datetime
    last_interaction_at: datetime
    expires_at: datetime

    class Config:
        from_attributes = True
