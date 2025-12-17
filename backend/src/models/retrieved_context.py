from datetime import datetime
from typing import Optional
from pydantic import BaseModel


class RetrievedContextBase(BaseModel):
    query_id: str
    chunk_id: str
    similarity_score: float
    context_text: str
    source_position: int


class RetrievedContextCreate(RetrievedContextBase):
    pass


class RetrievedContextUpdate(BaseModel):
    similarity_score: Optional[float] = None
    context_text: Optional[str] = None
    source_position: Optional[int] = None


class RetrievedContext(RetrievedContextBase):
    id: str
    retrieved_at: datetime

    class Config:
        from_attributes = True
