from datetime import datetime
from typing import Optional, List
from pydantic import BaseModel


class ChunkBase(BaseModel):
    book_id: str
    text: str
    position: int
    embedding: List[float]
    vector_id: str


class ChunkCreate(ChunkBase):
    pass


class ChunkUpdate(BaseModel):
    text: Optional[str] = None
    position: Optional[int] = None
    embedding: Optional[List[float]] = None
    vector_id: Optional[str] = None


class Chunk(ChunkBase):
    id: str
    created_at: datetime

    class Config:
        from_attributes = True
