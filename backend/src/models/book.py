from datetime import datetime
from typing import Optional, List
from pydantic import BaseModel


class BookBase(BaseModel):
    title: str
    author: str
    content: str
    metadata: Optional[dict] = None


class BookCreate(BookBase):
    pass


class BookUpdate(BaseModel):
    title: Optional[str] = None
    author: Optional[str] = None
    content: Optional[str] = None
    metadata: Optional[dict] = None


class Book(BookBase):
    id: str
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True
