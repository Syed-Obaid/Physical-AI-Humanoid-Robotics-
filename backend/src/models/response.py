from datetime import datetime
from typing import Optional, List
from pydantic import BaseModel


class ResponseBase(BaseModel):
    query_id: str
    response_text: str
    sources: List[str]  # IDs of RetrievedContext used
    has_citations: bool = False


class ResponseCreate(ResponseBase):
    pass


class ResponseUpdate(BaseModel):
    response_text: Optional[str] = None
    sources: Optional[List[str]] = None
    has_citations: Optional[bool] = None


class Response(ResponseBase):
    id: str
    generated_at: datetime

    class Config:
        from_attributes = True
