from datetime import datetime
from typing import Optional
from pydantic import BaseModel


class UserQueryBase(BaseModel):
    session_id: str
    query_text: str
    processed: bool = False


class UserQueryCreate(UserQueryBase):
    pass


class UserQueryUpdate(BaseModel):
    processed: Optional[bool] = None
    processed_at: Optional[datetime] = None


class UserQuery(UserQueryBase):
    id: str
    created_at: datetime
    processed_at: Optional[datetime] = None

    class Config:
        from_attributes = True
