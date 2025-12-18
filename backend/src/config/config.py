from pydantic_settings import BaseSettings, SettingsConfigDict
from typing import Optional


class Settings(BaseSettings):
    model_config = SettingsConfigDict(env_file='.env', env_file_encoding='utf-8')

    # Database settings
    DATABASE_URL: str = "postgresql://user:password@localhost/dbname"

    # API settings
    API_V1_STR: str = "/v1"
    PROJECT_NAME: str = "RAG Chatbot API"

    # Security settings
    SECRET_KEY: str = "your-secret-key-here"
    ALGORITHM: str = "HS256"
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 30

    # Session settings
    SESSION_EXPIRY_MINUTES: int = 60  # Session expires after 1 hour of inactivity

    # Cohere settings
    COHERE_API_KEY: str = ""
    COHERE_MODEL: str = "command-a-03-2025"  # Default model for generation (Command A - Dec 2025)
    COHERE_EMBED_MODEL: str = "embed-english-v3.0"  # Default model for embeddings

    # Qdrant settings
    QDRANT_API_KEY: str = ""
    QDRANT_URL: str = "https://your-cluster-url.gcp.cloud.qdrant.io"
    QDRANT_PORT: int = 6333
    QDRANT_COLLECTION_NAME: str = "book_embeddings"

    # Performance settings
    RESPONSE_TIMEOUT: int = 5  # 5 seconds timeout
    RATE_LIMIT_REQUESTS: int = 10  # 10 requests
    RATE_LIMIT_WINDOW: int = 60  # per 60 seconds

    # Vector settings
    EMBEDDING_DIMENSION: int = 1024  # Default embedding dimension size

    # Book chunking settings
    CHUNK_SIZE: int = 512  # Characters per chunk
    CHUNK_OVERLAP: int = 50  # Overlap between chunks in characters


settings = Settings()
