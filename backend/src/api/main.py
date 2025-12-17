from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
# from .routers import chat, books
from src.api.routers import chat, books
from ..config import settings


def create_app():
    app = FastAPI(
        title=settings.PROJECT_NAME,
        version="1.0.0",
        description="API for the Retrieval-Augmented Generation chatbot embedded in digital books",
    )
    
    # Add CORS middleware for frontend integration
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],  # In production, specify exact origins
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )
    
    # Include API routers
    app.include_router(chat.router, prefix=settings.API_V1_STR)
    app.include_router(books.router, prefix=settings.API_V1_STR)
    
    @app.get("/")
    def read_root():
        return {"message": "RAG Chatbot API", "version": "1.0.0"}
    
    return app


app = create_app()