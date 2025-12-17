import cohere
from typing import List, Optional
from ..config import settings


class EmbeddingService:
    def __init__(self):
        # Initialize Cohere client with API key
        self.client = None
        self.model = settings.COHERE_EMBED_MODEL
        self.connection_error = None

        try:
            # Only initialize if API key is provided
            if settings.COHERE_API_KEY:
                self.client = cohere.Client(settings.COHERE_API_KEY)
            else:
                self.connection_error = "No Cohere API key configured"
                print("Warning: No Cohere API key configured.")
                print("Embedding operations will not be available until COHERE_API_KEY is set in .env")
        except Exception as e:
            self.connection_error = str(e)
            print(f"Warning: Could not initialize Cohere client: {e}")
            print("Embedding operations will not be available until Cohere is properly configured.")
    
    def create_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Create embeddings for a list of text chunks

        Args:
            texts: List of text chunks to embed

        Returns:
            List of embedding vectors
        """
        if self.client is None:
            raise Exception(f"Embedding service not available: {self.connection_error}")

        # Create embeddings using Cohere
        response = self.client.embed(
            texts=texts,
            model=self.model,
            input_type="search_document"
        )

        # Handle different response types from newer Cohere SDK
        if hasattr(response, 'embeddings'):
            return response.embeddings
        elif hasattr(response, 'embeddings_float'):
            return response.embeddings_float
        else:
            # Fallback
            return response.embeddings
    
    def embed_query(self, query: str) -> List[float]:
        """
        Create embedding for a query string

        Args:
            query: Query text to embed

        Returns:
            Embedding vector for the query
        """
        if self.client is None:
            raise Exception(f"Embedding service not available: {self.connection_error}")

        response = self.client.embed(
            texts=[query],
            model=self.model,
            input_type="search_query"
        )

        # Handle different response types from newer Cohere SDK
        if hasattr(response, 'embeddings'):
            return response.embeddings[0]
        elif hasattr(response, 'embeddings_float'):
            return response.embeddings_float[0]
        else:
            return response.embeddings[0]  # Fallback
    
    def embed_text(self, text: str) -> List[float]:
        """
        Create embedding for a single text

        Args:
            text: Text to embed

        Returns:
            Embedding vector for the text
        """
        if self.client is None:
            raise Exception(f"Embedding service not available: {self.connection_error}")

        response = self.client.embed(
            texts=[text],
            model=self.model,
            input_type="search_document"
        )

        # Handle different response types from newer Cohere SDK
        if hasattr(response, 'embeddings'):
            return response.embeddings[0]
        elif hasattr(response, 'embeddings_float'):
            return response.embeddings_float[0]
        else:
            return response.embeddings[0]  # Fallback