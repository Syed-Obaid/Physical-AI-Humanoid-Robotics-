from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List
from uuid import uuid4
from ..config import settings


class VectorDBClient:
    def __init__(self):
        # Initialize Qdrant client with settings
        self.collection_name = settings.QDRANT_COLLECTION_NAME
        self.embedding_dimension = settings.EMBEDDING_DIMENSION
        self.client = None
        self.connection_error = None

        try:
            # Try to connect to Qdrant - use in-memory mode if URL is placeholder
            if settings.QDRANT_URL == "https://your-cluster-url.gcp.cloud.qdrant.io" or not settings.QDRANT_URL:
                # Use in-memory mode for development
                self.client = QdrantClient(":memory:")
            else:
                self.client = QdrantClient(
                    url=settings.QDRANT_URL,
                    api_key=settings.QDRANT_API_KEY,
                    port=settings.QDRANT_PORT,
                    https=True
                )

            # Ensure the collection exists
            self._ensure_collection()
        except Exception as e:
            self.connection_error = str(e)
            # Log the error but don't crash - allow the server to start
            print(f"Warning: Could not connect to Qdrant: {e}")
            print("Vector database operations will not be available until Qdrant is properly configured.")
    
    def _ensure_collection(self):
        """Create the collection if it doesn't exist"""
        try:
            # Check if collection exists
            self.client.get_collection(self.collection_name)
        except:
            # Create collection if it doesn't exist
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=self.embedding_dimension,
                    distance=models.Distance.COSINE
                )
            )
    
    def store_embeddings(self, book_id: str, chunks: List[dict]) -> List[str]:
        """
        Store chunk embeddings in Qdrant

        Args:
            book_id: ID of the book these chunks belong to
            chunks: List of dictionaries containing 'text', 'position', and 'embedding'

        Returns:
            List of vector IDs for the stored chunks
        """
        if self.client is None:
            raise Exception(f"Vector database not available: {self.connection_error}")

        # Prepare points for insertion
        points = []
        vector_ids = []
        
        for chunk in chunks:
            vector_id = str(uuid4())
            vector_ids.append(vector_id)
            
            point = models.PointStruct(
                id=vector_id,
                vector=chunk["embedding"],
                payload={
                    "book_id": book_id,
                    "text": chunk["text"],
                    "position": chunk["position"]
                }
            )
            points.append(point)
        
        # Insert points into collection
        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )
        
        return vector_ids
    
    def search_similar(self, query_embedding: List[float], book_id: str, top_k: int = 5) -> List[dict]:
        """
        Search for similar chunks in the vector database for a specific book

        Args:
            query_embedding: Embedding vector for the query
            book_id: ID of the book to search within
            top_k: Number of results to return

        Returns:
            List of dictionaries containing text, position, and similarity score
        """
        if self.client is None:
            raise Exception(f"Vector database not available: {self.connection_error}")

        # Search for similar vectors
        results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            query_filter=models.Filter(
                must=[
                    models.FieldCondition(
                        key="book_id",
                        match=models.MatchValue(value=book_id)
                    )
                ]
            ),
            limit=top_k
        )
        
        # Format results
        formatted_results = []
        for result in results:
            formatted_results.append({
                "text": result.payload["text"],
                "position": result.payload["position"],
                "similarity_score": result.score,
                "vector_id": result.id
            })
        
        return formatted_results
    
    def delete_book_embeddings(self, book_id: str):
        """
        Delete all embeddings for a specific book

        Args:
            book_id: ID of the book to delete embeddings for
        """
        if self.client is None:
            raise Exception(f"Vector database not available: {self.connection_error}")

        # Find all points associated with this book
        response = self.client.scroll(
            collection_name=self.collection_name,
            scroll_filter=models.Filter(
                must=[
                    models.FieldCondition(
                        key="book_id",
                        match=models.MatchValue(value=book_id)
                    )
                ]
            ),
            limit=10000  # Adjust based on expected max chunks
        )
        
        # Extract IDs to delete
        ids_to_delete = [point.id for point in response[0]]
        
        if ids_to_delete:
            # Delete the points
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(
                    points=ids_to_delete
                )
            )