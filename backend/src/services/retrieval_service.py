from typing import List, Dict
from ..vector_db.vector_db_client import VectorDBClient
from ..services.embedding_service import EmbeddingService


class RetrievalService:
    def __init__(self):
        self.vector_db = VectorDBClient()
        self.embedding_service = EmbeddingService()
    
    def retrieve_relevant_context(self, query: str, book_id: str, top_k: int = 5) -> List[Dict]:
        """
        Retrieve relevant context chunks for a query from a specific book
        
        Args:
            query: User's question
            book_id: ID of the book to search in
            top_k: Number of relevant chunks to retrieve
        
        Returns:
            List of relevant chunks with text, position and similarity score
        """
        # Embed the query
        query_embedding = self.embedding_service.embed_query(query)
        
        # Search for similar chunks in the vector database
        results = self.vector_db.search_similar(
            query_embedding=query_embedding,
            book_id=book_id,
            top_k=top_k
        )
        
        return results
    
    def add_book_content(self, book_id: str, content: str, chunk_size: int = 512, overlap: int = 50) -> List[str]:
        """
        Add book content to the vector database by chunking and embedding
        
        Args:
            book_id: ID of the book
            content: Full text content of the book
            chunk_size: Size of each text chunk
            overlap: Overlap between chunks
        
        Returns:
            List of vector IDs for the stored chunks
        """
        # Split the content into overlapping chunks
        chunks = self._chunk_text(content, chunk_size, overlap)
        
        # Embed each chunk
        texts_to_embed = [chunk["text"] for chunk in chunks]
        embeddings = self.embedding_service.create_embeddings(texts_to_embed)
        
        # Add chunk position and embedding to the chunks
        for i, chunk in enumerate(chunks):
            chunk["embedding"] = embeddings[i]
        
        # Store embeddings in vector database
        vector_ids = self.vector_db.store_embeddings(book_id, chunks)
        
        return vector_ids
    
    def _chunk_text(self, text: str, chunk_size: int, overlap: int) -> List[Dict]:
        """
        Split text into overlapping chunks
        
        Args:
            text: Text to chunk
            chunk_size: Size of each chunk
            overlap: Overlap between chunks
        
        Returns:
            List of chunk dictionaries with text, position, and other metadata
        """
        if len(text) <= chunk_size:
            return [{"text": text, "position": 0}]
        
        chunks = []
        start = 0
        position = 0
        
        while start < len(text):
            # Determine the end position
            end = start + chunk_size
            
            # If this is not the last chunk, try to break at a sentence or word boundary
            if end < len(text):
                # Look for a good breaking point
                snippet = text[start:end]
                last_sentence = max(snippet.rfind('.'), snippet.rfind('!'), snippet.rfind('?'))
                last_space = snippet.rfind(' ')
                
                # Prefer sentence endings, but use word boundaries if needed
                if last_sentence > len(snippet) // 2:  # If sentence break is in the second half
                    end = start + last_sentence + 1
                elif last_space > len(snippet) // 2:  # If word break is in the second half
                    end = start + last_space
            
            # Extract the chunk
            chunk_text = text[start:end].strip()
            if chunk_text:  # Only add non-empty chunks
                chunks.append({
                    "text": chunk_text,
                    "position": position
                })
                position += 1
            
            # Move start position, considering overlap
            start = end - overlap
        
        return chunks