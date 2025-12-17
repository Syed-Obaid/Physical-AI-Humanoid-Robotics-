import cohere
from typing import List, Dict
from ..config import settings


class LLMService:
    def __init__(self):
        # Initialize Cohere client with API key
        self.client = None
        self.model = settings.COHERE_MODEL
        self.connection_error = None

        try:
            # Only initialize if API key is provided
            if settings.COHERE_API_KEY:
                self.client = cohere.Client(settings.COHERE_API_KEY)
            else:
                self.connection_error = "No Cohere API key configured"
                print("Warning: No Cohere API key configured.")
                print("LLM operations will not be available until COHERE_API_KEY is set in .env")
        except Exception as e:
            self.connection_error = str(e)
            print(f"Warning: Could not initialize Cohere client: {e}")
            print("LLM operations will not be available until Cohere is properly configured.")
    
    def generate_response(self, query: str, context_chunks: List[Dict], max_tokens: int = 500) -> str:
        """
        Generate a response to the user's query based on the retrieved context

        Args:
            query: User's question
            context_chunks: List of relevant text chunks with context
            max_tokens: Maximum number of tokens in the response

        Returns:
            Generated response string
        """
        if self.client is None:
            raise Exception(f"LLM service not available: {self.connection_error}")

        # Format the context into a single string
        formatted_context = self._format_context(context_chunks)
        
        # Construct the prompt for the LLM
        prompt = f"""
        Based on the following context, please answer the user's question.
        If the answer is not in the context, please say so clearly.
        
        Context:
        {formatted_context}
        
        Question: {query}
        
        Answer:
        """
        
        # Generate the response using Cohere
        response = self.client.generate(
            model=self.model,
            prompt=prompt,
            max_tokens=max_tokens,
            temperature=0.3,  # Low temperature for more factual responses
            stop_sequences=["\n\n"]  # Stop at double newlines
        )
        
        # Extract and return the generated text
        return response.generations[0].text.strip()
    
    def _format_context(self, context_chunks: List[Dict]) -> str:
        """
        Format the context chunks into a single string
        
        Args:
            context_chunks: List of context chunks with text and metadata
        
        Returns:
            Formatted context string
        """
        formatted_chunks = []
        for chunk in context_chunks:
            formatted_chunks.append(f"Source: {chunk['text'][:200]}... (Position: {chunk['position']})")
        
        return "\n\n".join(formatted_chunks)
    
    def check_hallucination(self, response: str, context: List[Dict]) -> bool:
        """
        Basic check to see if the response is grounded in the context
        
        Args:
            response: The generated response
            context: The context used to generate the response
        
        Returns:
            True if potential hallucination detected, False otherwise
        """
        # This is a basic check - in a real implementation, you'd want more sophisticated validation
        # For now, we just return False to indicate no hallucination detected
        return False