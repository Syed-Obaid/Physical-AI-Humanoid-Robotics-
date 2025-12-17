// ChatbotWidget.js
// A self-contained JavaScript widget for embedding in digital books

class RAGChatbot {
  constructor(options = {}) {
    this.containerId = options.containerId || 'chatbot-container';
    this.bookId = options.bookId;
    this.apiUrl = options.apiUrl || 'http://localhost:8000/v1';
    this.sessionId = null;
    this.messageHistory = [];
    
    this.init();
  }
  
  async init() {
    const container = document.getElementById(this.containerId);
    if (!container) {
      console.error(\`Container with ID '\${this.containerId}' not found\`);
      return;
    }
    
    // Create the chatbot UI
    this.createUI(container);
    
    // Create a new chat session
    await this.createSession();
  }
  
  createUI(container) {
    container.innerHTML = \`
      <div id="chatbot-ui" style="font-family: Arial, sans-serif; max-width: 500px; margin: 0 auto; border: 1px solid #ccc; border-radius: 8px; overflow: hidden;">
        <div id="chatbot-header" style="background-color: #4a6cf7; color: white; padding: 15px; text-align: center;">
          <h3>Book Assistant</h3>
        </div>
        <div id="chatbot-messages" style="height: 400px; overflow-y: auto; padding: 15px; background-color: #f9f9f9;">
          <div class="message bot-message">Hello! I'm your book assistant. Ask me anything about this book.</div>
        </div>
        <div id="chatbot-input-area" style="padding: 15px; border-top: 1px solid #eee; display: flex;">
          <input 
            type="text" 
            id="chatbot-input" 
            placeholder="Ask a question about the book..." 
            style="flex: 1; padding: 10px; border: 1px solid #ddd; border-radius: 4px; margin-right: 10px;"
          />
          <button 
            id="chatbot-send-btn" 
            style="padding: 10px 20px; background-color: #4a6cf7; color: white; border: none; border-radius: 4px; cursor: pointer;"
          >
            Send
          </button>
        </div>
      </div>
    \`;
    
    // Add event listeners
    const input = document.getElementById('chatbot-input');
    const sendBtn = document.getElementById('chatbot-send-btn');
    
    sendBtn.addEventListener('click', () => this.sendMessage());
    input.addEventListener('keypress', (e) => {
      if (e.key === 'Enter') {
        this.sendMessage();
      }
    });
  }
  
  async createSession() {
    try {
      const response = await fetch(\`\${this.apiUrl}/chat/sessions\`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({
          bookId: this.bookId
        })
      });
      
      if (response.ok) {
        const sessionData = await response.json();
        this.sessionId = sessionData.id;
        console.log('Chat session created:', this.sessionId);
      } else {
        console.error('Failed to create session:', response.status);
      }
    } catch (error) {
      console.error('Error creating session:', error);
    }
  }
  
  async sendMessage() {
    const input = document.getElementById('chatbot-input');
    const message = input.value.trim();
    
    if (!message) return;
    
    if (!this.sessionId) {
      alert('Chat session not initialized. Please refresh the page.');
      return;
    }
    
    // Add user message to UI
    this.addMessageToUI(message, 'user');
    input.value = '';
    
    try {
      // Show loading indicator
      this.showTypingIndicator();
      
      // Send the message to the API
      const response = await fetch(\`\${this.apiUrl}/chat/sessions/\${this.sessionId}/messages\`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({
          message: message
        })
      });
      
      if (response.ok) {
        const data = await response.json();
        
        // Remove typing indicator
        this.removeTypingIndicator();
        
        // Add bot response to UI
        this.addMessageToUI(data.response_text, 'bot');
        
        // Store in message history
        this.messageHistory.push({role: 'user', content: message});
        this.messageHistory.push({role: 'bot', content: data.response_text});
      } else {
        // Remove typing indicator
        this.removeTypingIndicator();
        
        // Add error message to UI
        this.addMessageToUI('Sorry, I encountered an error processing your request. Please try again.', 'bot');
      }
    } catch (error) {
      // Remove typing indicator
      this.removeTypingIndicator();
      
      // Add error message to UI
      this.addMessageToUI('Sorry, I encountered an error. Please try again.', 'bot');
      console.error('Error sending message:', error);
    }
  }
  
  addMessageToUI(message, sender) {
    const messagesContainer = document.getElementById('chatbot-messages');
    
    const messageElement = document.createElement('div');
    messageElement.classList.add('message');
    messageElement.classList.add(sender === 'user' ? 'user-message' : 'bot-message');
    messageElement.textContent = message;
    
    messagesContainer.appendChild(messageElement);
    
    // Scroll to the bottom
    messagesContainer.scrollTop = messagesContainer.scrollHeight;
  }
  
  showTypingIndicator() {
    const messagesContainer = document.getElementById('chatbot-messages');
    
    const typingElement = document.createElement('div');
    typingElement.id = 'typing-indicator';
    typingElement.classList.add('message', 'bot-message');
    typingElement.innerHTML = \`
      <div style="display: flex; align-items: center;">
        <div>Bot is typing</div>
        <div style="margin-left: 8px;">
          <span>.</span><span>.</span><span>.</span>
        </div>
      </div>
    \`;
    
    messagesContainer.appendChild(typingElement);
    messagesContainer.scrollTop = messagesContainer.scrollHeight;
  }
  
  removeTypingIndicator() {
    const typingIndicator = document.getElementById('typing-indicator');
    if (typingIndicator) {
      typingIndicator.remove();
    }
  }
}

// Make RAGChatbot available globally
window.RAGChatbot = RAGChatbot;
