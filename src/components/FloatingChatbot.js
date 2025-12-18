import React, { useState, useEffect, useRef } from 'react';
import styles from './FloatingChatbot.module.css';

/**
 * Professional Robot Icon - Modern, minimal robot head design
 * Used for the floating launcher button (56px)
 * Design: Rounded rectangle head with antenna, rectangular eyes, speaker grill mouth
 */
const RobotIcon = ({ size = 28 }) => (
  <svg
    width={size}
    height={size}
    viewBox="0 0 48 48"
    fill="none"
    xmlns="http://www.w3.org/2000/svg"
    aria-hidden="true"
  >
    {/* Antenna */}
    <circle cx="24" cy="6" r="3" fill="currentColor" />
    <rect x="22" y="8" width="4" height="6" rx="2" fill="currentColor" />
    {/* Head */}
    <rect x="8" y="14" width="32" height="28" rx="6" fill="currentColor" />
    {/* Eyes - cutout style */}
    <rect x="14" y="22" width="8" height="6" rx="2" fill="white" />
    <rect x="26" y="22" width="8" height="6" rx="2" fill="white" />
    {/* Mouth / speaker grill */}
    <rect x="16" y="34" width="16" height="3" rx="1.5" fill="white" />
  </svg>
);

/**
 * Header Robot Icon - Clean robot head for chat header
 * Used in the chat header (24px)
 */
const HeaderBotIcon = ({ size = 24 }) => (
  <svg
    width={size}
    height={size}
    viewBox="0 0 48 48"
    fill="none"
    xmlns="http://www.w3.org/2000/svg"
    aria-hidden="true"
  >
    {/* Antenna */}
    <circle cx="24" cy="6" r="3" fill="currentColor" />
    <rect x="22" y="8" width="4" height="6" rx="2" fill="currentColor" />
    {/* Head */}
    <rect x="8" y="14" width="32" height="28" rx="6" fill="currentColor" />
    {/* Eyes */}
    <rect x="14" y="22" width="8" height="6" rx="2" fill="white" />
    <rect x="26" y="22" width="8" height="6" rx="2" fill="white" />
    {/* Mouth */}
    <rect x="16" y="34" width="16" height="3" rx="1.5" fill="white" />
  </svg>
);

/**
 * Message Robot Icon - Smaller robot avatar for messages
 * Used next to bot messages (16px)
 */
const MessageBotIcon = ({ size = 16 }) => (
  <svg
    width={size}
    height={size}
    viewBox="0 0 48 48"
    fill="none"
    xmlns="http://www.w3.org/2000/svg"
    aria-hidden="true"
  >
    {/* Antenna */}
    <circle cx="24" cy="6" r="3" fill="currentColor" />
    <rect x="22" y="8" width="4" height="6" rx="2" fill="currentColor" />
    {/* Head */}
    <rect x="8" y="14" width="32" height="28" rx="6" fill="currentColor" />
    {/* Eyes */}
    <rect x="14" y="22" width="8" height="6" rx="2" fill="white" />
    <rect x="26" y="22" width="8" height="6" rx="2" fill="white" />
    {/* Mouth */}
    <rect x="16" y="34" width="16" height="3" rx="1.5" fill="white" />
  </svg>
);

/**
 * Send Icon - Paper plane
 */
const SendIcon = ({ size = 18 }) => (
  <svg
    width={size}
    height={size}
    viewBox="0 0 24 24"
    fill="none"
    xmlns="http://www.w3.org/2000/svg"
    aria-hidden="true"
  >
    <path
      d="M22 2L11 13"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    />
    <path
      d="M22 2L15 22L11 13L2 9L22 2Z"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    />
  </svg>
);

/**
 * Close Icon - X mark
 */
const CloseIcon = ({ size = 18 }) => (
  <svg
    width={size}
    height={size}
    viewBox="0 0 24 24"
    fill="none"
    xmlns="http://www.w3.org/2000/svg"
    aria-hidden="true"
  >
    <path
      d="M18 6L6 18M6 6l12 12"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    />
  </svg>
);

export default function FloatingChatbot({ bookId, apiUrl = 'http://localhost:8000/v1' }) {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    {
      role: 'bot',
      content: 'Hello! I\'m your AI assistant for robotics and embodied AI. How can I help you today?'
    }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [sessionId, setSessionId] = useState(null);
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  // Auto-scroll to bottom when new messages arrive
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Focus input when chat opens
  useEffect(() => {
    if (isOpen && inputRef.current) {
      setTimeout(() => inputRef.current?.focus(), 100);
    }
  }, [isOpen]);

  // Create a chat session when chat opens
  useEffect(() => {
    if (isOpen && !sessionId) {
      createSession();
    }
  }, [isOpen, sessionId]);

  const createSession = async () => {
    try {
      const response = await fetch(`${apiUrl}/chat/sessions`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({
          book_id: bookId || 'default-book'
        })
      });

      if (response.ok) {
        const sessionData = await response.json();
        setSessionId(sessionData.session_token);
      } else {
        console.error('Failed to create session:', response.status);
        setMessages(prev => [...prev, {
          role: 'bot',
          content: 'Sorry, I couldn\'t connect to the server. Please try again later.'
        }]);
      }
    } catch (error) {
      console.error('Error creating session:', error);
      setMessages(prev => [...prev, {
        role: 'bot',
        content: 'Connection error. Please check if the backend server is running.'
      }]);
    }
  };

  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    if (!sessionId) {
      setMessages(prev => [...prev, {
        role: 'bot',
        content: 'Session not initialized. Please close and reopen the chat.'
      }]);
      return;
    }

    const userMessage = inputValue.trim();
    setInputValue('');

    // Add user message to UI
    setMessages(prev => [...prev, { role: 'user', content: userMessage }]);
    setIsLoading(true);

    try {
      const response = await fetch(`${apiUrl}/chat/sessions/${sessionId}/messages`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({
          message: userMessage
        })
      });

      if (response.ok) {
        const data = await response.json();
        setMessages(prev => [...prev, {
          role: 'bot',
          content: data.response_text || data.message || 'I received your message.'
        }]);
      } else {
        setMessages(prev => [...prev, {
          role: 'bot',
          content: 'Sorry, I encountered an error. Please try again.'
        }]);
      }
    } catch (error) {
      console.error('Error sending message:', error);
      setMessages(prev => [...prev, {
        role: 'bot',
        content: 'Failed to send message. Please check your connection.'
      }]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  const handleClose = () => {
    setIsOpen(false);
  };

  const handleOpen = () => {
    setIsOpen(true);
  };

  return (
    <div className={styles.chatbotRoot}>
      {/* Chat Window */}
      <div
        className={`${styles.chatWindow} ${isOpen ? styles.chatWindowOpen : ''}`}
        role="dialog"
        aria-labelledby="chatbot-title"
        aria-modal="true"
        aria-hidden={!isOpen}
      >
        {/* Header */}
        <header className={styles.chatHeader}>
          <div className={styles.headerContent}>
            <div className={styles.botAvatar}>
              <HeaderBotIcon size={24} />
            </div>
            <div className={styles.headerText}>
              <h3 id="chatbot-title">ROBOX Assistant</h3>
              <div className={styles.headerStatus}>
                <span className={styles.statusDot} />
                <span className={styles.statusText}>Online</span>
              </div>
            </div>
          </div>
          <button
            type="button"
            className={styles.closeButton}
            onClick={handleClose}
            aria-label="Close chat"
          >
            <CloseIcon size={18} />
          </button>
        </header>

        {/* Messages */}
        <div
          className={styles.messagesContainer}
          role="log"
          aria-live="polite"
          aria-label="Chat messages"
        >
          {messages.map((msg, index) => (
            <div
              key={index}
              className={`${styles.message} ${
                msg.role === 'user' ? styles.userMessage : styles.botMessage
              }`}
            >
              {msg.role === 'bot' && (
                <div className={styles.messageBotAvatar} aria-hidden="true">
                  <MessageBotIcon size={16} />
                </div>
              )}
              <div className={styles.messageContent}>
                {msg.content}
              </div>
            </div>
          ))}

          {/* Typing indicator */}
          {isLoading && (
            <div className={`${styles.message} ${styles.botMessage}`}>
              <div className={styles.messageBotAvatar} aria-hidden="true">
                <MessageBotIcon size={16} />
              </div>
              <div
                className={styles.typingIndicator}
                role="status"
                aria-label="Assistant is typing"
              >
                <span />
                <span />
                <span />
              </div>
            </div>
          )}

          <div ref={messagesEndRef} aria-hidden="true" />
        </div>

        {/* Input Area */}
        <div className={styles.inputArea}>
          <div className={styles.inputWrapper}>
            <input
              ref={inputRef}
              type="text"
              placeholder="Ask me anything..."
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyDown={handleKeyDown}
              disabled={isLoading}
              className={styles.messageInput}
              aria-label="Type your message"
            />
            <button
              type="button"
              onClick={sendMessage}
              disabled={isLoading || !inputValue.trim()}
              className={styles.sendButton}
              aria-label="Send message"
            >
              <SendIcon size={18} />
            </button>
          </div>
          <div className={styles.inputFooter}>
            <span>Powered by RAG + Cohere AI</span>
          </div>
        </div>
      </div>

      {/* Floating Launcher Button */}
      <button
        type="button"
        className={`${styles.floatingButton} ${isOpen ? styles.floatingButtonHidden : ''}`}
        onClick={handleOpen}
        aria-label="Open AI chat assistant"
        aria-expanded={isOpen}
        aria-haspopup="dialog"
      >
        <RobotIcon size={28} />
        <span className={styles.notificationBadge} aria-hidden="true">
          1
        </span>
      </button>
    </div>
  );
}
