import React from 'react';
import FloatingChatbot from '../components/FloatingChatbot';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

// Root component wraps the entire app
export default function Root({ children }) {
  // Get API URL - safe for both server and browser
  const getApiConfig = () => {
    // Use Docusaurus's ExecutionEnvironment to check if we're in browser
    if (ExecutionEnvironment.canUseDOM) {
      // In browser, check for injected config or use default
      return {
        apiUrl: window.CHATBOT_API_URL || 'https://obaid987-robotics-book-chatbot-api.hf.space/v1',
        bookId: window.CHATBOT_BOOK_ID || '878aa88f-4103-42bc-9786-05095e99abdc'
      };
    }
    // Server-side rendering fallback
    return {
      apiUrl: 'https://obaid987-robotics-book-chatbot-api.hf.space/v1',
      bookId: '878aa88f-4103-42bc-9786-05095e99abdc'
    };
  };

  const { apiUrl, bookId } = getApiConfig();

  return (
    <>
      {children}
      <FloatingChatbot
        bookId={bookId}
        apiUrl={apiUrl}
      />
    </>
  );
}
