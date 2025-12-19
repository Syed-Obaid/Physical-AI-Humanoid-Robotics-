import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

// Root component wraps the entire app
export default function Root({ children }) {
  return (
    <>
      {children}
      <BrowserOnly>
        {() => {
          // Import FloatingChatbot only in browser environment
          const FloatingChatbot = require('../components/FloatingChatbot').default;
          const apiUrl = typeof window !== 'undefined' && window.CHATBOT_API_URL
            ? window.CHATBOT_API_URL
            : 'https://obaid987-robotics-book-chatbot-api.hf.space/v1';
          const bookId = typeof window !== 'undefined' && window.CHATBOT_BOOK_ID
            ? window.CHATBOT_BOOK_ID
            : '72fe6bba-7264-4721-80cf-d7cf1bf0caee';
          return <FloatingChatbot bookId={bookId} apiUrl={apiUrl} />;
        }}
      </BrowserOnly>
    </>
  );
}
