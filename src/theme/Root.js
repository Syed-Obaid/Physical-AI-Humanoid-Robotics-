import React from 'react';
import FloatingChatbot from '../components/FloatingChatbot';
import BrowserOnly from '@docusaurus/BrowserOnly';

// Root component wraps the entire app
export default function Root({ children }) {
  return (
    <>
      {children}
      <BrowserOnly fallback={<div />}>
        {() => {
          const apiUrl = window.CHATBOT_API_URL || 'https://obaid987-robotics-book-chatbot-api.hf.space/v1';
          const bookId = window.CHATBOT_BOOK_ID || '878aa88f-4103-42bc-9786-05095e99abdc';
          return <FloatingChatbot bookId={bookId} apiUrl={apiUrl} />;
        }}
      </BrowserOnly>
    </>
  );
}
