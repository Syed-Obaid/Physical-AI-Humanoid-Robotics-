# Chatbot Integration Setup Guide

This guide explains how to set up and use the integrated RAG chatbot in your Docusaurus frontend.

## Overview

The chatbot is a floating widget that appears in the bottom-right corner of every page. It connects to your FastAPI backend to provide AI-powered assistance using RAG (Retrieval-Augmented Generation).

## Features

- **Floating Chat Icon**: Professional chat button with pulse animation
- **Modern UI**: Clean, dark-themed interface matching your site design
- **Real-time Chat**: Instant responses from your RAG backend
- **Session Management**: Automatic session creation and management
- **Responsive Design**: Works on desktop and mobile devices
- **Smooth Animations**: Professional slide-in/out effects

## Setup Instructions

### 1. Configure Environment Variables

Copy the example environment file:

```bash
cp .env.example .env
```

Edit `.env` and set your backend API URL:

```env
REACT_APP_API_URL=http://localhost:8000/v1
REACT_APP_BOOK_ID=robotics-book
```

**For Production:**
```env
REACT_APP_API_URL=https://your-backend-domain.com/v1
REACT_APP_BOOK_ID=your-book-id
```

### 2. Start the Backend Server

Make sure your FastAPI backend is running:

```bash
cd backend
source venv/bin/activate  # or venv\Scripts\activate on Windows
uvicorn src.api.main:app --reload --host 0.0.0.0 --port 8000
```

### 3. Start the Frontend

```bash
npm install
npm start
```

The chatbot will now appear on all pages!

## File Structure

```
src/
├── components/
│   ├── FloatingChatbot.js          # Main chatbot component
│   └── FloatingChatbot.module.css  # Chatbot styles
└── theme/
    └── Root.js                      # Global wrapper (adds chatbot to all pages)
```

## Customization

### Change Colors

Edit `src/components/FloatingChatbot.module.css`:

```css
/* Change the primary color */
.floatingButton {
  background: linear-gradient(135deg, #your-color 0%, #your-color-light 100%);
}
```

### Change Position

In `FloatingChatbot.module.css`:

```css
.floatingButton {
  bottom: 24px;  /* Distance from bottom */
  right: 24px;   /* Distance from right */
}
```

### Change Book ID

In `src/theme/Root.js`:

```jsx
<FloatingChatbot
  bookId="your-custom-book-id"
  apiUrl={process.env.REACT_APP_API_URL || 'http://localhost:8000/v1'}
/>
```

### Disable on Specific Pages

You can conditionally render the chatbot based on the current route:

```jsx
import { useLocation } from '@docusaurus/router';

export default function Root({ children }) {
  const location = useLocation();
  const showChatbot = !location.pathname.startsWith('/admin');

  return (
    <>
      {children}
      {showChatbot && <FloatingChatbot />}
    </>
  );
}
```

## API Integration

The chatbot communicates with these backend endpoints:

1. **Create Session**: `POST /v1/chat/sessions`
   ```json
   {
     "bookId": "robotics-book"
   }
   ```

2. **Send Message**: `POST /v1/chat/sessions/{sessionId}/messages`
   ```json
   {
     "message": "What is ROS2?"
   }
   ```

## Troubleshooting

### Chatbot not appearing?

1. Check browser console for errors
2. Verify backend is running: `curl http://localhost:8000/`
3. Check CORS settings in backend (`backend/src/api/main.py`)

### Connection errors?

1. Verify `REACT_APP_API_URL` in `.env`
2. Check backend logs for errors
3. Ensure backend CORS allows your frontend domain

### Session errors?

1. Check backend logs for session creation errors
2. Verify book ID exists in your database
3. Try refreshing the page to create a new session

## Production Deployment

### Frontend

1. Set production API URL in `.env`:
   ```env
   REACT_APP_API_URL=https://api.yourdomain.com/v1
   ```

2. Build the site:
   ```bash
   npm run build
   ```

3. Deploy the `build/` folder to your hosting service

### Backend CORS

Update backend CORS settings for production (`backend/src/api/main.py`):

```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://yourdomain.com",
        "https://www.yourdomain.com"
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

## Support

If you encounter any issues:

1. Check the browser console for errors
2. Check backend logs: `docker logs backend` or check uvicorn output
3. Verify all environment variables are set correctly
4. Ensure backend and frontend are on compatible versions

---

Built with ❤️ using React, Docusaurus, FastAPI, and Cohere AI
