# Hugging Face Deployment Checklist

## ✅ Pre-Deployment Checklist

### 1. Files Created
- [x] `Dockerfile` - Docker configuration for HF Spaces
- [x] `README_HF.md` - Space documentation with metadata
- [x] `.dockerignore` - Files to exclude from Docker build
- [x] `.env.example` - Example environment variables
- [x] `start.sh` - Startup script
- [x] `DEPLOYMENT.md` - Detailed deployment guide

### 2. Get API Keys
- [ ] **Cohere API Key** from [https://cohere.com/](https://cohere.com/)
  - Sign up → Dashboard → API Keys → Create Trial Key
  - Free tier: 100 API calls/month

- [ ] **Qdrant Cloud Credentials** from [https://cloud.qdrant.io/](https://cloud.qdrant.io/)
  - Sign up → Create Cluster (Free tier: 1GB)
  - Get Cluster URL and API Key from cluster settings

### 3. Test Locally (Optional)
- [ ] Build Docker image: `docker build -t rag-chatbot .`
- [ ] Run container: `docker run -p 7860:7860 --env-file .env rag-chatbot`
- [ ] Test API: `curl http://localhost:7860/`

## 📝 Deployment Steps

### Step 1: Create Hugging Face Space
- [ ] Go to [https://huggingface.co/spaces](https://huggingface.co/spaces)
- [ ] Click "Create new Space"
- [ ] Choose **Docker** as SDK
- [ ] Set Space name (e.g., `rag-chatbot-api`)
- [ ] Choose visibility (Public/Private)
- [ ] Click "Create Space"

### Step 2: Configure Secrets
Go to Space Settings → Variables and secrets, add:

**Required Secrets:**
- [ ] `COHERE_API_KEY` = `<your-cohere-key>`
- [ ] `QDRANT_API_KEY` = `<your-qdrant-key>`
- [ ] `QDRANT_URL` = `https://xxx.gcp.cloud.qdrant.io`

**Optional Secrets (with defaults):**
- [ ] `COHERE_MODEL` = `command-r` (default)
- [ ] `COHERE_EMBED_MODEL` = `embed-english-v3.0` (default)
- [ ] `QDRANT_COLLECTION_NAME` = `book_embeddings` (default)

### Step 3: Upload Files

**Option A: Git (Recommended)**
```bash
# Clone your Space
git clone https://huggingface.co/spaces/YOUR_USERNAME/YOUR_SPACE_NAME
cd YOUR_SPACE_NAME

# Copy backend files
cp -r /path/to/backend/src .
cp /path/to/backend/Dockerfile .
cp /path/to/backend/requirements.txt .
cp /path/to/backend/README_HF.md README.md

# Commit and push
git add .
git commit -m "Deploy RAG Chatbot API"
git push
```

**Option B: Web Interface**
- [ ] Upload `Dockerfile`
- [ ] Upload `requirements.txt`
- [ ] Upload entire `src/` folder
- [ ] Rename `README_HF.md` to `README.md` and upload

### Step 4: Wait for Build
- [ ] Monitor build logs in Space
- [ ] Build typically takes 5-10 minutes
- [ ] Check for any errors in logs

### Step 5: Test Deployment
- [ ] Visit: `https://YOUR_USERNAME-YOUR_SPACE_NAME.hf.space/`
- [ ] Check API docs: `.../docs`
- [ ] Test endpoints with curl or Postman

## 🧪 Testing Your Deployed API

### 1. Health Check
```bash
curl https://YOUR_USERNAME-YOUR_SPACE_NAME.hf.space/
```

Expected response:
```json
{
  "message": "RAG Chatbot API",
  "version": "1.0.0"
}
```

### 2. Create a Book
```bash
curl -X POST "https://YOUR_USERNAME-YOUR_SPACE_NAME.hf.space/v1/books" \
  -H "Content-Type: application/json" \
  -d '{
    "title": "Test Book",
    "author": "Test Author",
    "content": "This is a test book about artificial intelligence and machine learning.",
    "metadata": {"genre": "tech"}
  }'
```

### 3. List Books
```bash
curl https://YOUR_USERNAME-YOUR_SPACE_NAME.hf.space/v1/books
```

### 4. Create Chat Session
```bash
# Use book_id from previous response
curl -X POST "https://YOUR_USERNAME-YOUR_SPACE_NAME.hf.space/v1/chat/sessions" \
  -H "Content-Type: application/json" \
  -d '{"book_id": "YOUR_BOOK_ID"}'
```

### 5. Send Message
```bash
# Use session_token from previous response
curl -X POST "https://YOUR_USERNAME-YOUR_SPACE_NAME.hf.space/v1/chat/sessions/YOUR_SESSION_TOKEN/messages" \
  -H "Content-Type: application/json" \
  -d '{"message": "What is this book about?"}'
```

## 🐛 Troubleshooting

### Build Fails
- [ ] Check logs for specific error messages
- [ ] Verify all required files are present
- [ ] Ensure requirements.txt has correct dependencies
- [ ] Check Dockerfile syntax

### API Returns 500 Errors
- [ ] Verify all environment variables are set
- [ ] Check Cohere API key is valid
- [ ] Verify Qdrant credentials are correct
- [ ] Check application logs in Space

### Slow Response Times
- [ ] Consider upgrading Space hardware
- [ ] Optimize chunking settings (reduce CHUNK_SIZE)
- [ ] Reduce number of embeddings processed at once

## 📊 Post-Deployment

### Monitoring
- [ ] Check Space metrics regularly
- [ ] Monitor API response times
- [ ] Review error logs

### Optimization
- [ ] Add request caching
- [ ] Implement connection pooling
- [ ] Add API rate limiting
- [ ] Set up error tracking (e.g., Sentry)

### Security
- [ ] Review CORS settings for production
- [ ] Add authentication if needed
- [ ] Regularly rotate API keys
- [ ] Keep dependencies updated

## 🔗 Important Links

- **Your Space**: `https://huggingface.co/spaces/YOUR_USERNAME/YOUR_SPACE_NAME`
- **API Endpoint**: `https://YOUR_USERNAME-YOUR_SPACE_NAME.hf.space`
- **API Docs**: `https://YOUR_USERNAME-YOUR_SPACE_NAME.hf.space/docs`
- **Cohere Dashboard**: [https://dashboard.cohere.com/](https://dashboard.cohere.com/)
- **Qdrant Console**: [https://cloud.qdrant.io/](https://cloud.qdrant.io/)

## 📚 Documentation

- Full deployment guide: See `DEPLOYMENT.md`
- API documentation: Available at `/docs` endpoint
- Hugging Face Spaces: [https://huggingface.co/docs/hub/spaces](https://huggingface.co/docs/hub/spaces)

## ✨ Tips

1. **Start with free tiers** to test everything
2. **Monitor your usage** to avoid unexpected costs
3. **Use Secrets** for all sensitive data
4. **Test locally first** if possible
5. **Keep your Space public** to share with others (or private for sensitive projects)

---

**Ready to deploy?** Follow the steps above and you'll have your RAG Chatbot API live on Hugging Face Spaces! 🚀
