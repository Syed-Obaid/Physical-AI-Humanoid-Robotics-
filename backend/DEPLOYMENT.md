# Deploying to Hugging Face Spaces

This guide will walk you through deploying your RAG Chatbot API to Hugging Face Spaces.

## Prerequisites

1. **Hugging Face Account**: Sign up at [https://huggingface.co/](https://huggingface.co/)
2. **API Keys**:
   - **Cohere API Key**: Get from [https://cohere.com/](https://cohere.com/) (Free tier available)
   - **Qdrant API Key**: Get from [https://cloud.qdrant.io/](https://cloud.qdrant.io/) (Free tier available)

## Step 1: Create API Keys

### Cohere API Key
1. Go to [https://cohere.com/](https://cohere.com/)
2. Sign up or log in
3. Navigate to API Keys section
4. Create a new API key
5. Copy the key (you'll need it later)

### Qdrant Cloud Setup
1. Go to [https://cloud.qdrant.io/](https://cloud.qdrant.io/)
2. Sign up or log in
3. Create a new cluster (free tier is sufficient)
4. Once created, get:
   - Cluster URL (e.g., `https://xxx.gcp.cloud.qdrant.io`)
   - API Key (from cluster settings)

## Step 2: Create a Hugging Face Space

1. Go to [https://huggingface.co/spaces](https://huggingface.co/spaces)
2. Click **"Create new Space"**
3. Fill in the details:
   - **Space name**: `rag-chatbot-api` (or your preferred name)
   - **License**: MIT
   - **Select SDK**: Choose **Docker**
   - **Space hardware**: CPU basic (free) is sufficient for testing
   - Set visibility (Public or Private)
4. Click **"Create Space"**

## Step 3: Configure Environment Variables

1. In your Space, go to **Settings** → **Variables and secrets**
2. Add the following secrets (click **"New secret"** for each):

   ```
   COHERE_API_KEY = <your-cohere-api-key>
   QDRANT_API_KEY = <your-qdrant-api-key>
   QDRANT_URL = <your-qdrant-cluster-url>
   ```

3. Optional secrets (use defaults if not specified):
   ```
   COHERE_MODEL = command-r
   COHERE_EMBED_MODEL = embed-english-v3.0
   QDRANT_COLLECTION_NAME = book_embeddings
   SESSION_EXPIRY_MINUTES = 60
   ```

## Step 4: Deploy Your Code

### Option A: Using Git (Recommended)

1. **Clone your Space repository**:
   ```bash
   git clone https://huggingface.co/spaces/YOUR_USERNAME/YOUR_SPACE_NAME
   cd YOUR_SPACE_NAME
   ```

2. **Copy your backend files**:
   ```bash
   # Copy all necessary files from your backend directory
   cp -r /path/to/your/backend/* .

   # Make sure you have these essential files:
   # - Dockerfile
   # - requirements.txt
   # - src/ (entire directory)
   # - README_HF.md (rename to README.md)
   ```

3. **Rename README**:
   ```bash
   mv README_HF.md README.md
   ```

4. **Commit and push**:
   ```bash
   git add .
   git commit -m "Initial deployment of RAG Chatbot API"
   git push
   ```

### Option B: Using Hugging Face Web Interface

1. Go to your Space → **Files** tab
2. Click **"Add file"** → **"Upload files"**
3. Upload the following files/folders:
   - `Dockerfile`
   - `requirements.txt`
   - `src/` (entire folder with all subfolders)
   - Rename `README_HF.md` to `README.md` and upload
4. Click **"Commit changes to main"**

## Step 5: Wait for Build

1. After pushing, Hugging Face will automatically start building your Space
2. You can monitor the build progress in the **Logs** section
3. The first build may take 5-10 minutes
4. Once complete, your Space will be running at:
   ```
   https://YOUR_USERNAME-YOUR_SPACE_NAME.hf.space
   ```

## Step 6: Test Your Deployment

### Test the API

1. **Check if it's running**:
   ```bash
   curl https://YOUR_USERNAME-YOUR_SPACE_NAME.hf.space/
   ```

2. **Access API Documentation**:
   - Swagger UI: `https://YOUR_USERNAME-YOUR_SPACE_NAME.hf.space/docs`
   - ReDoc: `https://YOUR_USERNAME-YOUR_SPACE_NAME.hf.space/redoc`

3. **Test book creation**:
   ```bash
   curl -X POST "https://YOUR_USERNAME-YOUR_SPACE_NAME.hf.space/v1/books" \
     -H "Content-Type: application/json" \
     -d '{
       "title": "Test Book",
       "author": "Test Author",
       "content": "This is test content...",
       "metadata": {}
     }'
   ```

## Troubleshooting

### Build Fails

1. Check the **Logs** in your Space
2. Common issues:
   - Missing dependencies in `requirements.txt`
   - Syntax errors in code
   - Port configuration (must use 7860)

### API Not Responding

1. Verify environment variables are set correctly in Space settings
2. Check logs for startup errors
3. Ensure Cohere and Qdrant credentials are valid

### Memory Issues

1. If you get memory errors, consider:
   - Upgrading to a paid Space tier with more resources
   - Optimizing your code to use less memory
   - Reducing batch sizes for embeddings

## Updating Your Space

To update your deployed API:

```bash
# Make changes to your code locally
# Test locally first

# Commit and push changes
git add .
git commit -m "Update: description of changes"
git push
```

Hugging Face will automatically rebuild and redeploy your Space.

## Monitoring

- **Logs**: Check real-time logs in the Space's **Logs** tab
- **Usage**: Monitor API usage in Space analytics
- **Health**: The API includes a health check endpoint at `/`

## Cost Considerations

- **Hugging Face Spaces**: Free tier available (CPU basic)
- **Cohere**: Free tier includes 100 API calls/month
- **Qdrant**: Free tier includes 1GB cluster

For production use, consider upgrading to paid tiers for better performance and higher limits.

## Security Best Practices

1. ✅ Always use Secrets for API keys (never commit them)
2. ✅ Keep your Space private if handling sensitive data
3. ✅ Regularly update dependencies for security patches
4. ✅ Monitor API usage for unusual activity
5. ✅ Implement rate limiting in production

## Next Steps

- Add authentication to your API
- Implement rate limiting
- Add monitoring and alerting
- Set up CI/CD for automated deployments
- Add comprehensive logging

## Support

- Hugging Face Spaces Docs: [https://huggingface.co/docs/hub/spaces](https://huggingface.co/docs/hub/spaces)
- Cohere Docs: [https://docs.cohere.com/](https://docs.cohere.com/)
- Qdrant Docs: [https://qdrant.tech/documentation/](https://qdrant.tech/documentation/)
