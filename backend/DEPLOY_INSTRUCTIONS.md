# Hugging Face Spaces Deployment Instructions

## Quick Deploy Steps

Run these commands from **PowerShell** in Windows:

```powershell
# Navigate to your backend directory
cd D:\roboticbook\frontend\backend

# Run the deployment script
bash deploy_to_hf.sh
```

When prompted for Hugging Face credentials:
- **Username**: SYED-OBAID (or your HF username)
- **Password**: Use your Hugging Face **Access Token** (not your password)
  - Get your token at: https://huggingface.co/settings/tokens
  - Create a new token with "write" access if you don't have one

## After Deployment

1. **Configure Secrets** at: https://huggingface.co/spaces/SYED-OBAID/rag-chatbot-api/settings

   Add these environment variables as **Secrets**:
   ```
   COHERE_API_KEY = <your-cohere-api-key>
   QDRANT_API_KEY = <your-qdrant-api-key>
   QDRANT_URL = <your-qdrant-cluster-url>
   ```

   Optional (will use defaults if not set):
   ```
   COHERE_MODEL = command-r
   COHERE_EMBED_MODEL = embed-english-v3.0
   SESSION_EXPIRY_MINUTES = 60
   ```

2. **Monitor Build**: https://huggingface.co/spaces/SYED-OBAID/rag-chatbot-api

3. **API will be live at**: https://syed-obaid-rag-chatbot-api.hf.space

4. **API Docs**: https://syed-obaid-rag-chatbot-api.hf.space/docs

## Testing Your API

```bash
# Check if running
curl https://syed-obaid-rag-chatbot-api.hf.space/

# View API documentation
# Open in browser: https://syed-obaid-rag-chatbot-api.hf.space/docs
```

## Manual Deployment (Alternative)

If the script doesn't work, deploy manually:

```powershell
# Clone your HF Space
git clone https://huggingface.co/spaces/SYED-OBAID/rag-chatbot-api hf-deploy
cd hf-deploy

# Copy files from backend
cp -r ../src .
cp ../Dockerfile .
cp ../requirements.txt .
cp ../README_HF.md README.md

# Commit and push
git add .
git commit -m "Deploy RAG Chatbot API"
git push origin main
```

## Troubleshooting

**Authentication failed?**
- Use your Hugging Face **Access Token** as password, not your account password
- Get token at: https://huggingface.co/settings/tokens

**Build failed?**
- Check logs at: https://huggingface.co/spaces/SYED-OBAID/rag-chatbot-api
- Verify all environment variables are set in Space settings
- Ensure API keys are valid

**API not responding?**
- Wait 5-10 minutes for first build to complete
- Check that all secrets (COHERE_API_KEY, QDRANT_API_KEY, QDRANT_URL) are configured
- View logs in the Space's Logs tab
