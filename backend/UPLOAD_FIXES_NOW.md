# 🚀 Upload Your Fixes to Hugging Face - SIMPLE METHOD

Since git authentication is needed, here's the **easiest way** to upload your fixes:

## 📤 Upload via Hugging Face Web Interface

### **Step 1: Update Dockerfile**

1. Go to: https://huggingface.co/spaces/SYED-OBAID/rag-chatbot-api/blob/main/Dockerfile

2. Click the **"Edit"** button (pencil icon)

3. **Replace ALL content** with this:

```dockerfile
# Use Python 3.11 slim image (more stable for HF Spaces)
FROM python:3.11-slim

# Set working directory
WORKDIR /app

# Set environment variables
ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PIP_NO_CACHE_DIR=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1

# Install system dependencies
RUN apt-get update && apt-get install -y \
    gcc \
    g++ \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements first for better caching
COPY requirements.txt .

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY . .

# Expose port 7860 (Hugging Face Spaces default port)
EXPOSE 7860

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=30s --retries=3 \
    CMD curl -f http://localhost:7860/ || exit 1

# Run the application
CMD ["uvicorn", "src.api.main:app", "--host", "0.0.0.0", "--port", "7860"]
```

4. Click **"Commit changes to main"**

---

### **Step 2: Update requirements.txt**

1. Go to: https://huggingface.co/spaces/SYED-OBAID/rag-chatbot-api/blob/main/requirements.txt

2. Click the **"Edit"** button

3. **Replace ALL content** with this:

```
fastapi==0.104.1
uvicorn[standard]==0.24.0
cohere>=5.20.0
qdrant-client==1.8.0
pydantic==2.5.0
pydantic-settings==2.1.0
python-dotenv==1.0.0
httpx==0.25.2
```

4. Click **"Commit changes to main"**

---

### **Step 3: Create missing __init__.py**

1. Go to: https://huggingface.co/spaces/SYED-OBAID/rag-chatbot-api/tree/main/src/vector_db

2. Click **"Add file"** → **"Create new file"**

3. In the **"Name your file..."** field, type: `__init__.py`

4. Leave the file content **empty** (or just add a comment like `# Vector DB module`)

5. Click **"Commit new file to main"**

---

## ✅ That's It!

After these 3 simple steps:

1. **Hugging Face will automatically rebuild** your Space (5-10 minutes)

2. **Monitor the build progress**:
   - Go to: https://huggingface.co/spaces/SYED-OBAID/rag-chatbot-api
   - Look for the "Building" indicator
   - Click "Logs" to see progress

3. **Expected log output**:
   ```
   ✅ Installing dependencies
   ✅ Building Docker image
   ✅ Starting application
   ✅ Application startup complete
   ✅ Uvicorn running on http://0.0.0.0:7860
   ```

4. **Test your API** once it says "Running":
   ```bash
   curl https://syed-obaid-rag-chatbot-api.hf.space/
   ```

   Expected response:
   ```json
   {
     "message": "RAG Chatbot API",
     "version": "1.0.0"
   }
   ```

---

## 🔑 Double-Check Your Secrets

While waiting for the build, verify these are set:

Go to: https://huggingface.co/spaces/SYED-OBAID/rag-chatbot-api/settings

**Repository secrets** section should have:

- ✅ `COHERE_API_KEY`
- ✅ `QDRANT_API_KEY`
- ✅ `QDRANT_URL`

If any are missing, add them now!

---

## 🧪 Test Your API (After Build Completes)

### Quick Health Check:
```bash
curl https://syed-obaid-rag-chatbot-api.hf.space/
```

### View API Documentation:
- Swagger UI: https://syed-obaid-rag-chatbot-api.hf.space/docs
- ReDoc: https://syed-obaid-rag-chatbot-api.hf.space/redoc

### Create a Test Book:
```bash
curl -X POST "https://syed-obaid-rag-chatbot-api.hf.space/v1/books" \
  -H "Content-Type: application/json" \
  -d '{
    "title": "Test Book",
    "author": "Test Author",
    "content": "This is a test book about AI and machine learning.",
    "metadata": {"genre": "tech"}
  }'
```

---

## 🐛 If You Still See Errors

Check the logs at: https://huggingface.co/spaces/SYED-OBAID/rag-chatbot-api/logs

Common issues:
- **Missing secrets**: Add them in Settings → Repository secrets
- **Build timeout**: Click "Factory reboot" in Settings
- **Port issues**: The fix above should resolve this

---

## 📞 Need Help?

- **Logs**: https://huggingface.co/spaces/SYED-OBAID/rag-chatbot-api/logs
- **Settings**: https://huggingface.co/spaces/SYED-OBAID/rag-chatbot-api/settings
- **Restart Space**: Settings → Factory reboot

---

Your fixes are ready to upload! Just follow the 3 steps above. 🚀
