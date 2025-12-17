#!/bin/bash
# Git-based deployment script for Hugging Face Spaces
# Run this if you prefer git over the Python upload script

set -e

echo "🚀 Hugging Face Git Deployment"
echo "=============================="
echo ""
echo "⚠️  You need your Hugging Face access token"
echo "Get it from: https://huggingface.co/settings/tokens"
echo ""
read -p "Enter your HF username (SYED-OBAID): " HF_USER
HF_USER=${HF_USER:-SYED-OBAID}

read -sp "Enter your HF token: " HF_TOKEN
echo ""

SPACE_NAME="rag-chatbot-api"
REPO_URL="https://${HF_USER}:${HF_TOKEN}@huggingface.co/spaces/${HF_USER}/${SPACE_NAME}"

# Create temp directory
TMP_DIR=$(mktemp -d)
echo "📁 Using temp directory: $TMP_DIR"

# Clone the repo
echo "📥 Cloning HF Space..."
git clone "$REPO_URL" "$TMP_DIR" 2>/dev/null || {
    echo "❌ Failed to clone. Check your credentials."
    rm -rf "$TMP_DIR"
    exit 1
}

# Copy files
echo "📋 Copying files..."
cd "$(dirname "$0")"

cp Dockerfile "$TMP_DIR/"
cp requirements.txt "$TMP_DIR/"
cp README.md "$TMP_DIR/"
cp .dockerignore "$TMP_DIR/" 2>/dev/null || true
cp .gitignore "$TMP_DIR/" 2>/dev/null || true

# Copy src directory
cp -r src "$TMP_DIR/"

# Commit and push
cd "$TMP_DIR"
git add .
git commit -m "Deploy RAG Chatbot API with all fixes" || {
    echo "✅ No changes to commit (already up to date)"
    rm -rf "$TMP_DIR"
    exit 0
}

echo "📤 Pushing to HF Space..."
git push origin main

# Cleanup
cd -
rm -rf "$TMP_DIR"

echo ""
echo "=============================="
echo "✅ Deployment Complete!"
echo "=============================="
echo ""
echo "📋 Next Steps:"
echo ""
echo "1. Configure Secrets:"
echo "   https://huggingface.co/spaces/${HF_USER}/${SPACE_NAME}/settings"
echo ""
echo "2. Monitor Build:"
echo "   https://huggingface.co/spaces/${HF_USER}/${SPACE_NAME}"
echo ""
echo "3. Test API (after build):"
echo "   https://${HF_USER,,}-${SPACE_NAME}.hf.space/docs"
echo ""
echo "🎉 Your Space will be live in 5-10 minutes!"
