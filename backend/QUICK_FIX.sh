#!/bin/bash

# Quick fix script for Hugging Face deployment

echo "🚀 Quick Fix for Hugging Face Space Deployment"
echo "=============================================="
echo ""

# Navigate to the fixed repository
cd /tmp/rag-chatbot-api

echo "📊 Current status:"
git status --short
echo ""

echo "🔍 Recent commits:"
git log --oneline -3
echo ""

echo "Do you want to push these fixes to Hugging Face? (y/n)"
read -r response

if [[ "$response" =~ ^[Yy]$ ]]; then
    echo ""
    echo "🚀 Pushing to Hugging Face..."
    git push origin main

    if [ $? -eq 0 ]; then
        echo ""
        echo "✅ SUCCESS! Fixes have been pushed!"
        echo ""
        echo "📍 Next steps:"
        echo "1. Monitor rebuild: https://huggingface.co/spaces/SYED-OBAID/rag-chatbot-api"
        echo "2. Check logs: https://huggingface.co/spaces/SYED-OBAID/rag-chatbot-api/logs"
        echo "3. Wait 5-10 minutes for rebuild"
        echo ""
        echo "🧪 Test when ready:"
        echo "   curl https://syed-obaid-rag-chatbot-api.hf.space/"
        echo ""
    else
        echo ""
        echo "❌ Push failed!"
        echo ""
        echo "You may need to authenticate with Hugging Face."
        echo "Try using the web interface instead:"
        echo "   https://huggingface.co/spaces/SYED-OBAID/rag-chatbot-api/tree/main"
        echo ""
        echo "See FIX_DEPLOYMENT.md for manual upload instructions."
    fi
else
    echo ""
    echo "ℹ️  Push cancelled."
    echo ""
    echo "You can push later with:"
    echo "   cd /tmp/rag-chatbot-api && git push origin main"
    echo ""
    echo "Or see FIX_DEPLOYMENT.md for manual upload instructions."
fi
