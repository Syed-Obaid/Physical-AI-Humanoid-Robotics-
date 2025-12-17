#!/usr/bin/env python3
"""
Automated Hugging Face Spaces Uploader
Uploads all backend files to your HF Space
"""

import os
import sys
from pathlib import Path

try:
    from huggingface_hub import HfApi, login
except ImportError:
    print("❌ Error: huggingface_hub not installed")
    print("\nInstall it with:")
    print("  pip install huggingface_hub")
    sys.exit(1)

# Configuration
SPACE_ID = "SYED-OBAID/rag-chatbot-api"
REPO_TYPE = "space"

# Files and directories to upload
FILES_TO_UPLOAD = [
    "Dockerfile",
    "requirements.txt",
    "README.md",
    ".dockerignore",
    ".gitignore",
]

DIRS_TO_UPLOAD = [
    "src",
]

def main():
    # Get token from user
    print("🚀 Hugging Face Spaces Uploader")
    print("=" * 50)
    print(f"\nTarget Space: {SPACE_ID}")
    print("\nYou need a Hugging Face access token with 'write' permissions.")
    print("Get it from: https://huggingface.co/settings/tokens\n")

    token = input("Enter your HF token (or press Enter to use saved token): ").strip()

    try:
        # Login
        if token:
            login(token=token)
            print("✅ Logged in with provided token")
        else:
            # Try to use cached credentials
            print("✅ Using cached credentials")

        # Initialize API
        api = HfApi()

        print(f"\n📤 Uploading files to {SPACE_ID}...")
        print("-" * 50)

        # Upload individual files
        for file in FILES_TO_UPLOAD:
            if os.path.exists(file):
                print(f"⬆️  Uploading {file}...")
                api.upload_file(
                    path_or_fileobj=file,
                    path_in_repo=file,
                    repo_id=SPACE_ID,
                    repo_type=REPO_TYPE,
                    token=token if token else None,
                )
                print(f"   ✅ {file} uploaded")
            else:
                print(f"   ⚠️  {file} not found, skipping")

        # Upload directories
        for dir_name in DIRS_TO_UPLOAD:
            if os.path.exists(dir_name):
                print(f"⬆️  Uploading {dir_name}/ directory...")
                api.upload_folder(
                    folder_path=dir_name,
                    path_in_repo=dir_name,
                    repo_id=SPACE_ID,
                    repo_type=REPO_TYPE,
                    token=token if token else None,
                )
                print(f"   ✅ {dir_name}/ uploaded")
            else:
                print(f"   ⚠️  {dir_name}/ not found, skipping")

        print("\n" + "=" * 50)
        print("✅ Upload Complete!")
        print("=" * 50)
        print("\n📋 Next Steps:")
        print("\n1. Configure Secrets:")
        print(f"   Go to: https://huggingface.co/spaces/{SPACE_ID}/settings")
        print("   Add these secrets:")
        print("   - COHERE_API_KEY")
        print("   - QDRANT_API_KEY")
        print("   - QDRANT_URL")
        print("\n2. Monitor Build:")
        print(f"   https://huggingface.co/spaces/{SPACE_ID}")
        print("   Click 'Logs' to watch build progress")
        print("\n3. Test API:")
        print(f"   https://syed-obaid-rag-chatbot-api.hf.space/docs")
        print("\n🎉 Your Space will be live in 5-10 minutes!")

    except Exception as e:
        print(f"\n❌ Error: {e}")
        print("\nTroubleshooting:")
        print("1. Make sure your token has 'write' permissions")
        print("2. Verify the Space ID is correct: SYED-OBAID/rag-chatbot-api")
        print("3. Check your internet connection")
        sys.exit(1)

if __name__ == "__main__":
    # Change to script directory
    script_dir = Path(__file__).parent
    os.chdir(script_dir)

    main()
