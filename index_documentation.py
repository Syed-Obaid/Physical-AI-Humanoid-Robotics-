#!/usr/bin/env python3
"""
Index all documentation content into the RAG chatbot backend.
This script reads all markdown files from the docs/ directory and creates a book in the backend.
"""

import os
import glob
import requests
import json

# Configuration
API_URL = "https://obaid987-robotics-book-chatbot-api.hf.space/v1"
DOCS_DIR = "docs"

def read_markdown_files(docs_dir):
    """Read all markdown files from the docs directory."""
    content_parts = []

    # Get all markdown files
    md_files = glob.glob(os.path.join(docs_dir, "**/*.md"), recursive=True)
    md_files.sort()  # Sort for consistent ordering

    print(f"Found {len(md_files)} markdown files")

    for md_file in md_files:
        try:
            with open(md_file, 'r', encoding='utf-8') as f:
                content = f.read()
                # Add file header for context
                relative_path = os.path.relpath(md_file, docs_dir)
                content_parts.append(f"\n\n--- FILE: {relative_path} ---\n\n{content}")
                print(f"  [OK] Read: {relative_path}")
        except Exception as e:
            print(f"  [ERR] Error reading {md_file}: {e}")

    return "\n\n".join(content_parts)

def create_book(title, content):
    """Create a new book in the backend."""
    url = f"{API_URL}/books"

    payload = {
        "title": title,
        "author": "ROBOX Team",
        "content": content,
        "metadata": {
            "source": "Docusaurus documentation",
            "version": "1.0.0",
            "indexed_at": "2025-12-19"
        }
    }

    print(f"\nCreating book: {title}")
    print(f"Content length: {len(content)} characters")

    try:
        response = requests.post(url, json=payload, timeout=300)  # 5 min timeout for large content
        response.raise_for_status()
        book_data = response.json()
        print(f"[SUCCESS] Book created successfully!")
        print(f"  Book ID: {book_data['id']}")
        print(f"  Title: {book_data['title']}")
        return book_data
    except requests.exceptions.RequestException as e:
        print(f"[ERROR] Error creating book: {e}")
        if hasattr(e, 'response') and e.response is not None:
            print(f"  Response: {e.response.text}")
        return None

def main():
    print("=" * 60)
    print("ROBOX Documentation Indexer")
    print("=" * 60)

    # Check if docs directory exists
    if not os.path.exists(DOCS_DIR):
        print(f"[ERROR] {DOCS_DIR} directory not found!")
        return

    # Read all documentation
    print(f"\nReading documentation from {DOCS_DIR}/...")
    content = read_markdown_files(DOCS_DIR)

    if not content:
        print("[ERROR] No content found!")
        return

    # Create book
    book = create_book("Physical AI & Humanoid Robotics - Complete Guide", content)

    if book:
        print("\n" + "=" * 60)
        print("SUCCESS! Documentation indexed successfully")
        print("=" * 60)
        print(f"\nYour chatbot is now ready to answer questions about:")
        print("  • Physical AI & Humanoid Robotics")
        print("  • ROS 2 Foundation")
        print("  • Digital Twin Simulation")
        print("  • Motion Planning with MoveIt 2")
        print("  • Embodied AI and Computer Vision")
        print("  • Integration and Deployment")
        print(f"\nBook ID: {book['id']}")
        print(f"\nUpdate your chatbot configuration to use this Book ID:")
        print(f"  File: static/js/chatbot-config.js")
        print(f"  Set: window.CHATBOT_BOOK_ID = '{book['id']}';")
    else:
        print("\n[ERROR] Failed to index documentation")

if __name__ == "__main__":
    main()
