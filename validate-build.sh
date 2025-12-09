#!/bin/bash

# Build validation script for Docusaurus project
echo "Starting build validation..."

# Run the Docusaurus build command
yarn build

# Check if the build was successful
if [ $? -eq 0 ]; then
    echo "✓ Build validation passed successfully"
    exit 0
else
    echo "✗ Build validation failed"
    exit 1
fi