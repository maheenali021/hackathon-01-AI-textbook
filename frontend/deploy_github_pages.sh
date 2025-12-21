#!/bin/bash
# Script to deploy the Docusaurus site to GitHub Pages

set -e  # Exit on any error

echo "🚀 Deploying frontend to GitHub Pages..."

# Navigate to the frontend directory
cd "$(dirname "$0")"

# Check if build directory exists
if [ ! -d "build" ]; then
    echo "❌ Build directory does not exist. Please run 'npm run build' first."
    exit 1
fi

# Check if we're in a git repository
if [ ! -d ".git" ]; then
    echo "❌ Not in a git repository. Please initialize git in the frontend directory."
    exit 1
fi

echo "✅ Repository check passed"

# Add the built files to git
echo "📦 Adding built files to git..."
git add build/

# Commit the changes
git commit -m "Deploy: Update built site with latest changes [skip ci]" || echo "No changes to commit"

# Push to the gh-pages branch (GitHub Pages deployment)
echo "📤 Pushing to gh-pages branch..."
git subtree push --prefix build origin gh-pages

echo "✅ Frontend deployed successfully to GitHub Pages!"
echo ""
echo "🌐 Your site is now available at:"
echo "   https://maheenali021.github.io/hackathon-01-AI-textbook/"
echo ""
echo "💡 The chatbot functionality is now integrated and will connect to your backend API"
echo "   Make sure your backend is running at the configured endpoint."