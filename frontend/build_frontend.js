#!/usr/bin/env node
/**
 * Script to build the frontend for deployment
 * This prepares the Docusaurus site for GitHub Pages deployment
 */

const { execSync } = require('child_process');
const fs = require('fs');
const path = require('path');

console.log('🚀 Building frontend for deployment...');

try {
  // Check if we're in the correct directory
  const packageJsonPath = path.join(process.cwd(), 'package.json');
  if (!fs.existsSync(packageJsonPath)) {
    throw new Error('No package.json found. Please run this script from the frontend directory.');
  }

  console.log('✅ Package.json found');

  // Read package.json to verify it's the right project
  const packageJson = JSON.parse(fs.readFileSync(packageJsonPath, 'utf8'));
  if (!packageJson.name || !packageJson.name.includes('docusaurus')) {
    console.warn('⚠️  Warning: This might not be a Docusaurus project');
  }

  console.log('🧹 Cleaning previous build...');
  if (fs.existsSync('./build')) {
    execSync('rmdir /s /q build', { stdio: 'inherit' });
  }

  console.log('📦 Installing dependencies...');
  execSync('npm install', { stdio: 'inherit' });

  console.log('🏗️  Building static site...');
  execSync('npm run build', { stdio: 'inherit' });

  console.log('✅ Build completed successfully!');
  console.log('📁 The built site is in the "build" directory');
  console.log('');
  console.log('📝 Next steps:');
  console.log('   1. The site is ready for deployment to GitHub Pages');
  console.log('   2. The chatbot component is integrated and will connect to the backend');
  console.log('   3. If deploying locally, make sure the backend is running on http://localhost:8001');
  console.log('');
  console.log('🌐 For GitHub Pages deployment:');
  console.log('   - The site is configured to deploy from the gh-pages branch');
  console.log('   - Base URL is set to /hackathon-01-AI-textbook/');

} catch (error) {
  console.error('❌ Build failed:', error.message);
  process.exit(1);
}