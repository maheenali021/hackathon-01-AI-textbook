# Quickstart Guide: Physical AI & Humanoid Robotics Educational Book

## Overview
This guide provides a quick setup and development workflow for the Physical AI & Humanoid Robotics Educational Book project. The project uses Docusaurus as the documentation framework and follows a modular content structure.

## Prerequisites

### System Requirements
- Node.js 18.x or higher
- npm or yarn package manager
- Git for version control
- A modern web browser for testing

### Development Tools
- Code editor (VS Code recommended)
- Terminal/command line interface
- GitHub account for version control

## Getting Started

### 1. Clone the Repository
```bash
git clone <repository-url>
cd hackathon-01-AI-textbook
git checkout 1-physical-ai-book  # Switch to the feature branch
```

### 2. Setup Frontend (Docusaurus)
```bash
# Navigate to frontend directory
cd frontend

# Install dependencies
npm install
# or
yarn install
```

### 3. Start Development Server
```bash
# From the frontend directory
npm run start
# or
yarn start
```

This will start the Docusaurus development server with hot reloading. The site will be available at `http://localhost:3000`.

### 4. Project Structure
```
frontend/
├── docs/                 # Educational content organized by modules
│   ├── intro/           # Introduction module
│   ├── module1/         # ROS 2 module
│   ├── module2/         # Digital Twin module
│   ├── module3/         # AI-Robot Brain module
│   ├── module4/         # VLA module
│   └── capstone/        # Capstone project module
├── src/                 # Custom React components and theme
├── static/              # Static assets (images, documents)
├── docusaurus.config.js # Docusaurus configuration
├── sidebars.js          # Navigation sidebar configuration
└── package.json         # Project dependencies
```

## Content Creation Workflow

### 1. Create a New Chapter
```bash
# Navigate to the appropriate module directory
cd frontend/docs/module1  # or module2, module3, etc.

# Create a new markdown file with proper naming convention
# Format: [order]-[short-description].md
touch 03-new-chapter-topic.md
```

### 2. Chapter Template
Create your chapter file using this template:

```markdown
---
title: Your Chapter Title
sidebar_label: Short Label
description: Brief description of the chapter content
keywords: [relevant, keywords, for, search]
---

# Chapter Title

## Learning Objectives
- Objective 1
- Objective 2
- Objective 3

## Content

Your chapter content here...

### Code Example
import CodeBlock from '@theme/CodeBlock';
import { javascript } from 'prismjs/components/prism-javascript';

```js
// Your code example here
const example = "Hello, ROS 2!";
console.log(example);
```

### Interactive Element
{/* You can include React components for interactive elements */}
```

### 3. Add to Navigation
Update the `sidebars.js` file to include your new chapter in the navigation:

```javascript
module.exports = {
  docs: [
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module1/01-what-is-robotic-nervous-system',
        'module1/02-setting-up-ros2-humble',
        'module1/03-new-chapter-topic', // Add your chapter here
        // ... other chapters
      ],
    },
    // ... other modules
  ],
};
```

## Development Commands

### Frontend Commands
```bash
# Start development server
npm run start

# Build for production
npm run build

# Serve production build locally
npm run serve

# Deploy to GitHub Pages
npm run deploy

# Generate static HTML
npm run build && npx docusaurus serve
```

### Content Validation
```bash
# Check for broken links
npx docusaurus write-heading-ids  # Update heading IDs if needed

# Format code
npm run format  # If configured
```

## Content Guidelines

### Writing Style
- Use clear, concise language appropriate for the target audience
- Include practical examples and hands-on exercises
- Follow the modular structure as defined in the requirements
- Ensure content is pedagogically sound

### Code Examples
- Use proper syntax highlighting
- Include comments explaining complex concepts
- Test all code examples before publishing
- Follow language-specific best practices

### Multimedia Assets
- Optimize images for web (use appropriate formats: WebP, AVIF, or JPEG/PNG)
- Include alt text for accessibility
- Store in the `static/` directory
- Reference using relative paths: `![Alt text](/img/asset-name.png)`

## Testing Your Changes

### 1. Local Testing
- Use `npm run start` to see changes in real-time
- Test navigation and content display
- Verify all links work correctly
- Check responsive design on different screen sizes

### 2. Accessibility Testing
- Ensure proper heading hierarchy (H1 → H2 → H3)
- Verify alt text for all images
- Test keyboard navigation
- Validate color contrast ratios

### 3. Performance Testing
- Check page load times
- Optimize images and assets
- Verify all interactive elements work properly

## Deployment

### GitHub Pages Deployment
The site is configured for GitHub Pages deployment:

1. Ensure your changes are committed and pushed to the main branch
2. The GitHub Actions workflow will automatically build and deploy the site
3. Check the Actions tab in GitHub for deployment status

### Manual Deployment
```bash
# From the frontend directory
GIT_USER=<Your GitHub username> CURRENT_BRANCH=main npm run deploy
```

## Troubleshooting

### Common Issues

**Docusaurus won't start**
- Ensure Node.js 18+ is installed
- Run `npm install` to install dependencies
- Clear cache: `npm run clear` or delete `node_modules` and reinstall

**Content not showing**
- Check file naming conventions
- Verify the file is added to `sidebars.js`
- Ensure proper frontmatter in the markdown file

**Build fails**
- Check for syntax errors in markdown files
- Verify all referenced assets exist
- Ensure all links point to valid resources

## Next Steps

1. Review the [full specification](./spec.md) for detailed requirements
2. Check the [implementation plan](./plan.md) for project timeline
3. Follow the [tasks list](./tasks.md) for specific implementation items
4. Refer to the [data model](./data-model.md) for content structure
5. Consult the [research document](./research.md) for technical decisions

## Getting Help

- Check the [Docusaurus documentation](https://docusaurus.io/docs)
- Review the project's GitHub Issues for known problems
- Reach out to the development team for specific questions