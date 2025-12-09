# Physical AI & Humanoid Robotics Educational Book - Frontend

This directory contains the Docusaurus-based frontend for the Physical AI & Humanoid Robotics Educational Book.

## Overview

This educational book covers four core modules:
- Module 1: The Robotic Nervous System (ROS 2)
- Module 2: The Digital Twin (Gazebo & Unity)
- Module 3: The AI-Robot Brain (NVIDIA Isaac™)
- Module 4: Vision-Language-Action (VLA)

## Getting Started

### Installation

1. Make sure you have Node.js 18+ installed
2. Install dependencies:
```bash
npm install
```

### Local Development

```bash
npm start
```

This command starts a local development server and opens the documentation in your browser. Most changes are reflected live without restarting the server.

### Build

```bash
npm run build
```

This command generates static content into the `build` directory and can be served using any static hosting service.

### Deployment

The site is configured for GitHub Pages deployment. The GitHub Actions workflow in `.github/workflows/deploy.yml` will automatically build and deploy the site when changes are pushed to the main branch.

## Project Structure

- `docs/` - Contains all the educational content organized by modules
- `src/` - Custom React components and theme overrides
- `static/` - Static assets like images and documents
- `docusaurus.config.js` - Main Docusaurus configuration
- `sidebars.js` - Navigation sidebar configuration
- `package.json` - Project dependencies and scripts

## Contributing

To add new content:
1. Create a new markdown file in the appropriate module directory under `docs/`
2. Add the new file to the sidebar in `sidebars.js`
3. Follow the existing content structure and formatting conventions

## Tech Stack

- [Docusaurus](https://docusaurus.io/): Static site generator
- [React](https://reactjs.org/): UI library
- [Node.js](https://nodejs.org/): JavaScript runtime
- [npm](https://www.npmjs.com/): Package manager