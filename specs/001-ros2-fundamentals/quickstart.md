# Quickstart: Setting up the ROS2 Educational Module

## Prerequisites

Before you begin, ensure you have the following installed on your system:

- **Node.js**: Version 18.0 or higher
- **npm**: Version 8.0 or higher (or yarn/pnpm)
- **Git**: For version control

## Installation

1. **Clone the repository** (if not already done):
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Navigate to the project directory**:
   ```bash
   cd <project-root>
   ```

3. **Install Docusaurus dependencies**:
   ```bash
   npm install
   ```
   or if using yarn:
   ```bash
   yarn install
   ```

## Initial Setup

1. **Create the docs directory** (if it doesn't exist):
   ```bash
   mkdir -p docs
   ```

2. **Create the module directory**:
   ```bash
   mkdir -p docs/module-1
   ```

3. **Start the development server**:
   ```bash
   npm run start
   ```
   or if using yarn:
   ```bash
   yarn start
   ```

4. **Open your browser** to `http://localhost:3000` to view the documentation site.

## Creating the Module 1 Chapters

1. **Create Chapter 1 - ROS 2 Fundamentals**:
   ```bash
   # Create the file docs/module-1/chapter-1-ros2-fundamentals.md
   ```

2. **Create Chapter 2 - ROS 2 Communication**:
   ```bash
   # Create the file docs/module-1/chapter-2-ros2-communication.md
   ```

3. **Create Chapter 3 - Python Agents & Robot Description**:
   ```bash
   # Create the file docs/module-1/chapter-3-python-agents.md
   ```

## Configuration

1. **Configure sidebar navigation** in `sidebar.js` or `sidebars.js`:
   ```javascript
   module.exports = {
     tutorialSidebar: [
       'intro',
       {
         type: 'category',
         label: 'Module 1: The Robotic Nervous System (ROS 2)',
         items: [
           'module-1/chapter-1-ros2-fundamentals',
           'module-1/chapter-2-ros2-communication',
           'module-1/chapter-3-python-agents',
         ],
       },
     ],
   };
   ```

2. **Update the main configuration** in `docusaurus.config.js` to include your new documentation.

## Running the Development Server

To start the development server with hot reloading:

```bash
npm run start
```

To build the static site for production:

```bash
npm run build
```

To serve the built site locally for testing:

```bash
npm run serve
```

## Adding Content to Chapters

Each chapter file should follow the Docusaurus Markdown format:

```markdown
---
title: Chapter Title
sidebar_label: Chapter Label
---

# Chapter Title

## Learning Objectives

- Objective 1
- Objective 2

## Content

Your chapter content here...

## Summary

Summary of key points...
```

## Next Steps

1. Follow the detailed implementation plan in `plan.md`
2. Create the three required chapters as specified
3. Test the navigation and content
4. Deploy to GitHub Pages when ready