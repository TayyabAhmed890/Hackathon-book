<!-- SYNC IMPACT REPORT:
Version change: N/A (initial creation) → 1.0.0
Added sections: All sections (initial constitution)
Removed sections: None
Templates requiring updates: ✅ No existing templates to update
Follow-up TODOs: None
-->
# AI-Driven Technical Book with Embedded RAG Chatbot Constitution

## Core Principles

### Spec-First Development
Every feature and content element originates from Spec-Kit Plus specifications; All book content must be generated strictly from formal specs; No hallucinated APIs, libraries, or behaviors allowed - everything must be grounded in documented requirements.

### Technical Accuracy and Correctness
All code examples and technical explanations must be production-grade and verifiable; Every implementation detail must match the actual codebase; Explanations must be precise and technically sound for developer audiences.

### AI-Native Architecture
Leverage Claude Code for content authoring and development; Use OpenAI Agents/ChatKit SDKs for the RAG chatbot functionality; Design systems that integrate AI tools natively throughout the development lifecycle.

### Modular and Maintainable Design
Structure the book and associated systems in modular components; Each section and feature should be independently testable and maintainable; Clear separation of concerns between frontend, backend, and AI layers.

### Grounded Content Delivery
The RAG chatbot responses must be grounded in retrieved content only; No generated responses that aren't based on book content; Ensure all chatbot interactions are traceable to specific book sections.

### Production-Grade Implementation
All code must meet production standards for security, performance, and reliability; Implement proper error handling, logging, and monitoring; Follow security best practices for all components including the vector database and API endpoints.

## Architecture Standards

### Frontend Requirements
Docusaurus static site generator for book presentation; Deployed on GitHub Pages for accessibility; Responsive design optimized for technical documentation; Embedded chatbot interface integrated seamlessly with content.

### Backend Specifications
FastAPI backend for API services and RAG functionality; Proper authentication and rate limiting for chatbot endpoints; Secure connection to vector and relational databases; Comprehensive API documentation and validation.

### AI and Data Layer Standards
Qdrant Cloud vector database for RAG content storage (Free Tier); Neon Serverless Postgres for relational data; Claude Code integration for automated content generation; Proper data indexing and retrieval strategies.

## Development Workflow

### Content Creation Process
Specifications drive all content creation via Spec-Kit Plus; Content must be reviewed and validated against technical accuracy; Automated testing for code examples and functionality; Progressive content delivery aligned with development milestones.

### Quality Assurance
All code examples must be tested and verified in isolated environments; Integration tests for the RAG functionality and chatbot responses; Performance testing for the vector database queries and API endpoints; Security scanning for all dependencies and components.

### Deployment and Release
Automated deployment pipeline for GitHub Pages; Version control for all specifications and generated content; Rollback procedures for content and functionality changes; Staging environment for previewing changes.

## Governance

This constitution governs all development activities for the AI-Driven Technical Book project. All contributors must adhere to these principles and standards. Changes to this constitution require explicit approval and documentation of the rationale. Compliance with these principles is verified during code reviews and content validation processes.

**Version**: 1.0.0 | **Ratified**: 2025-12-16 | **Last Amended**: 2025-12-16