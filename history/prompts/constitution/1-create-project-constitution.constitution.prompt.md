---
id: 1
title: Create Project Constitution
stage: constitution
date: 2025-12-16
surface: agent
model: claude-haiku-4-5-20251001
feature: none
branch: master
user: user
command: /sp.constitution
labels: [constitution, setup, governance]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

```
/sp.constitution

Project: AI-Driven Technical Book with Embedded RAG Chatbot

Objective:
Create a technical book using Spec-Kit Plus and Claude Code, built with Docusaurus, deployed on GitHub Pages, and enhanced with an embedded Retrieval-Augmented Generation (RAG) chatbot.

Core Principles:
- Spec-first development (specs are the source of truth)
- Technical accuracy and correctness
- Clear, developer-focused writing
- AI-native and reproducible architecture
- Modular and maintainable design

Standards:
- Book content must be generated strictly from Spec-Kit Plus specs
- No hallucinated APIs, libraries, or behaviors
- Code must be production-grade and readable
- Explanations must match the implemented code
- Chatbot responses must be grounded in retrieved content only

Architecture Constraints:
- Frontend: Docusaurus (static site)
- Backend: FastAPI
- AI Layer: Claude Code for authoring, OpenAI Agents/ChatKit SDKs for chatbot
- Vector DB: Qdrant Cloud (Free Tier)
- Relational DB: Neon Serverless Postgres
-
```

## Response snapshot

Created the project constitution file (.specify/memory/constitution.md) with the following sections:
- Core Principles (Spec-First Development, Technical Accuracy, AI-Native Architecture, Modular Design, Grounded Content Delivery, Production-Grade Implementation)
- Architecture Standards (Frontend Requirements, Backend Specifications, AI and Data Layer Standards)
- Development Workflow (Content Creation Process, Quality Assurance, Deployment and Release)
- Governance section

## Outcome

- ✅ Impact: Created initial project constitution establishing governance and principles
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md with project-specific principles
- 🔁 Next prompts: Continue with project setup using Spec-Kit Plus
- 🧠 Reflection: Constitution properly captures all specified principles and constraints

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin spec creation for the technical book