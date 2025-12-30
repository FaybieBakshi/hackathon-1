---
id: 1
title: Create Project Constitution
stage: constitution
date: 2025-12-25
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: main
user: user
command: /sp.constitution
labels: [constitution,spec-driven,ai-book,rag-chatbot]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
---

## Prompt

```
/sp.constitution

Project: AI-Driven Book with Embedded RAG Chatbot

Core principles:
• AI/Spec-Driven: Use Claude Code + Spec-Kit Plus
• Integrated RAG: Chatbot must answer from full book or user-selected text only
• Deployable: Book on GitHub Pages; chatbot in production

Key standards:
• Book: Built with Docusaurus
• Chatbot Stack: OpenAI SDKs + FastAPI + Neon Postgres + Qdrant Cloud
• Full integration within published book

Hard constraints:
• Chatbot answers from two contexts:
  1. Entire book content
  2. User-selected text only (when requested)
• All components deployed and operational

Success criteria:
• Book live on GitHub Pages
• Embedded chatbot fully functional
• Chatbot correctly respects both query modes
• Complete, working production system
```

## Response snapshot

Created the project constitution file at .specify/memory/constitution.md with the following principles:
1. AI/Spec-Driven Development
2. Integrated RAG Architecture
3. Deployable Architecture
4. Dual-Context Chatbot
5. Production-Ready Systems
6. Full Integration Standard

The constitution includes additional constraints, development workflow, and governance sections aligned with the project requirements.

## Outcome

- ✅ Impact: Created foundational project constitution with 6 core principles and governance structure
- 🧪 Tests: No tests required for this task
- 📁 Files: Created .specify/memory/constitution.md with project principles
- 🔁 Next prompts: Plan and specify the AI-driven book features
- 🧠 Reflection: Constitution provides clear governance framework for the AI-Driven Book with Embedded RAG Chatbot project

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Not applicable
- Prompt variant (if applicable): Not applicable
- Next experiment (smallest change to try): Begin feature specification based on constitution