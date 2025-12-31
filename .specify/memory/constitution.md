<!-- SYNC IMPACT REPORT
Version change: 0.1.0 → 1.0.0
Modified principles: [PRINCIPLE_1_NAME] → AI/Spec-Driven Development, [PRINCIPLE_2_NAME] → Integrated RAG Architecture, [PRINCIPLE_3_NAME] → Deployable Architecture, [PRINCIPLE_4_NAME] → Dual-Context Chatbot, [PRINCIPLE_5_NAME] → Production-Ready Systems
Added sections: Core Principles (all), Additional Constraints, Development Workflow
Removed sections: None
Templates requiring updates:
  - .specify/templates/plan-template.md ✅ updated
  - .specify/templates/spec-template.md ✅ updated
  - .specify/templates/tasks-template.md ✅ updated
  - .specify/templates/commands/*.md ⚠ pending
Follow-up TODOs: None
-->

# AI-Driven Book with Embedded RAG Chatbot Constitution

## Core Principles

### AI/Spec-Driven Development
All development follows AI/Spec-Driven methodology using Claude Code + Spec-Kit Plus. Every feature must be specified before implementation, with clear acceptance criteria and testable outcomes. This ensures systematic, verifiable progress aligned with project goals.

### Integrated RAG Architecture
The RAG system must be tightly integrated within the Docusaurus-based book platform. The chatbot answers exclusively from book content or user-selected text, never from external sources. This maintains content integrity and ensures accurate, contextually-relevant responses.

### Deployable Architecture
The system must be deployable with book content on GitHub Pages and the chatbot in production. All components must support this deployment model with clear separation of concerns between static content and dynamic chatbot services.

### Dual-Context Chatbot
The chatbot must operate in two distinct modes: (1) answering from entire book content, and (2) answering from user-selected text only when requested. The system must clearly distinguish between these modes and respect context boundaries.

### Production-Ready Systems
All components must be production-ready with appropriate error handling, monitoring, and security measures. The technology stack (OpenAI SDKs + FastAPI + Neon Postgres + Qdrant Cloud) must be properly integrated and operational.

### Full Integration Standard
The chatbot must be fully embedded within the published book, providing seamless user experience. Integration points must be well-defined and maintainable, with clear interfaces between frontend and backend components.

## Additional Constraints

The technology stack is fixed as:
- Book platform: Docusaurus
- Chatbot backend: OpenAI SDKs + FastAPI
- Database: Neon Postgres
- Vector storage: Qdrant Cloud
- Deployment: GitHub Pages for book, production environment for chatbot

Hard constraints:
- Chatbot answers from two contexts only (entire book or user-selected text)
- All components deployed and operational
- No external data sources for chatbot responses

## Development Workflow

All development follows the Spec-Kit Plus methodology:
- Specification phase with clear requirements and acceptance criteria
- Planning phase with architectural decisions and implementation approach
- Task breakdown with testable units
- Implementation with continuous validation
- Deployment and operational readiness verification

## Governance

This constitution supersedes all other development practices. All code reviews, pull requests, and implementations must verify compliance with these principles. Any deviation requires explicit amendment to this constitution with proper documentation and approval process.

Version control follows semantic versioning with major changes requiring governance approval, minor changes for principle additions/expansions, and patches for clarifications.

**Version**: 1.0.0 | **Ratified**: 2025-12-25 | **Last Amended**: 2025-12-25