---
id: 1
title: "Create-Project-Constitution"
stage: constitution
date: 2025-12-09
model: claude-sonnet-4-5-20250929
feature: Physical AI & Humanoid Robotics Educational Book
branch: main
user: Claude
command: /sp.constitution
labels: ["constitution", "setup", "governance"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests: []
outcome: Successfully created the project constitution for the Physical AI & Humanoid Robotics Educational Book
evaluation: The constitution enforces clarity, structure, correctness, and modularity as required
---

# Create Project Constitution

## PROMPT_TEXT

/sp.constitution

The purpose of this project is to create a complete educational book titled
"Physical AI & Humanoid Robotics" using Docusaurus.
The book must follow the quarter outline, course details, modules, hardware requirements,
and weekly breakdown provided in the user instructions.

The project must be built using Claude Code and Spec-Kit Plus.

The main objectives in this stage:
1. Generate all book content in clean MDX pages.
2. Organize content into chapters and sections based on the course outline.
3. Follow Docusaurus structure inside /frontend.
4. Absolutely DO NOT build the RAG chatbot yet.
5. Absolutely DO NOT create backend logic yet.
6. Only produce content for the book and file structure.
7. Future stages will add:
   - RAG chatbot
   - User login with BetterAuth
   - Content personalization
   - Urdu translation button
   - Subagents & skills for bonus points.

This constitution must enforce clarity, structure, correctness, and modularity.

## RESPONSE_TEXT

Created the Physical AI & Humanoid Robotics Educational Book Constitution with core principles including Content-First Development, Docusaurus-Centric Architecture, Modular Content Structure, Interactive Learning Experience, Mobile-Responsive Design, and Clean Separation of Concerns. The constitution enforces the project requirements and includes technical requirements, development workflow guidelines, and governance rules.