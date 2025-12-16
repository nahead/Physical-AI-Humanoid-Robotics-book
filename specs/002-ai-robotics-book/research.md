# Research for Physical AI & Humanoid Robotics Book

**Date**: 2025-12-05
**Feature**: [specs/002-ai-robotics-book/spec.md](specs/002-ai-robotics-book/spec.md)

## 1. Docusaurus Personalization and Translation

**Decision:** Implement personalization and translation using a custom React component within Docusaurus.

**Rationale:**
- **Personalization:** A custom React Context provider (`PersonalizationProvider`) will manage user preferences (e.g., audience type) stored in `localStorage`. This allows for dynamic content rendering based on client-side state without needing a complex backend. A `PersonalizedContent` component will conditionally render content based on these settings.
- **Translation:** Docusaurus has a robust built-in internationalization (i18n) system. The custom component will leverage this system by using the `useI18n` hook, ensuring that translations are managed consistently with the rest of the site. This is more efficient than using a third-party library.

**Alternatives Considered:**
- **Backend-driven personalization:** This would require a server to render different versions of the content. This adds significant complexity and is not well-suited to the static-first nature of Docusaurus.
- **Third-party translation services:** While services like Weblate or Crowdin could be used, they add external dependencies and complexity. Docusaurus's built-in i18n is sufficient for the project's needs.

## 2. Hardware vs. Cloud Simulation

**Decision:** The book will primarily focus on local hardware simulation, but will include a dedicated section discussing cloud options and providing guidance for a hybrid approach.

**Rationale:**
- **Target Audience:** The primary audience (students, hackathon participants) is more likely to have access to local hardware (like an RTX-capable workstation or a Jetson kit) than a budget for cloud simulation. A local-first approach is more accessible.
- **Core Concepts:** Fundamental concepts of robotics, ROS 2, and simulation can be effectively taught using local setups. Gazebo and basic Isaac Sim scenarios run well on modern local hardware.
- **Hybrid Approach:** For advanced topics like large-scale synthetic data generation or reinforcement learning, the cloud is superior. The book will explain the trade-offs and guide the reader on how to move their local projects to a cloud environment (e.g., AWS, Azure with Isaac Sim containers) when scale is needed. This provides a complete educational path.

**Alternatives Considered:**
- **Cloud-only:** This would limit the accessibility of the book for users without cloud access or a budget. It also introduces latency and connectivity as potential issues for real-time control examples.
- **Hardware-only:** This would fail to cover industry-standard practices for large-scale AI training and simulation, which is a key learning objective for aspiring robotics engineers.

## 3. Simulation Libraries: Gazebo vs. Unity vs. Isaac Sim

**Decision:** The book will teach all three, but will structure them by use case.
1.  **Gazebo:** Will be introduced first as the "workhorse" for fundamental ROS 2 integration, physics simulation, and sensor modeling. It's the standard in the ROS community and a crucial starting point.
2.  **Unity:** Will be presented for its strengths in high-fidelity rendering and creating visually appealing human-robot interaction scenarios. It will be positioned as an alternative to Gazebo for projects where visual quality is paramount.
3.  **NVIDIA Isaac Sim:** Will be the focus of the advanced "AI-Robot Brain" module. Its strengths in photorealistic synthetic data generation, sim-to-real transfer, and tight integration with NVIDIA's AI stack (Isaac ROS, TAO) make it essential for teaching modern AI-driven robotics.

**Rationale:**
- **Comprehensive Education:** Teaching all three provides a complete picture of the simulation landscape. Each tool has a distinct advantage and is used in the industry for different purposes.
- **Progressive Learning:** The structure (Gazebo -> Unity -> Isaac Sim) follows a logical progression from fundamental robotics simulation to advanced AI-driven simulation.
- **ROS 2 Integration:** All three platforms have good-to-excellent ROS 2 integration, which is a core requirement of the book.

**Alternatives Considered:**
- **Choosing only one simulator:** This would oversimplify the landscape and fail to equip the reader with a full understanding of the tools available. For example, focusing only on Gazebo would miss the AI training power of Isaac Sim, while focusing only on Isaac Sim might be too complex for a beginner's first simulation.
