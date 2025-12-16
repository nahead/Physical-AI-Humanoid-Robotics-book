# Tasks: Physical AI & Humanoid Robotics Book

**Input**: Design documents from `specs/002-ai-robotics-book/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Initialize the project structure according to the implementation plan.

- [x] T001 [P] Create root directories: `/docs`, `/src`, `/rag-backend`, `/docusaurus`
- [x] T002 Initialize a new Docusaurus project in the `/docusaurus` directory
- [x] T003 [P] Create module directories within `/docs`: `/docs/module1-ros`, `/docs/module2-simulation`, `/docs/module3-isaac`, `/docs/module4-vla`
- [x] T004 [P] Create Python project structure in `/src` with a `requirements.txt`
- [x] T005 [P] Create FastAPI project structure in `/rag-backend` with a `requirements.txt`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Configure core infrastructure that MUST be complete before content creation begins.

- [x] T006 Configure Docusaurus `docusaurus.config.js` with book title, theme, and navigation
- [x] T007 [P] Set up initial sidebar structure in a `/docusaurus/sidebars.js` file
- [x] T008 [P] Add dependencies for ROS 2 Python examples to `/src/requirements.txt`
- [x] T009 [P] Add dependencies for FastAPI, Neon Postgres, and Qdrant to `/rag-backend/requirements.txt`
- [x] T010 Set up database schema for `User` and `ChatSession` models in `rag-backend`
- [x] T011 Implement basic user authentication using `Better-Auth` in `rag-backend`

---

## Phase 3: User Story 1 - Reader Navigates Book Modules (P1) 🎯 MVP

**Goal**: Create the basic structure of the book so that a reader can see the layout and navigate between empty chapters.
**Independent Test**: The Docusaurus site builds successfully, and the sidebar shows all planned modules and chapters, which can be clicked on.

- [x] T012 [P] [US1] Create placeholder `_category_.json` and `intro.md` for `/docs/module1-ros`
- [x] T013 [P] [US1] Create placeholder `_category_.json` and `intro.md` for `/docs/module2-simulation`
- [x] T014 [P] [US1] Create placeholder `_category_.json` and `intro.md` for `/docs/module3-isaac`
- [x] T015 [P] [US1] Create placeholder `_category_.json` and `intro.md` for `/docs/module4-vla`
- [x] T016 [US1] Update `/docusaurus/sidebars.js` to auto-generate sidebars from the `/docs` directory structure
- [x] T017 [US1] Verify that the Docusaurus site builds and all placeholders are navigable
- [x] T017.1 [US1] Implement the HomepageFeatures component with module cards in `/docusaurus/src/components/HomepageFeatures/index.js`

---

## Phase 4: User Story 2 & 3 - Content Creation (P1 & P2)

**Goal**: Write the educational content, including code examples and exercises, for each module.
**Independent Test**: Each completed chapter is readable, code examples are functional, and exercises are clear.

### Module 1: ROS 2
- [x] T018 [P] [US2] Write content for Module 1, Chapter 1: Introduction to ROS 2 in `/docs/module1-ros/01-intro-to-ros.md`
- [x] T019 [P] [US2] Write content and code for Module 1, Chapter 2: ROS 2 Nodes, Topics, & Services in `/docs/module1-ros/02-nodes-topics-services.md`
- [x] T020 [P] [US2] Write content and code for Module 1, Chapter 3: URDF for Humanoids in `/docs/module1-ros/04-urdf-humanoids.md`
- [x] T020.1 [P] [US2] Write content and code for Module 1, Chapter 3: Bridging Python Agents to ROS controllers using rclpy in `/docs/module1-ros/03-python-rclpy-bridge.md`

### Module 2: Simulation
- [x] T021 [P] [US2] Write content for Module 2, Chapter 1: Introduction to Simulation in `/docs/module2-simulation/01-intro-to-simulation.md`
- [x] T022 [P] [US2] Write content and examples for Module 2, Chapter 2: Gazebo for Physics, Gravity, and Collisions in `/docs/module2-simulation/01-gazebo-physics.md`
- [x] T023 [P] [US2] Write content and examples for Module 2, Chapter 3: Unity for High-Fidelity Rendering and HRI in `/docs/module2-simulation/02-unity-rendering-hri.md`
- [x] T023.1 [P] [US2] Write content and examples for Module 2, Chapter 4: Sensor Simulation (LiDAR, Depth Cameras, IMUs) in `/docs/module2-simulation/03-sensor-simulation.md`

### Module 3: NVIDIA Isaac
- [x] T024 [P] [US2] Write content for Module 3, Chapter 1: NVIDIA Isaac Sim: Photorealistic simulation & synthetic data in `/docs/module3-isaac/01-isaac-sim-photorealistic.md`
- [x] T025 [P] [US2] Write content and code for Module 3, Chapter 2: Isaac ROS: Hardware-accelerated VSLAM & perception in `/docs/module3-isaac/02-isaac-ros-vslam-perception.md`
- [x] T026 [P] [US2] Write content and code for Module 3, Chapter 3: Nav2: Path planning for bipedal humanoid movement in `/docs/module3-isaac/03-nav2-humanoid-pathing.md`

### Module 4: VLA
- [x] T027 [P] [US2] Write content for Module 4, Chapter 1: VLA Pipelines Overview in `/docs/module4-vla/01-vla-pipelines-overview.md`
- [x] T028 [P] [US2] Write content and code for Module 4, Chapter 2: Voice-to-Action with OpenAI Whisper in `/docs/module4-vla/02-whisper-voice-to-action.md`
- [x] T029 [P] [US2] Write content and code for Module 4, Chapter 3: Capstone Project: Autonomous Humanoid in `/docs/module4-vla/03-capstone-project.md`

---

## Phase 5: Bonus Feature - RAG Chatbot

**Goal**: Implement the RAG chatbot for interactive Q&A.
**Independent Test**: The chatbot UI appears on the site and can answer questions based on the book's content.

- [ ] T030 [P] [US3] Implement content indexing endpoint (`/index-content`) in `/rag-backend/app/main.py`
- [ ] T031 [US3] Implement chat endpoint (`/chat`) in `/rag-backend/app/main.py`
- [ ] T032 [P] [US3] Create a React component for the chatbot UI in `/docusaurus/src/theme/Chatbot.js`
- [ ] T033 [US3] Integrate the Chatbot component into the Docusaurus layout

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Final review, validation, and deployment setup.

- [ ] T034 [P] Review all content for Flesch-Kincaid readability grade (8–10)
- [ ] T035 [P] Validate all internal and external links
- [ ] T036 [P] Test all code examples for reproducibility from `quickstart.md`
- [x] T037 Set up GitHub Actions workflow for deploying the Docusaurus site to GitHub Pages in `.github/workflows/deploy.yml`
- [ ] T038 Final validation of the live deployed site

---

## Dependencies & Execution Order

- **Phase 1 (Setup)** must be completed before all other phases.
- **Phase 2 (Foundational)** depends on Phase 1.
- **Phase 3 (User Story 1)** depends on Phase 2. This creates the skeleton of the book.
- **Phase 4 (Content Creation)** depends on Phase 3. The bulk of the work is here and tasks within this phase are highly parallelizable.
- **Phase 5 (Bonus Feature)** can run in parallel with Phase 4.
- **Phase 6 (Polish)** is the final phase and depends on all others.
