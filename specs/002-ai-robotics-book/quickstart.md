# Quickstart Guide

**Date**: 2025-12-05
**Feature**: [specs/002-ai-robotics-book/spec.md](specs/002-ai-robotics-book/spec.md)

This guide provides instructions on how to set up the development environment for the "Physical AI & Humanoid Robotics Book" project.

## Prerequisites

- **Node.js**: Version 20.x or later.
- **Python**: Version 3.11.x or later.
- **Git**: For version control.
- **ROS 2 Humble**: Follow the official installation guide for your operating system.
- **NVIDIA GPU**: An RTX-capable GPU is required for Isaac Sim.
- **NVIDIA Isaac Sim**: Follow the official installation guide.
- **Docker**: For running some services if not installed locally.

## 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-directory>
```

## 2. Docusaurus Site Setup

The core of the book is a Docusaurus website.

```bash
# Navigate to the docusaurus project directory
cd docusaurus

# Install dependencies
npm install

# Run the development server
npm start
```

The website will be available at `http://localhost:3000`.

## 3. ROS 2 Environment Setup

Ensure your ROS 2 environment is sourced correctly.

```bash
# On Linux
source /opt/ros/humble/setup.bash
```

To run the Python code examples for ROS 2, you will need to install the necessary dependencies and build the packages.

```bash
# Navigate to the source directory for Python code
cd src

# (Instructions for building ROS 2 packages will be provided here)
```

## 4. RAG Chatbot Backend (Optional)

The RAG chatbot is an optional bonus feature. To run it locally:

```bash
# Navigate to the backend directory
cd rag-backend

# Create a Python virtual environment
python -m venv venv
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Run the FastAPI server
uvicorn app.main:app --reload
```

You will also need to have instances of Neon Postgres and Qdrant running, either locally via Docker or using their cloud services. Configuration will be managed via a `.env` file.

## 5. Running Simulations

- **Gazebo**: Launch Gazebo simulations via ROS 2 launch files.
- **NVIDIA Isaac Sim**: Launch Isaac Sim and load the appropriate scene or robot model.

Detailed instructions for each simulation will be provided within the relevant chapters of the book.
