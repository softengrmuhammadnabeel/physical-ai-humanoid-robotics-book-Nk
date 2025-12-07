# 🤖 Physical AI & Humanoid Robotics: The Book & RAG Platform

## 🚀 Project Overview

This project is a high-impact, **Docusaurus-based book platform** for the **Physical AI & Humanoid Robotics** course. It is integrated with a cutting-edge **Retrieval-Augmented Generation (RAG) Chatbot** and features advanced learning tools like **contextual text querying**.

The platform provides a **modern, interactive, and personalized learning experience** for students venturing from **core ROS 2 fundamentals** to deploying **autonomous, Vision-Language-Action (VLA) powered humanoid robots**.

---

## 📚 Course Content Structure

The **13-week course** is structured into **four main modules**, progressing from foundational robotics to advanced VLA models and culminating in a **capstone project**.

| Topic / Module | Key Focus Areas | Duration |
|---------------|---------------|----------|
| **Intro** | Physical AI, Humanoid Robotics, Hardware (RTX 4070 Ti+, Jetson Orin), 13-Week Roadmap | Introductory |
| **Module 1: ROS 2 — The Robotic Nervous System** | ROS 2 vs ROS 1, Nodes, Topics, Services, Actions, C++ / Python, URDF, ros2_control & Controllers | Project: Build a Full Humanoid URDF |
| **Module 2: The Digital Twin** | Gazebo & Unity Simulation, Physics Engines, LiDAR & IMU, Noise Modeling, ROS 2 ↔ Unity Bridge, HRI | Project: Walking Humanoid in Gazebo |
| **Module 3: NVIDIA Isaac Sim** | Omniverse & USD, Synthetic Data, Isaac ROS Gems (NVBlox, cuVSLAM), Nav2, ZMP, MPC, RL | Project: Nav2 Navigation in Isaac Sim |
| **Module 4: Vision-Language-Action (VLA)** | LLMs in Robotics, Whisper Live, NLP, GPT-4o, LLaMA 3, OpenVLA, Octo, RT-X | Project: Voice-Controlled Assistant |
| **Capstone Project** | Autonomous Humanoid Assistant deployable on real hardware | Culminating |

---

## ✅ Features Completed

### 1. 💻 Book Platform & Content (Docusaurus)

- Structured into **50+ markdown (.md) files**
- Organized across **4 modules + reference sections**
- Fully navigable via **Docusaurus sidebar configuration**

---

### 2. 💬 Interactive RAG Chatbot Integration (Frontend)

- **React-based Chatbot UI**
- Non-obtrusive, smooth open/close interaction
- **ThreeDotBounce typing animation**
- Components:
  - `Chatbot.tsx`
  - `Chatbot.css`
  - `ThreeDotBounce.tsx`
  - `ThreeDotBounce.css`

---

### 3. 🌐 Backend Core (FastAPI)

- **FastAPI backend** served with Uvicorn
- RAG-ready architecture
- Installed core dependencies:
  - `fastapi[standard]`
  - `openai-agents`
  - `python-dotenv`
  - `qdrant-client`
  - `uvicorn`

---

### 4. 💡 Contextual Text Querying

- Users can **select any text snippet**
- **“Ask AI”** button appears automatically
- Selected content is copied into the chatbot input
- Enables **instant, context-aware questions**

---

## ⚙️ Features in Progress

### 🔐 Better Authentication Integration

- Secure **login & signup** flow
- User session management
- Route protection for authenticated pages (e.g., Chatbot)

---

## ⏭️ Next Steps & Roadmap

| Priority | Task | Details |
|--------|------|---------|
| **HIGH** | Complete Auth (Frontend/Backend) | Finalize secure authentication flow |
| **HIGH** | Enhanced RAG Chatbot Integration | Enable real embeddings, Qdrant vector search, and contextual memory |
| **MEDIUM** | User Data Storage with Neon | Store progress, queries, and engagement data |
| **MEDIUM** | Personalization & Memory | Connect Neon data for user-specific chatbot memory |

