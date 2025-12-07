// components/HomepageFeatures.tsx
import type { ReactNode } from 'react';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

const modules = [
  {
    number: '01',
    title: 'ROS 2 — The Robotic Nervous System',
    description: 'Master real-time distributed systems, URDF modeling, ros2_control, and build a full 30+ DOF humanoid robot description ready for simulation.',
    weeks: 'Weeks 1–3',
    color: 'from-cyan-400 to-blue-600',
    hoverColor: 'hover:shadow-cyan-500/25',
  },
  {
    number: '02',
    title: 'Digital Twin & High-Fidelity Simulation',
    description: 'Gazebo Harmonic + Unity Robotics Hub. Simulate physics, sensors (LiDAR, RGB-D, IMU), noise, and human-robot interaction with photorealistic digital humans.',
    weeks: 'Weeks 4–6',
    color: 'from-emerald-400 to-teal-600',
    hoverColor: 'hover:shadow-emerald-500/25',
  },
  {
    number: '03',
    title: 'NVIDIA Isaac Sim — The AI Robotics Platform',
    description: 'USD scenes, synthetic data, NVBlox 3D reconstruction, cuVSLAM, Gemini perception, and bipedal Nav2 navigation in photorealistic environments.',
    weeks: 'Weeks 7–9',
    color: 'from-violet-400 to-purple-600',
    hoverColor: 'hover:shadow-violet-500/25',
  },
  {
    number: '04',
    title: 'Vision-Language-Action Models (VLA)',
    description: 'Connect GPT-4o Realtime, Whisper Live, and OpenVLA to your robot. “Bring me the red cup” → autonomous execution. Train offline models with Octo/RT-X.',
    weeks: 'Weeks 10–12',
    color: 'from-rose-400 to-pink-600',
    hoverColor: 'hover:shadow-rose-500/25',
  },
  {
    number: 'CAPSTONE',
    title: 'Deploy Your Autonomous Humanoid',
    description: 'Full-stack integration: ROS 2 + Isaac Sim + VLA → a deployable humanoid assistant. Optional real hardware deployment on Jetson Orin.',
    weeks: 'Week 13',
    color: 'from-amber-400 to-orange-600',
    hoverColor: 'hover:shadow-amber-500/25',
    isCapstone: true,
  },
];

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        {/* Header */}
        <div className={styles.header}>
          <span className={styles.badge}>13-Week Professional Curriculum</span>
          <Heading as="h2" className={styles.title}>
            The Complete Path to <span className={styles.highlight}>Physical AI</span>
          </Heading>
          <p className={styles.subtitle}>
            From ROS 2 foundations to deploying Vision-Language-Action powered humanoid robots — 
            the exact curriculum used by leading robotics labs in 2025.
          </p>
        </div>

        {/* Module Grid */}
        <div className={styles.grid}>
          {modules.map((mod) => (
            <div
              key={mod.number}
              className={styles.moduleCard}
              data-capstone={mod.isCapstone || undefined}
            >
              <div className={styles.cardHeader}>
                <span className={styles.moduleNumber}>{mod.number}</span>
                <span className={styles.weekLabel}>{mod.weeks}</span>
              </div>

              <div className={styles.cardBody}>
                <h3 className={styles.moduleTitle}>{mod.title}</h3>
                <p className={styles.moduleDesc}>{mod.description}</p>
              </div>

              {/* Gradient Overlay */}
              <div className={`${styles.gradientOverlay} ${mod.color}`} />
              <div className={`${styles.glowEffect} ${mod.hoverColor}`} />
            </div>
          ))}
        </div>

        {/* CTA */}
        <div className={styles.cta}>
          <Link to="/docs/overview" className={styles.startButton}>
            Start the 13-Week Journey
          </Link>
        </div>
      </div>
    </section>
  );
}