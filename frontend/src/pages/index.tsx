import type { ReactNode } from 'react';
import Link from '@docusaurus/Link';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import HomepageFeatures from '../components/HomepageFeatures';
import styles from './index.module.css';

const TECH_STACK = ['ROS 2', 'NVIDIA Isaac Sim', 'PyTorch', 'Gazebo'];

export default function Home(): ReactNode {
  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="Master embodied intelligence and humanoid robot control with this free, open-source university-level course."
    >
      <header className={styles.hero}>
        <div className={styles.container}>
          {/* Left: Text Content */}
          <div className={styles.content}>
            <div className={styles.badge}>Free Open-Source Course</div>

            <Heading as="h1" className={styles.title}>
              Physical AI &<br />
              <span className={styles.gradientTitle}>Humanoid Robotics</span>
            </Heading>

            <p className={styles.description}>
              A complete university-level course on building intelligent humanoid robots from scratch.
              Go from simulation to real hardware using cutting-edge reinforcement learning,
              whole-body control, and sim-to-real transfer — taught through interactive notebooks,
              videos, and real robot deployments.
            </p>

            <div className={styles.stats}>
              <div className={styles.stat}>
                <strong>13 Weeks</strong>
                <span>Self-paced</span>
              </div>
              <div className={styles.stat}>
                <strong>Advanced</strong>
                <span>Graduate Level</span>
              </div>
              <div className={styles.stat}>
                <strong>8,200+</strong>
                <span>Students Enrolled</span>
              </div>
              <div className={styles.stat}>
                <strong>4.9</strong>
                <span>Course Rating</span>
              </div>
            </div>

            <div className={styles.actions}>
              <Link className={styles.enrollButton} to="/docs/overview">
                Start Learning — It's Free
              </Link>
            </div>

            <div className={styles.tech}>
              <span className={styles.techLabel}>Powered by industry-standard tools:</span>
              <div className={styles.techList}>
                {TECH_STACK.map((tech) => (
                  <span key={tech} className={styles.techItem}>{tech}</span>
                ))}
              </div>
            </div>
          </div>

          {/* Right: Beautiful Course Illustration */}
          <div className={styles.illustration}>
            <div className={styles.imagePlaceholder}>
              {/* Replace with your actual hero image */}
              <div className={styles.robotImage}>
                <span aria-label="humanoid robot">AI Humanoid Robotics</span>
                <span className={styles.robotImage2} aria-label="humanoid robot">By Muhammad Nabeel</span>
              </div>
              <div className={styles.floatingCard1}>Reinforcement Learning</div>
              <div className={styles.floatingCard2}>Whole-Body Control</div>
              <div className={styles.floatingCard3}>Sim-to-Real</div>
            </div>
          </div>
        </div>
      </header>

      <main className={styles.featuresSection}>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}