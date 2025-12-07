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
              <Link
                className={styles.githubButton}
                to="https://github.com/softengrmuhammadnabeel/physical-ai-humanoid-robotics-book"
              >
                <svg viewBox="0 0 24 24" className={styles.githubIcon}>
                  <path fill="currentColor" d="M12 0c-6.626 0-12 5.373-12 12 0 5.302 3.438 9.8 8.207 11.387.599.111.793-.261.793-.577v-2.234c-3.338.726-4.033-1.416-4.033-1.416-.546-1.387-1.333-1.756-1.333-1.756-1.089-.745.083-.729.083-.729 1.205.084 1.839 1.237 1.839 1.237 1.07 1.834 2.807 1.304 3.492.997.107-.775.418-1.305.762-1.604-2.665-.305-5.467-1.334-5.467-5.931 0-1.311.469-2.381 1.236-3.221-.124-.303-.535-1.523.117-3.176 0 0 1.008-.322 3.301 1.23.957-.266 1.983-.399 3.003-.404 1.02.005 2.047.138 3.006.404 2.291-1.552 3.297-1.23 3.297-1.23.653 1.653.242 2.873.118 3.176.77.84 1.235 1.911 1.235 3.221 0 4.609 tubuh-2.807 5.624-5.479 5.921.43.372.823 1.102.823 2.222v3.293c0 .319.192.694.801.576 4.765-1.589 8.199-6.086 8.199-11.386 0-6.627-5.373-12-12-12z" />
                </svg>
                View on GitHub
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