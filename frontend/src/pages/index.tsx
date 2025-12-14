import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={styles.heroBanner}>
      <div className={styles.heroContainer}>
        <div className={styles.heroContent}>
          <span className={styles.badge}>AI-Native Textbook</span>
          <Heading as="h1" className={styles.heroTitle}>
            Master Physical AI &<br />Humanoid Robotics
          </Heading>
          <p className={styles.heroSubtitle}>
            An interactive, AI-powered textbook for learning embodied intelligence.
            Build real-world robots with ROS 2, deep learning, and foundation models.
          </p>
          <div className={styles.heroButtons}>
            <Link className={styles.primaryButton} to="/docs/intro">
              Start Learning
              <span className={styles.arrow}>→</span>
            </Link>
            <Link className={styles.secondaryButton} to="/docs/quarter-overview">
              View Curriculum
            </Link>
          </div>
          <div className={styles.heroStats}>
            <div className={styles.stat}>
              <div className={styles.statNumber}>10</div>
              <div className={styles.statLabel}>Weeks</div>
            </div>
            <div className={styles.stat}>
              <div className={styles.statNumber}>40+</div>
              <div className={styles.statLabel}>Labs</div>
            </div>
            <div className={styles.stat}>
              <div className={styles.statNumber}>5</div>
              <div className={styles.statLabel}>Projects</div>
            </div>
          </div>
        </div>
        <div className={styles.heroVisual}>
          <div className={styles.floatingCard}>
            <div className={styles.cardIcon}>🤖</div>
            <div className={styles.cardTitle}>Humanoid Robotics</div>
            <div className={styles.cardDesc}>Bipedal locomotion & whole-body control</div>
          </div>
          <div className={styles.floatingCard} style={{animationDelay: '0.2s'}}>
            <div className={styles.cardIcon}>👁️</div>
            <div className={styles.cardTitle}>Computer Vision</div>
            <div className={styles.cardDesc}>YOLOv8, depth sensing, point clouds</div>
          </div>
          <div className={styles.floatingCard} style={{animationDelay: '0.4s'}}>
            <div className={styles.cardIcon}>🧠</div>
            <div className={styles.cardTitle}>Foundation Models</div>
            <div className={styles.cardDesc}>LLMs & VLMs for robot intelligence</div>
          </div>
        </div>
      </div>
    </header>
  );
}

function Features() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className={styles.featuresHeader}>
          <Heading as="h2">Why This Textbook?</Heading>
          <p>Go beyond static tutorials. Learn with AI-native interactive features.</p>
        </div>
        <div className={styles.featuresGrid}>
          <div className={styles.feature}>
            <div className={styles.featureIcon}>💬</div>
            <h3>RAG-Powered Chatbot</h3>
            <p>Highlight any text and ask questions. Get contextual explanations powered by GPT-4 and vector search.</p>
          </div>
          <div className={styles.feature}>
            <div className={styles.featureIcon}>🎯</div>
            <h3>Personalized Content</h3>
            <p>Content adapts to your skill level. Beginners get detailed explanations, experts get condensed summaries.</p>
          </div>
          <div className={styles.feature}>
            <div className={styles.featureIcon}>🌐</div>
            <h3>Multilingual Support</h3>
            <p>Translate chapters to Urdu with RTL rendering. Perfect for global learners.</p>
          </div>
          <div className={styles.feature}>
            <div className={styles.featureIcon}>⚙️</div>
            <h3>Hardware Specs Lookup</h3>
            <p>Query robotics hardware instantly with <code>/hardware Jetson Orin Nano</code></p>
          </div>
          <div className={styles.feature}>
            <div className={styles.featureIcon}>🔧</div>
            <h3>ROS 2 Code Generator</h3>
            <p>Generate ROS 2 commands and code snippets with <code>/ros2 launch lidar node</code></p>
          </div>
          <div className={styles.feature}>
            <div className={styles.featureIcon}>🎓</div>
            <h3>Hands-On Labs</h3>
            <p>40+ labs with NVIDIA Jetson, Intel RealSense cameras, and Isaac Sim simulation.</p>
          </div>
        </div>
      </div>
    </section>
  );
}

function Curriculum() {
  const modules = [
    { weeks: '1-2', title: 'ROS 2 Foundations', icon: '🚀', topics: ['Nodes & Topics', 'Services & Actions', 'Launch Files'] },
    { weeks: '3-4', title: 'Computer Vision', icon: '👁️', topics: ['YOLOv8', 'Point Clouds', 'Depth Sensing'] },
    { weeks: '5-6', title: 'Manipulation', icon: '🦾', topics: ['Kinematics', 'MoveIt 2', 'Grasping'] },
    { weeks: '7-8', title: 'Locomotion', icon: '🤖', topics: ['Bipedal Walking', 'Isaac Sim', 'PPO/SAC'] },
    { weeks: '9-10', title: 'Foundation Models', icon: '🧠', topics: ['LLMs', 'VLMs', 'Multi-modal'] },
  ];

  return (
    <section className={styles.curriculum}>
      <div className="container">
        <div className={styles.curriculumHeader}>
          <Heading as="h2">10-Week Curriculum</Heading>
          <p>From ROS 2 basics to deploying foundation models on humanoid robots</p>
        </div>
        <div className={styles.timeline}>
          {modules.map((module, idx) => (
            <div key={idx} className={styles.timelineItem}>
              <div className={styles.timelineIcon}>{module.icon}</div>
              <div className={styles.timelineContent}>
                <div className={styles.timelineWeeks}>Weeks {module.weeks}</div>
                <h3>{module.title}</h3>
                <div className={styles.timelineTopics}>
                  {module.topics.map((topic, i) => (
                    <span key={i} className={styles.topic}>{topic}</span>
                  ))}
                </div>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function CTA() {
  return (
    <section className={styles.cta}>
      <div className="container">
        <div className={styles.ctaContent}>
          <Heading as="h2">Ready to Build the Future of Robotics?</Heading>
          <p>Join thousands of students mastering Physical AI and Humanoid Robotics</p>
          <div className={styles.ctaButtons}>
            <Link className={styles.ctaPrimary} to="/docs/intro">
              Start Your Journey
            </Link>
            <Link className={styles.ctaSecondary} to="/docs/hardware-requirements">
              View Hardware Requirements
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title="AI-Native Textbook for Physical AI & Humanoid Robotics"
      description="Master Physical AI and Humanoid Robotics with an interactive, AI-powered textbook. Learn ROS 2, computer vision, motion planning, and foundation models.">
      <HomepageHeader />
      <main>
        <Features />
        <Curriculum />
        <CTA />
      </main>
    </Layout>
  );
}
