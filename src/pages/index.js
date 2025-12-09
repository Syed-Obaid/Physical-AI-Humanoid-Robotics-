
import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

// Simple Icons Components
const Icons = {
    Box: () => (
        <svg width="24" height="24" fill="none" viewBox="0 0 24 24" stroke="currentColor">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M20 7l-8-4-8 4m16 0l-8 4m8-4v10l-8 4m0-10L4 7m8 4v10M4 7v10l8 4" />
        </svg>
    ),
    Code: () => (
        <svg width="24" height="24" fill="none" viewBox="0 0 24 24" stroke="currentColor">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M10 20l4-16m4 4l4 4-4 4M6 16l-4-4 4-4" />
        </svg>
    ),
    Globe: () => (
        <svg width="24" height="24" fill="none" viewBox="0 0 24 24" stroke="currentColor">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M3.055 11H5a2 2 0 012 2v1a2 2 0 002 2 2 2 0 012 2v2.945M8 3.935V5.5A2.5 2.5 0 0010.5 8h.5a2 2 0 012 2 2 2 0 104 0 2 2 0 012-2h1.064M15 20.488V18a2 2 0 012-2h3.064M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
        </svg>
    ),
    Clock: () => (
        <svg width="24" height="24" fill="none" viewBox="0 0 24 24" stroke="currentColor">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 8v4l3 3m6-3a9 9 0 11-18 0 9 9 0 0118 0z" />
        </svg>
    ),
    Brain: () => (
        <svg width="24" height="24" fill="none" viewBox="0 0 24 24" stroke="currentColor">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9.663 17h4.673M12 3v1m6.364 1.636l-.707.707M21 12h-1M4 12H3m3.343-5.657l-.707-.707m2.828 9.9a5 5 0 117.072 0l-.548.547A3.374 3.374 0 0014 18.469V19a2 2 0 11-4 0v-.531c0-.895-.356-1.754-.988-2.386l-.548-.547z" />
        </svg>
    ),
    Building: () => (
        <svg width="24" height="24" fill="none" viewBox="0 0 24 24" stroke="currentColor">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 21V5a2 2 0 00-2-2H7a2 2 0 00-2 2v16m14 0h2m-2 0h-5m-9 0H3m2 0h5M9 7h1m-1 4h1m4-4h1m-1 4h1m-5 10v-5a1 1 0 011-1h2a1 1 0 011 1v5m-4 0h4" />
        </svg>
    ),
    Lightning: () => (
        <svg width="24" height="24" fill="none" viewBox="0 0 24 24" stroke="currentColor">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 10V3L4 14h7v7l9-11h-7z" />
        </svg>
    ),
    Eye: () => (
        <svg width="24" height="24" fill="none" viewBox="0 0 24 24" stroke="currentColor">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M15 12a3 3 0 11-6 0 3 3 0 016 0z" />
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M2.458 12C3.732 7.943 7.523 5 12 5c4.478 0 8.268 2.943 9.542 7-1.274 4.057-5.064 7-9.542 7-4.477 0-8.268-2.943-9.542-7z" />
        </svg>
    ),
    Robot: () => (
        <svg width="24" height="24" fill="none" viewBox="0 0 24 24" stroke="currentColor">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9.75 17L9 20l-1 1h8l-1-1-.75-3M3 13h18M5 17h14a2 2 0 002-2V5a2 2 0 00-2-2H5a2 2 0 00-2 2v10a2 2 0 002 2z" />
        </svg>
    ),
    Translate: () => (
        <svg width="24" height="24" fill="none" viewBox="0 0 24 24" stroke="currentColor">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M3 5h12M9 3v2m1.048 9.5A18.022 18.022 0 016.412 9m6.088 9h7M11 21l5-10 5 10M12.751 5C11.783 10.77 8.07 15.61 3 18.129" />
        </svg>
    ),
};

function HeroSection() {
    return (
        <section className="hero-section">
            <div className="hero-badge">
                🤖 The Future of Robotics - ROS 2 & Isaac Sim - 2025 Edition
            </div>
            <h1 className="hero-title">
                Physical AI &<br />
                <span className="text-gradient-pink">Humanoid Robotics</span>
            </h1>
            <p className="hero-subtitle">
                Master the complete stack: from building the nervous system with
                ROS 2 to training digital twins in NVIDIA Isaac and deploying Vision-Language-Action models.
            </p>
            <div className="hero-buttons">
                <Link to="/docs/" className="btn-primary">
                    Start Learning
                </Link>
                <Link to="#modules" className="btn-secondary">
                    Explore Modules
                </Link>
            </div>

            <div className="hero-code-container">
                <div className="code-header">
                    <div className="window-dot dot-red"></div>
                    <div className="window-dot dot-yellow"></div>
                    <div className="window-dot dot-green"></div>
                </div>
                <pre className="code-content">
                    <span className="code-keyword">class</span> <span className="code-class">HumanoidController</span>(<span className="code-class">Node</span>):{'\n'}
                    {'    '}<span className="code-keyword">def</span> <span className="code-function">__init__</span>(<span className="code-variable">self</span>):{'\n'}
                    {'        '}<span className="code-function">super</span>().<span className="code-function">__init__</span>(<span className="code-string">'brain_node'</span>){'\n'}
                    {'        '}<span className="code-variable">self</span>.joint_pub = <span className="code-variable">self</span>.create_publisher(<span className="code-class">JointState</span>, <span className="code-string">'/cmd_vel'</span>, 10){'\n'}
                    {'        '}<span className="code-comment"># Initialize VLA Model connection...</span>
                </pre>
            </div>
        </section>
    );
}

function StatsSection() {
    return (
        <section className="stats-section">
            <div className="stats-grid">
                <div className="stat-item">
                    <div className="stat-icon"><Icons.Box /></div>
                    <div className="stat-number">4</div>
                    <div className="stat-label">Core Modules</div>
                    <div className="stat-desc">Modular Learning</div>
                </div>
                <div className="stat-item">
                    <div className="stat-icon"><Icons.Code /></div>
                    <div className="stat-number">150+</div>
                    <div className="stat-label">Technical Topics</div>
                    <div className="stat-desc">In-Depth Topics</div>
                </div>
                <div className="stat-item">
                    <div className="stat-icon"><Icons.Globe /></div>
                    <div className="stat-number">2</div>
                    <div className="stat-label">Languages</div>
                    <div className="stat-desc">Available In</div>
                </div>
                <div className="stat-item">
                    <div className="stat-icon"><Icons.Clock /></div>
                    <div className="stat-number">2025</div>
                    <div className="stat-label">Jazzy Edition</div>
                    <div className="stat-desc">Latest Release</div>
                </div>
            </div>
        </section>
    );
}

function FeaturesGrid() {
    const features = [
        {
            icon: <Icons.Brain />,
            title: 'ROS 2 Nervous System',
            desc: 'Deep dive into nodes, topics, actions, and services for robot control.'
        },
        {
            icon: <Icons.Building />,
            title: 'Digital Twins',
            desc: 'Simulate physics and environments with Gazebo Harmonics and Unity.'
        },
        {
            icon: <Icons.Lightning />,
            title: 'NVIDIA Isaac Sim',
            desc: 'Leverage GPU-accelerated simulation and Isaac Lab for training agents.'
        },
        {
            icon: <Icons.Eye />,
            title: 'VLA Models',
            desc: 'Implement Vision-Language-Action models for multimodal reasoning.'
        },
        {
            icon: <Icons.Robot />,
            title: 'Humanoid Control',
            desc: 'Master kinematics, joint constraints, and URDF for humanoid robots.'
        },
        {
            icon: <Icons.Translate />,
            title: 'Bilingual Learning',
            desc: 'Complete content available in both English and Roman Urdu.'
        },
    ];

    return (
        <section className="features-section" id="modules">
            <div className="section-header">
                <h2 className="section-title">Master Physical AI</h2>
                <p className="section-subtitle">
                    From low-level motor control to high-level reasoning with VLA models, get the complete skill set.
                </p>
            </div>
            <div className="features-grid">
                {features.map((feature, idx) => (
                    <div className="feature-card" key={idx}>
                        <div className="feature-icon-box">{feature.icon}</div>
                        <h3 className="feature-title">{feature.title}</h3>
                        <p className="feature-desc">{feature.desc}</p>
                    </div>
                ))}
            </div>
        </section>
    );
}

function CurriculumSection() {
    return (
        <section className="curriculum-section">
            <div className="curriculum-container">
                <div className="curriculum-content">
                    <h2 className="section-title">Structured for Mastery</h2>
                    <p className="section-subtitle" style={{ margin: '0 0 2rem' }}>
                        From core ROS 2 concepts to advanced Physical AI implementations. 4 Modules, 40+ Chapters.
                    </p>

                    <div className="curriculum-list">
                        {[
                            { id: '01', title: 'The Robotic Nervous System', desc: 'ROS 2, Nodes, Topics, Services, Actions, Parameters' },
                            { id: '02', title: 'The Digital Twin', desc: 'Gazebo Harmonics, Unity Integration, Physics Simulation' },
                            { id: '03', title: 'NVIDIA Isaac Platform', desc: 'Isaac Sim, Isaac Lab, GPU Acceleration, Reinforcement Learning' },
                            { id: '04', title: 'Vision-Language-Action', desc: 'Multimodal Models, VLA Architecture, Real-time Inference' },
                            { id: '05', title: 'Humanoid Control', desc: 'Kinematics, Dynamics, URDF, Whole-Body Control' },
                            { id: '06', title: 'Deployment', desc: 'Real-world Transfer, Latency Optimization, Safety' },
                        ].map(item => (
                            <div className="module-item" key={item.id}>
                                <div className="module-number">{item.id}</div>
                                <div className="module-info">
                                    <h4>{item.title}</h4>
                                    <p>{item.desc}</p>
                                </div>
                            </div>
                        ))}
                    </div>
                    {/* <div style={{ marginTop: '2rem', color: 'var(--color-accent-pink)', fontWeight: 'bold' }}>
                        + Hands-on Code Labs in Every Chapter
                    </div>
                    <div style={{ marginTop: '2rem' }}>
                        <Link to="/docs/next" className="btn-secondary">
                            View Full Table of Contents →
                        </Link>
                    </div> */}
                </div>

                <div className="book-visual">
                    <div className="book-cover">
                        <div className="book-logo-box">AI</div>
                        <div className="book-title">
                            PHYSICAL<br />AI BOOK
                        </div>
                    </div>
                </div>
            </div>
        </section>
    );
}

function CodeDemoSection() {
    return (
        <section className="physics-section">
            <h2 className="section-title">Real World Code. Real World Physics.</h2>
            <p className="section-subtitle">
                Connect AI brains to physical bodies with production-ready ROS 2 and Isaac Sim integrations.
            </p>

            <div className="terminal-mockup">
                <div className="editor-pane">
                    <div style={{ marginBottom: '1rem', display: 'flex', gap: '1rem', color: '#666' }}>
                        <span style={{ color: '#fff' }}>robot_controller.py</span>
                        <span>robot.urdf</span>
                    </div>
                    <pre style={{ margin: 0, color: '#d4d4d4' }}>
                        <span style={{ color: '#c586c0' }}>import</span> rclpy{'\n'}
                        <span style={{ color: '#c586c0' }}>from</span> rclpy.node <span style={{ color: '#c586c0' }}>import</span> Node{'\n'}
                        <span style={{ color: '#c586c0' }}>from</span> geometry_msgs.msg <span style={{ color: '#c586c0' }}>import</span> Twist{'\n'}
                        {'\n'}
                        <span style={{ color: '#569cd6' }}>class</span> <span style={{ color: '#4ec9b0' }}>RobotController</span>(<span style={{ color: '#4ec9b0' }}>Node</span>):{'\n'}
                        {'  '}<span style={{ color: '#569cd6' }}>def</span> <span style={{ color: '#dcdcaa' }}>__init__</span>(<span style={{ color: '#9cdcfe' }}>self</span>):{'\n'}
                        {'    '}<span style={{ color: '#569cd6' }}>super</span>().<span style={{ color: '#dcdcaa' }}>__init__</span>(<span style={{ color: '#ce9178' }}>'robot_controller'</span>){'\n'}
                        {'    '}<span style={{ color: '#9cdcfe' }}>self</span>.publisher_ = <span style={{ color: '#9cdcfe' }}>self</span>.create_publisher(Twist, <span style={{ color: '#ce9178' }}>'/cmd_vel'</span>, 10){'\n'}
                        {'    '}<span style={{ color: '#9cdcfe' }}>self</span>.timer = <span style={{ color: '#9cdcfe' }}>self</span>.create_timer(0.5, <span style={{ color: '#9cdcfe' }}>self</span>.timer_callback){'\n'}
                        {'    '}<span style={{ color: '#9cdcfe' }}>self</span>.get_logger().info(<span style={{ color: '#ce9178' }}>"Controller Node Started 🚀"</span>){'\n'}
                    </pre>
                </div>
                <div className="output-pane">
                    <div className="output-line">[INFO] [1715629.213]: Controller Node Started 🚀</div>
                    <div className="output-line">[INFO] [1715629.713]: Publishing Cmd: Linear: 2.0, Angular: 0.5</div>
                    <div className="output-line">[INFO] [1715630.213]: Publishing Cmd: Linear: 2.0, Angular: 0.5</div>
                    <div className="output-line">&gt; _</div>
                </div>
            </div>

            <div style={{ marginTop: '3rem' }}>
                <Link to="/docs/" className="btn-primary">
                    Build Your First Node →
                </Link>
            </div>
        </section>
    );
}

export default function Home() {
    const { siteConfig } = useDocusaurusContext();
    return (
        <Layout
            title="Physical AI & Humanoid Robotics"
            description="Master ROS 2, Isaac Sim, and VLA Models">
            <main className="landing-page">
                <HeroSection />
                <StatsSection />
                <FeaturesGrid />
                <CurriculumSection />
                <CodeDemoSection />
            </main>
        </Layout>
    );
}
