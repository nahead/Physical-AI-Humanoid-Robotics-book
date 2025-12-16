import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';
import Link from '@docusaurus/Link'; // Import Link component

const FeatureList = [
  {
    title: 'Module 1: The Robotic Nervous System (ROS 2)',
    // Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default, // No SVG for now
    description: (
      <>
        Learn ROS 2 nodes, topics, services, URDF, and Python agent control.
      </>
    ),
    link: '/docs/module1-ros/intro-to-ros',
  },
  {
    title: 'Module 2: The Digital Twin (Gazebo & Unity)',
    // Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default, // No SVG for now
    description: (
      <>
        Physics simulation, gravity, collisions, sensors, and Unity rendering.
      </>
    ),
    link: '/docs/module2-simulation/intro-to-simulation', // Corrected link
  },
  {
    title: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
    // Svg: require('@site/static/img/undraw_docusaurus_react.svg').default, // No SVG for now
    description: (
      <>
        Isaac Sim, VSLAM, Nav2, synthetic data, and GPU accelerated robotics.
      </>
    ),
    link: '/docs/module3-isaac/isaac-sim-photorealistic', // Corrected link
  },
  {
    title: 'Module 4: Vision-Language-Action (VLA)',
    // Svg: require('@site/static/img/undraw_docusaurus_react.svg').default, // No SVG for now
    description: (
      <>
        LLM planning, Whisper voice-to-action, and autonomous humanoid project.
      </>
    ),
    link: '/docs/module4-vla/vla-pipelines-overview', // Corrected link
  },
];

function Feature({title, description, link}) { // Removed Svg prop
  return (
    <div className={clsx('col col--3 margin-bottom--lg')}> {/* Changed col--4 to col--3, added margin */}
      <div className="card shadow--md"> {/* Added card and shadow classes */}
        <div className="card__header">
          <Heading as="h3">{title}</Heading>
        </div>
        <div className="card__body">
          <p>{description}</p>
        </div>
        <div className="card__footer">
          <Link
            className="button button--primary button--block"
            to={link}>
            Open Module →
          </Link>
        </div>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={clsx(styles.features, 'margin-top--lg')}> {/* Added margin-top */}
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}