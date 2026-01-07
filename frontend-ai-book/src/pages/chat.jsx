import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import ChatWidget from '@site/src/components/ChatWidget/ChatWidget';

import Heading from '@theme/Heading';

// Simple inline styles since Root.module.css doesn't exist
const chatStyles = {
  chatBanner: {
    padding: '2rem 0',
  },
  buttons: {
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    gap: '1rem',
    marginTop: '1rem',
  },
  chatMain: {
    padding: '2rem 0',
  },
  ChatWidgetContainer: {
    margin: '2rem auto',
    maxWidth: '900px',
  }
};

function ChatPageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx("hero hero--primary", "chat-banner")} style={chatStyles.chatBanner}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          AI Chat Assistant
        </Heading>
        <p className="hero__subtitle">Ask questions about Physical AI & Humanoid Robotics</p>
        <div style={chatStyles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro"
          >
            Learn More About the Book
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function ChatPage() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`AI Assistant | ${siteConfig.title}`}
      description="Interactive AI chat for Physical AI Robotics book">
      <ChatPageHeader />
      <main style={chatStyles.chatMain}>
        <div className="container">
          <div className="row">
            <div className="col col--12">
              <div style={chatStyles.ChatWidgetContainer}>
                <ChatWidget />
              </div>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}