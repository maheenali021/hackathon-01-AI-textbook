import React from 'react';
import Layout from '@theme/Layout';
import { Chatbot } from '../components/Chatbot/Chatbot';

export default function ChatbotPage() {
  return (
    <Layout
      title="AI Textbook Assistant"
      description="An AI-powered chatbot to help you with questions about the AI Robotics Textbook">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--12">
            <h1>AI Textbook Assistant</h1>
            <p>Ask me any questions about robotics, AI, or related topics, and I'll provide answers based on the textbook content with proper source attribution.</p>
            <div style={{ maxWidth: '800px', margin: '20px auto' }}>
              <Chatbot />
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}