import React, { useState, useRef, useEffect } from 'react';
import { ChatMessage } from './ChatMessage';
import { ChatInput } from './ChatInput';
import './Chatbot.css';

// Define the backend URL - this can be configured based on environment
const BACKEND_URL = typeof window !== 'undefined' && window.REACT_APP_BACKEND_URL
  ? window.REACT_APP_BACKEND_URL
  : 'https://maheenalishah-rag-chatbot-backend.hf.space';

export const Chatbot = () => {
  const [messages, setMessages] = useState([]);
  const [isLoading, setIsLoading] = useState(false);
  const [isClient, setIsClient] = useState(false);
  const messagesEndRef = useRef(null);

  // Initialize on client side only
  useEffect(() => {
    setIsClient(true);

    // Initialize messages from localStorage if available
    if (typeof window !== 'undefined' && window.localStorage) {
      const savedMessages = localStorage.getItem('chatbot_messages');
      if (savedMessages) {
        setMessages(JSON.parse(savedMessages));
      }
    }
  }, []);

  // Scroll to bottom of messages
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  // Save messages to localStorage whenever they change (client-side only)
  useEffect(() => {
    if (isClient && typeof window !== 'undefined' && window.localStorage) {
      localStorage.setItem('chatbot_messages', JSON.stringify(messages));
    }
  }, [messages, isClient]);

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSendMessage = async (query, userContext = null) => {
    if (!query.trim() || isLoading || !isClient) return;

    // Add user message to chat
    const userMessage = {
      id: Date.now(),
      role: 'user',
      content: userContext ? `${query}\n\nContext: ${userContext.substring(0, 200)}${userContext.length > 200 ? '...' : ''}` : query,
      timestamp: new Date().toISOString()
    };

    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);

    try {
      // Use the general chat endpoint without filters
      const requestBody = {
        query_text: query,
        context_mode: 'retrieval_pipeline',
        filters: {}, // No filters - search entire knowledge base
        conversation_id: typeof window !== 'undefined' && window.localStorage ? localStorage.getItem('conversation_id') || null : null
      };

      const response = await fetch(`${BACKEND_URL}/api/v1/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody)
      });

      if (!response.ok) {
        throw new Error(`API request failed with status ${response.status}`);
      }

      const data = await response.json();

      // Store conversation ID in localStorage for continuity (client-side only)
      if (data.conversation_id && typeof window !== 'undefined' && window.localStorage) {
        localStorage.setItem('conversation_id', data.conversation_id);
      }

      // Create assistant message with sources
      const assistantMessage = {
        id: Date.now() + 1,
        role: 'assistant',
        content: data.response_text,
        sources: data.sources,
        confidence: data.confidence_score,
        timestamp: data.timestamp
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Error sending message:', error);

      // Add error message to chat
      const errorMessage = {
        id: Date.now() + 1,
        role: 'assistant',
        content: 'Sorry, I encountered an error while processing your request. Please try again.',
        isError: true,
        timestamp: new Date().toISOString()
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const clearConversation = () => {
    if (typeof window !== 'undefined' && window.localStorage) {
      setMessages([]);
      localStorage.removeItem('chatbot_messages');
      // Optionally also clear the conversation ID
      localStorage.removeItem('conversation_id');
    }
  };

  return (
    <div className="chatbot-container">
      <div className="chatbot-header">
        <h3>AI Textbook Assistant</h3>
      </div>

      <div className="chatbot-messages">
        {messages.length === 0 ? (
          <div className="chatbot-welcome">
            <h4>🤖 Hello! I'm your AI assistant for the AI Robotics Textbook.</h4>
            <p>Ask me any questions about robotics, AI, or related topics, and I'll provide answers based on the textbook content with proper source attribution.</p>
          </div>
        ) : (
          messages.map((message) => (
            <ChatMessage key={message.id} message={message} />
          ))
        )}
        {isLoading && <div className="loading-indicator">AI is thinking...</div>}
        <div ref={messagesEndRef} />
      </div>

      <div className="chatbot-controls">
        <button onClick={clearConversation} className="clear-conversation-btn">
          Clear Conversation
        </button>
      </div>

      <ChatInput
        onSendMessage={handleSendMessage}
        isLoading={isLoading}
      />
    </div>
  );
};