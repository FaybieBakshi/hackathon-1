import React, { useState, useEffect, useRef } from 'react';
import clsx from 'clsx';
import styles from './ChatWidget.module.css';

const StreamingChatWidget = () => {
  // Check if running on client side (browser) to avoid SSR issues
  const [isClient, setIsClient] = useState(false);

  useEffect(() => {
    setIsClient(true);
  }, []);

  const [messages, setMessages] = useState(isClient ? [
    { id: 1, text: "Hello! I'm your AI assistant for the Physical AI Robotics book. How can I help you today?", sender: 'bot', timestamp: new Date() }
  ] : []);

  const [inputMessage, setInputMessage] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState(() => {
    // Check if running on client side (browser)
    if (typeof window !== 'undefined') {
      // Generate a unique session ID or retrieve from localStorage
      const savedSessionId = localStorage.getItem('chatbot_session_id');
      if (savedSessionId) return savedSessionId;

      const newSessionId = `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
      localStorage.setItem('chatbot_session_id', newSessionId);
      return newSessionId;
    }
    // For server-side rendering, return a placeholder
    return `session_placeholder_${Date.now()}`;
  });

  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const sendMessage = async () => {
    if (!inputMessage.trim() || isLoading || !isClient) return;

    const userMessage = {
      id: Date.now(),
      text: inputMessage.trim(),
      sender: 'user',
      timestamp: new Date()
    };

    // Add user message to chat
    setMessages(prev => [...prev, userMessage]);
    setInputMessage('');
    setIsLoading(true);

    try {
      // Call the backend API with streaming
      const response = await fetch('http://localhost:8002/chat/stream', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Accept': 'text/event-stream',
        },
        body: JSON.stringify({
          message: inputMessage.trim(),
          session_id: sessionId,
          options: {
            top_k: 5,
            temperature: 0.3
          }
        })
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      // Handle streaming response
      const reader = response.body.getReader();
      const decoder = new TextDecoder();
      let botMessage = {
        id: Date.now() + 1,
        text: '',
        sender: 'bot',
        timestamp: new Date(),
        citations: [],
        isStreaming: true
      };

      // Add initial bot message placeholder
      setMessages(prev => [...prev, botMessage]);

      let buffer = '';
      let done = false;

      while (!done) {
        const { value, done: readerDone } = await reader.read();
        done = readerDone;

        if (value) {
          const chunk = decoder.decode(value, { stream: true });
          buffer += chunk;

          // Process complete lines (SSE events are separated by \n)
          let lines = buffer.split('\n');
          buffer = lines.pop(); // Keep last incomplete line in buffer

          for (const line of lines) {
            if (line.startsWith('data: ')) {
              try {
                const data = JSON.parse(line.slice(6)); // Remove 'data: ' prefix

                if (data.event === 'message') {
                  // Update bot message content
                  botMessage.text += data.data.content;
                  setMessages(prev => {
                    const updated = [...prev];
                    const botMsgIndex = updated.findIndex(msg => msg.id === botMessage.id);
                    if (botMsgIndex !== -1) {
                      updated[botMsgIndex] = { ...botMessage };
                    }
                    return updated;
                  });
                } else if (data.event === 'citations') {
                  // Add citations to the bot message
                  botMessage.citations = data.data;
                  setMessages(prev => {
                    const updated = [...prev];
                    const botMsgIndex = updated.findIndex(msg => msg.id === botMessage.id);
                    if (botMsgIndex !== -1) {
                      updated[botMsgIndex] = { ...botMessage };
                    }
                    return updated;
                  });
                } else if (data.event === 'done') {
                  // Streaming is complete
                  botMessage.isStreaming = false;
                  setMessages(prev => {
                    const updated = [...prev];
                    const botMsgIndex = updated.findIndex(msg => msg.id === botMessage.id);
                    if (botMsgIndex !== -1) {
                      updated[botMsgIndex] = { ...botMessage };
                    }
                    return updated;
                  });
                  break;
                } else if (data.event === 'error') {
                  // Handle error
                  botMessage.text = `Error: ${data.data.message}`;
                  botMessage.isStreaming = false;
                  setMessages(prev => {
                    const updated = [...prev];
                    const botMsgIndex = updated.findIndex(msg => msg.id === botMessage.id);
                    if (botMsgIndex !== -1) {
                      updated[botMsgIndex] = { ...botMessage };
                    }
                    return updated;
                  });
                  break;
                }
              } catch (e) {
                console.error('Error parsing SSE data:', e);
              }
            }
          }
        }
      }

      reader.releaseLock();
    } catch (error) {
      console.error('Error sending message:', error);

      const errorMessage = {
        id: Date.now() + 1,
        text: "Sorry, I encountered an error processing your request. Please try again.",
        sender: 'bot',
        timestamp: new Date(),
        isError: true
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  const formatTime = (date) => {
    return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
  };

  // During SSR, return a minimal placeholder to avoid route context issues
  if (!isClient) {
    return <div className={styles.ChatWidgetContainer} style={{ visibility: 'hidden', height: 0 }} />;
  }

  return (
    <div className={styles.ChatWidgetContainer}>
      <div className={styles.chatHeader}>
        <div className={styles.headerContent}>
          <h3>AI Assistant</h3>
          <span className={styles.sessionInfo}>Session: {sessionId.substring(0, 8)}...</span>
        </div>
      </div>

      <div className={styles.chatMessages}>
        {messages.map((message) => (
          <div
            key={message.id}
            className={clsx(
              styles.message,
              styles[message.sender],
              message.isError && styles.error
            )}
          >
            <div className={styles.messageContent}>
              <p>{message.text}</p>
              {message.citations && message.citations.length > 0 && (
                <div className={styles.citations}>
                  <strong>Sources:</strong>
                  <ul>
                    {message.citations.map((citation, index) => (
                      <li key={index}>
                        <a
                          href={citation.url}
                          target="_blank"
                          rel="noopener noreferrer"
                          onClick={(e) => e.stopPropagation()}
                        >
                          {citation.title}
                        </a>
                      </li>
                    ))}
                  </ul>
                </div>
              )}
              {message.isStreaming && (
                <div className={styles.typingIndicator}>
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              )}
            </div>
            <span className={styles.timestamp}>
              {formatTime(message.timestamp)}
            </span>
          </div>
        ))}
        {isLoading && !messages.find(msg => msg.isStreaming) && (
          <div className={clsx(styles.message, styles.bot)}>
            <div className={styles.messageContent}>
              <div className={styles.typingIndicator}>
                <span></span>
                <span></span>
                <span></span>
              </div>
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      <div className={styles.chatInputForm}>
        <textarea
          ref={inputRef}
          value={inputMessage}
          onChange={(e) => setInputMessage(e.target.value)}
          onKeyPress={handleKeyPress}
          placeholder="Type your message here..."
          rows="1"
          disabled={isLoading}
          className={styles.textInput}
        />
        <button
          onClick={sendMessage}
          disabled={!inputMessage.trim() || isLoading}
          className={clsx(
            styles.sendButton,
            (!inputMessage.trim() || isLoading) && styles.sendButtonDisabled
          )}
        >
          {isLoading ? 'Sending...' : 'Send'}
        </button>
      </div>
    </div>
  );
};

export default StreamingChatWidget;