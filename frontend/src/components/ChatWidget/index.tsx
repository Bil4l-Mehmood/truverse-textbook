import React, { useState, useEffect, useRef } from 'react';
import { sendChatMessage, ChatMessage, SearchResult } from '../../services/api';
import { lookupHardware, generateROS2Command, HardwareSpec, ROS2Command } from '../../services/skillsService';
import HardwareSpecCard from '../Skills/HardwareSpecCard';
import ROS2CommandCard from '../Skills/ROS2CommandCard';
import './styles.css';

interface ExtendedMessage extends ChatMessage {
  skillType?: 'hardware' | 'ros2';
  skillData?: HardwareSpec | ROS2Command;
}

const ChatWidget: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<ExtendedMessage[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState<string | undefined>();
  const [selectedText, setSelectedText] = useState<string>('');
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Handle text selection (only on client-side)
  useEffect(() => {
    // Only set up event listener on client-side
    if (typeof window === 'undefined') {
      return;
    }

    const handleTextSelection = () => {
      const selected = window.getSelection()?.toString().trim();
      if (selected && selected.length > 0) {
        setSelectedText(selected);
        console.log('[ChatWidget] Text selected:', selected);
        // Optionally auto-open chat with hint
      }
    };

    document.addEventListener('mouseup', handleTextSelection);
    return () => document.removeEventListener('mouseup', handleTextSelection);
  }, []);

  const handleSendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const trimmedInput = inputValue.trim();
    const userMessage: ExtendedMessage = {
      role: 'user',
      content: trimmedInput,
      timestamp: new Date(),
    };

    setMessages((prev) => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Check for hardware skill command: /hardware <component>
      if (trimmedInput.startsWith('/hardware ')) {
        const component = trimmedInput.replace('/hardware ', '').trim();
        if (!component) {
          throw new Error('Please specify a component name. Usage: /hardware <component>');
        }

        const hardwareData = await lookupHardware(component);
        const assistantMessage: ExtendedMessage = {
          role: 'assistant',
          content: `Hardware specifications for ${component}:`,
          timestamp: new Date(),
          skillType: 'hardware',
          skillData: hardwareData,
        };
        setMessages((prev) => [...prev, assistantMessage]);
        return;
      }

      // Check for ROS2 skill command: /ros2 <task>
      if (trimmedInput.startsWith('/ros2 ')) {
        const task = trimmedInput.replace('/ros2 ', '').trim();
        if (!task) {
          throw new Error('Please specify a task. Usage: /ros2 <task>');
        }

        const ros2Data = await generateROS2Command(task);
        const assistantMessage: ExtendedMessage = {
          role: 'assistant',
          content: `ROS 2 command generation:`,
          timestamp: new Date(),
          skillType: 'ros2',
          skillData: ros2Data,
        };
        setMessages((prev) => [...prev, assistantMessage]);
        return;
      }

      // Fall back to RAG chat for regular queries
      const response = await sendChatMessage(trimmedInput, sessionId);

      if (!sessionId) {
        setSessionId(response.session_id);
      }

      const assistantMessage: ExtendedMessage = {
        role: 'assistant',
        content: response.answer,
        timestamp: new Date(),
        sources: response.sources,
      };

      setMessages((prev) => [...prev, assistantMessage]);
    } catch (error) {
      const errorMessage: ExtendedMessage = {
        role: 'assistant',
        content: `Sorry, I encountered an error: ${error instanceof Error ? error.message : 'Unknown error'}. Make sure the backend is running at http://localhost:8000`,
        timestamp: new Date(),
      };
      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  return (
    <>
      {/* Chat Button */}
      <button
        className="chat-button"
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Toggle chat"
      >
        {isOpen ? (
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor">
            <line x1="18" y1="6" x2="6" y2="18"></line>
            <line x1="6" y1="6" x2="18" y2="18"></line>
          </svg>
        ) : (
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor">
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
          </svg>
        )}
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className="chat-window">
          <div className="chat-header">
            <div>
              <h3>AI Textbook Assistant</h3>
              <p>Ask questions about the course content</p>
            </div>
            <button onClick={() => setIsOpen(false)} className="close-button">
              ×
            </button>
          </div>

          <div className="chat-messages">
            {messages.length === 0 && (
              <div className="welcome-message">
                <h4>👋 Hello! I'm your AI tutor</h4>
                <p>Ask me anything about Physical AI & Humanoid Robotics:</p>
                <ul>
                  <li>Hardware requirements</li>
                  <li>ROS 2 concepts</li>
                  <li>Computer vision topics</li>
                  <li>Humanoid robotics</li>
                </ul>
                <div className="skill-shortcuts">
                  <p><strong>💡 Pro Tips:</strong></p>
                  <p>• Use <code>/hardware nvidia-jetson-orin-nano</code> to lookup hardware specs</p>
                  <p>• Use <code>/ros2 launch lidar</code> to generate ROS 2 commands</p>
                </div>
              </div>
            )}

            {messages.map((message, index) => (
              <div key={index} className={`message ${message.role}`}>
                <div className="message-content">
                  {message.skillType === 'hardware' && message.skillData ? (
                    <HardwareSpecCard spec={message.skillData as HardwareSpec} />
                  ) : message.skillType === 'ros2' && message.skillData ? (
                    <ROS2CommandCard command={message.skillData as ROS2Command} />
                  ) : (
                    <>
                      <div className="message-text">{message.content}</div>
                      {message.sources && message.sources.length > 0 && (
                        <div className="message-sources">
                          <strong>Sources:</strong>
                          {message.sources.map((source, idx) => (
                            <div key={idx} className="source-item">
                              <span className="source-title">{source.title}</span>
                              <span className="source-score">
                                {(source.score * 100).toFixed(0)}% match
                              </span>
                            </div>
                          ))}
                        </div>
                      )}
                    </>
                  )}
                </div>
              </div>
            ))}

            {isLoading && (
              <div className="message assistant">
                <div className="message-content">
                  <div className="typing-indicator">
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          <div className="chat-input-container">
            {selectedText && (
              <div className="selected-text-hint">
                <small>
                  📌 Selected: "{selectedText.substring(0, 50)}
                  {selectedText.length > 50 ? '...' : ''}"
                </small>
              </div>
            )}
            <textarea
              className="chat-input"
              placeholder={selectedText ? `Ask about the selected text...` : 'Ask a question...'}
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              rows={1}
              disabled={isLoading}
            />
            <button
              className="send-button"
              onClick={handleSendMessage}
              disabled={isLoading || !inputValue.trim()}
              title={selectedText ? 'Ask about the selected text' : 'Send message'}
            >
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor">
                <line x1="22" y1="2" x2="11" y2="13"></line>
                <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
              </svg>
            </button>
          </div>
        </div>
      )}
    </>
  );
};

export default ChatWidget;
