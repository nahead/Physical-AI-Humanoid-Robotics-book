// docusaurus/src/components/Chatbot/index.tsx
import React, { useState, useRef, useEffect, useCallback } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './styles.module.css';
import { fetchChatResponse, ChatRequest } from './api'; // Keep ChatRequest, remove Language
import { useAuth } from '../../contexts/AuthContext';
import AuthModal from '../AuthModal';
import { supabase } from '../../lib/supabase';
import { useLanguage } from '../../contexts/LanguageContext'; // Corrected import placement

interface Message {
  sender: 'user' | 'chatbot';
  text: string;
  citations?: { source: string; url?: string }[];
}

const Chatbot = () => {
  const { siteConfig } = useDocusaurusContext();
  const { apiBaseUrl } = siteConfig.customFields as { apiBaseUrl?: string };

  const { user, logout, supabase } = useAuth(); // Destructure supabase from useAuth
  const { language, toggleLanguage } = useLanguage(); // Use shared language context
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [showAuthModal, setShowAuthModal] = useState(false); // Re-add state for auth modal
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Add state for profile data
  const [userProfile, setUserProfile] = useState<{ full_name?: string; software_background?: string; hardware_background?: string; } | null>(null);


  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Fetch user profile on login/refresh
  useEffect(() => {
    const fetchUserProfile = async () => {
      if (user && supabase) { // Ensure supabase client is available
        const { data, error } = await supabase
          .from('profiles')
          .select('full_name, preferred_language, software_background, hardware_background')
          .eq('id', user.id)
          .single();

        if (error) {
          console.error('Error fetching user profile:', error);
          setUserProfile(null);
        } else if (data) {
          setUserProfile(data);
          if (data.preferred_language && data.preferred_language !== language) {
            // This needs to update the global language context
            // You might want a setLanguage function from useLanguage here
            // For now, it's just logging, but this would be a future improvement
            // setLanguage(data.preferred_language);
          }
        }
      } else {
        setUserProfile(null);
        // Reset language if no user is logged in (handled by LanguageContext default)
      }
    };
    fetchUserProfile();
  }, [user, supabase, language]); // Added language to dependency array to avoid stale closure

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const handleInputChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setInputValue(event.target.value);
  };

  const handleLogout = async () => {
    await logout();
    setMessages([]); // Clear messages on logout
    setUserProfile(null); // Clear profile data
  };

  const handleSendMessage = async (event: React.FormEvent) => {
    event.preventDefault();
    if (inputValue.trim() === '' || isLoading) return;

    const userMessage: Message = { sender: 'user', text: inputValue };
    setMessages(prev => [...prev, userMessage]);
    const currentInput = inputValue;
    setInputValue('');
    setIsLoading(true);

    try {
      const apiRequest: ChatRequest = {
        query: currentInput,
        target_language: language,
      };
      
      const data = await fetchChatResponse(apiRequest, apiBaseUrl || '');

      const botMessage: Message = {
        sender: 'chatbot',
        text: data.response_translated || data.response_en,
        citations: data.citations,
      };
      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      const errorMessage: Message = {
        sender: 'chatbot',
        text: 'Sorry, something went wrong. Please try again later.',
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <>
      <button className={styles.chatToggleButton} onClick={toggleChat}>
        {isOpen ? 'Close' : 'Chat'}
      </button>
      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <h2>AI Assistant</h2>
            {user && <span className={styles.loggedInUser}>Welcome, {userProfile?.full_name || user.email}</span>}
            <div className={styles.langToggle} onClick={toggleLanguage}>
              <span className={language === 'en' ? styles.activeLang : ''}>EN</span>
              <span className={language === 'ur' ? styles.activeLang : ''}>UR</span>
            </div>
            {user ? (
              <button className={styles.authButton} onClick={handleLogout}>Logout</button>
            ) : (
              <button className={styles.authButton} onClick={() => setShowAuthModal(true)}>Login</button>
            )}
            <button onClick={toggleChat} className={styles.closeButton}>×</button>
          </div>
          <div className={styles.chatMessages}>
            {messages.map((message, index) => (
              <div key={index} className={`${styles.message} ${styles[message.sender]}`}>
                <p>{message.text}</p>
                {message.citations && message.citations.length > 0 && (
                  <div className={styles.citations}>
                    <strong>Sources:</strong>
                    <ul>
                      {message.citations.map((citation, idx) => (
                        <li key={idx}>
                          <a href={citation.url} target="_blank" rel="noopener noreferrer">
                            {citation.source}
                          </a>
                        </li>
                      ))}
                    </ul>
                  </div>
                )}
              </div>
            ))}
            {isLoading && <div className={`${styles.message} ${styles.chatbot} ${styles.loading}`}><span></span><span></span><span></span></div>}
            <div ref={messagesEndRef} />
          </div>
          <form className={styles.chatInputForm} onSubmit={handleSendMessage}>
            <input
              type="text"
              value={inputValue}
              onChange={handleInputChange}
              placeholder="Ask a question..."
              disabled={isLoading}
            />
            <button type="submit" disabled={isLoading}>Send</button>
          </form>
        </div>
      )}
      {showAuthModal && <AuthModal onClose={() => setShowAuthModal(false)} />}
    </>
  );
};

export default Chatbot;