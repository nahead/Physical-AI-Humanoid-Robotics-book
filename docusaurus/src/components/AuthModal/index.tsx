// docusaurus/src/components/AuthModal/index.tsx
import React, { useState } from 'react';
import { useAuth } from '../../contexts/AuthContext'; // Import useAuth for supabase client
import styles from './styles.module.css';

interface Props {
  onClose: () => void;
}

const AuthModal = ({ onClose }: Props) => {
  const { supabase } = useAuth(); // Get supabase client from context
  const [isLogin, setIsLogin] = useState(true);
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [username, setUsername] = useState('');
  const [error, setError] = useState('');
  const [message, setMessage] = useState('');

  const handleAuth = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setMessage('');

    if (!supabase) {
      setError("Authentication service is not available.");
      return;
    }

    if (isLogin) {
      const { error } = await supabase.auth.signInWithPassword({ email, password });
      if (error) {
        setError(error.message);
      } else {
        setMessage('Logged in successfully!');
        onClose();
      }
    } else {
      // Registration flow
      if (!username) {
        setError('Username is required for registration.');
        return;
      }
      const { error } = await supabase.auth.signUp({
        email,
        password,
        options: {
          data: {
            username: username, // Pass username to Supabase profile
          }
        }
      });
      if (error) {
        setError(error.message);
      } else {
        setMessage('Registration successful! Please check your email to confirm.');
      }
    }
  };

  return (
    <div className={styles.modalOverlay}>
      <div className={styles.modalContent}>
        <button className={styles.closeButton} onClick={onClose}>×</button>
        <h2>{isLogin ? 'Login' : 'Sign Up'}</h2>
        {error && <p className={styles.errorText}>{error}</p>}
        {message && <p className={styles.messageText}>{message}</p>}
        <form onSubmit={handleAuth}>
          <input
            type="email"
            placeholder="Email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            required
          />
          <input
            type="password"
            placeholder="Password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            required
          />
          {!isLogin && ( // Show username input only for Sign Up
            <input
              type="text"
              placeholder="Username"
              value={username}
              onChange={(e) => setUsername(e.target.value)}
              required
            />
          )}
          <button type="submit">{isLogin ? 'Login' : 'Sign Up'}</button>
        </form>
        <button className={styles.toggleButton} onClick={() => { setIsLogin(!isLogin); setError(''); setMessage(''); }}>
          {isLogin ? 'Need an account? Sign Up' : 'Have an account? Login'}
        </button>
      </div>
    </div>
  );
};

export default AuthModal;
