// docusaurus/src/pages/login.js
import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { AuthProvider, useAuth } from '../contexts/AuthContext'; // Import AuthProvider
import styles from './login.module.css';

// Create a new component for the form logic
const LoginForm = () => {
  const { user, supabase } = useAuth();
  const [isLogin, setIsLogin] = useState(true);
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [fullName, setFullName] = useState('');
  const [softwareBackground, setSoftwareBackground] = useState('beginner');
  const [hardwareBackground, setHardwareBackground] = useState('beginner');
  const [error, setError] = useState('');
  const [message, setMessage] = useState('');

  const handleAuth = async (e) => {
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
      }
    } else {
      if (!fullName) {
        setError('Full name is required for registration.');
        return;
      }
      const { error } = await supabase.auth.signUp({
        email,
        password,
        options: {
          data: {
            full_name: fullName,
            software_background: softwareBackground,
            hardware_background: hardwareBackground,
          },
        },
      });
      if (error) {
        setError(error.message);
      } else {
        setMessage('Registration successful! Please check your email to confirm.');
      }
    }
  };

  if (user) {
    return (
      <div className={styles.loginContainer}>
        <h1>You are already logged in as {user.email}</h1>
      </div>
    );
  }

  return (
    <div className={styles.loginContainer}>
      <div className={styles.authForm}>
        <h2>{isLogin ? 'Login' : 'Sign Up'}</h2>
        {error && <p className={styles.errorText}>{error}</p>}
        {message && <p className={styles.messageText}>{message}</p>}
        <form onSubmit={handleAuth}>
          {!isLogin && (
            <>
              <input
                type="text"
                placeholder="Full Name"
                value={fullName}
                onChange={(e) => setFullName(e.target.value)}
                required
              />
              <select value={softwareBackground} onChange={(e) => setSoftwareBackground(e.target.value)}>
                <option value="beginner">Software: Beginner</option>
                <option value="intermediate">Software: Intermediate</option>
                <option value="advanced">Software: Advanced</option>
              </select>
              <select value={hardwareBackground} onChange={(e) => setHardwareBackground(e.target.value)}>
                <option value="beginner">Hardware: Beginner</option>
                <option value="intermediate">Hardware: Intermediate</option>
                <option value="advanced">Hardware: Advanced</option>
              </select>
            </>
          )}
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
          <button type="submit">{isLogin ? 'Login' : 'Sign Up'}</button>
        </form>
        <button className={styles.toggleButton} onClick={() => { setIsLogin(!isLogin); setError(''); setMessage(''); }}>
          {isLogin ? 'Need an account? Sign Up' : 'Have an account? Login'}
        </button>
      </div>
    </div>
  );
};

// The default export for the page wraps the logic component in the provider
const LoginPage = () => {
  return (
    <Layout title="Login">
      <AuthProvider>
        <LoginForm />
      </AuthProvider>
    </Layout>
  );
};

export default LoginPage;

