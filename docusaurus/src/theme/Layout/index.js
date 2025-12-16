import React from 'react';
import Layout from '@theme-original/Layout';
import Chatbot from '../../components/Chatbot';
import { AuthProvider } from '../../contexts/AuthContext'; // Import the AuthProvider

export default function LayoutWrapper(props) {
  return (
    <AuthProvider>
      <Layout {...props} />
      <Chatbot />
    </AuthProvider>
  );
}