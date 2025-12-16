// docusaurus/src/contexts/AuthContext.tsx
import React, { createContext, useState, useEffect, useContext, ReactNode } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { getSupabaseClient } from '../lib/supabase'; // Import the new function
import { Session, User, SupabaseClient } from '@supabase/supabase-js';
import { LanguageProvider } from './LanguageContext';

interface AuthContextType {
  user: User | null;
  session: Session | null;
  loading: boolean;
  logout: () => Promise<void>;
  supabase: SupabaseClient | null; // Expose supabase client through context
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export const AuthProvider = ({ children }: { children: ReactNode }) => {
  const { siteConfig } = useDocusaurusContext();
  const { supabaseUrl, supabaseAnonKey } = siteConfig.customFields as { supabaseUrl?: string; supabaseAnonKey?: string; };

  const [supabaseClient, setSupabaseClient] = useState<SupabaseClient | null>(null);
  const [user, setUser] = useState<User | null>(null);
  const [session, setSession] = useState<Session | null>(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    if (supabaseUrl && supabaseAnonKey) {
      const client = getSupabaseClient(supabaseUrl, supabaseAnonKey);
      setSupabaseClient(client);

      const getInitialSession = async () => {
        const { data: { session } } = await client.auth.getSession();
        setSession(session);
        setUser(session?.user ?? null);
        setLoading(false);
      };

      getInitialSession();

      const { data: { subscription: authSubscription } } = client.auth.onAuthStateChange(
        (_event, session) => {
          setSession(session);
          setUser(session?.user ?? null);
          setLoading(false);
        }
      );

      return () => {
        authSubscription?.unsubscribe();
      };
    } else {
      console.error("Supabase URL or Anon Key is not available from Docusaurus customFields.");
      setLoading(false);
    }
  }, [supabaseUrl, supabaseAnonKey]);

  const value = {
    user,
    session,
    loading,
    logout: async () => {
      if (supabaseClient) {
        await supabaseClient.auth.signOut();
      }
    },
    supabase: supabaseClient,
  };

  return (
    <AuthContext.Provider value={value}>
      <LanguageProvider>
        {!loading && children}
      </LanguageProvider>
    </AuthContext.Provider>
  );
};


export const useAuth = () => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};