import React, { useState, useEffect, useCallback, useRef } from 'react';
import Layout from '@theme-original/DocItem/Layout';
import { useAuth } from '../../../contexts/AuthContext';
import Link from '@docusaurus/Link';
import { useLanguage } from '../../../contexts/LanguageContext';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import PersonalizeButton from '../../../components/PersonalizeButton'; // Import PersonalizeButton
import { fetchTranslation, fetchPersonalizedContent } from '../../../components/Chatbot/api'; // Import fetchPersonalizedContent
import styles from './styles.module.css';

export default function LayoutWrapper(props) {
  const { user, loading: authLoading, userProfile, supabase } = useAuth(); // Get userProfile and supabase
  const { language, toggleLanguage } = useLanguage();
  const { siteConfig } = useDocusaurusContext();
  const { apiBaseUrl } = siteConfig.customFields;

  const [translatedContent, setTranslatedContent] = useState(null);
  const [isTranslating, setIsTranslating] = useState(false);
  const contentRef = useRef(null);

  const [isPersonalized, setIsPersonalized] = useState(false);
  const [personalizingContent, setPersonalizingContent] = useState(null); // Stores personalized content
  const [isPersonalizing, setIsPersonalizing] = useState(false);
  const [personalizationError, setPersonalizationError] = useState(''); // State for personalization errors

  // Helper function to get page content from DOM
  const getPageContent = useCallback(() => {
    if (!contentRef.current) return null;

    // Try multiple selectors to find the content
    const selectors = [
      '[class*="mdxPageWrapper"]',
      '[class*="docItemContainer"] [class*="markdown"]',
      '[class*="docItemCol"] [class*="markdown"]',
      'article [class*="markdown"]',
      '[class*="markdown"]',
      'article',
      'main'
    ];

    for (const selector of selectors) {
      const el = contentRef.current.querySelector(selector);
      if (el && el.innerText && el.innerText.trim().length > 50) {
        console.log('Found content using selector:', selector);
        return el.innerText;
      }
    }

    // Fallback: get all text from contentRef
    const allText = contentRef.current.innerText;
    if (allText && allText.trim().length > 50) {
      console.log('Using fallback: contentRef.innerText');
      return allText;
    }

    return null;
  }, []);

  // Effect for translation - triggered by button click
  const handleTranslate = useCallback(async () => {
    if (language === 'ur') {
      // Switch back to English
      toggleLanguage();
      setTranslatedContent(null);
      return;
    }

    const textToTranslate = getPageContent();

    if (!textToTranslate) {
      console.error("No content found to translate. Available elements:", contentRef.current?.innerHTML?.substring(0, 500));
      alert("Could not find page content to translate. Please try refreshing the page.");
      return;
    }

    console.log("Translating content length:", textToTranslate.length);
    setIsTranslating(true);
    toggleLanguage(); // Switch to Urdu

    try {
      const { translated_text } = await fetchTranslation(
        { text: textToTranslate, target_language: 'ur' },
        apiBaseUrl || ''
      );
      setTranslatedContent(translated_text);
    } catch (e) {
      console.error("Error during translation:", e);
      setTranslatedContent("Translation failed. Please try again later.");
    } finally {
      setIsTranslating(false);
    }
  }, [language, apiBaseUrl, toggleLanguage, getPageContent]);

  // Clear translation when switching to English
  useEffect(() => {
    if (language === 'en') {
      setTranslatedContent(null);
    }
  }, [language]);

  // Handle personalization click
  const handlePersonalize = useCallback(async () => {
    if (!user || !supabase || !userProfile) { // Ensure user is logged in and profile loaded
      setPersonalizationError("You must be logged in with a complete profile to personalize content.");
      return;
    }

    if (isPersonalized) {
      // Revert to original content
      setIsPersonalized(false);
      setPersonalizingContent(null);
      setPersonalizationError('');
      return;
    }

    setIsPersonalizing(true);
    setPersonalizingContent(null); // Clear previous personalized content
    setPersonalizationError(''); // Clear any previous errors

    try {
      // Get text from DOM using helper
      const originalText = getPageContent();

      if (!originalText) {
        throw new Error("Could not extract content for personalization.");
      }

      // Fetch latest user profile to ensure correct background levels
      const { data: profile, error: profileError } = await supabase
        .from('profiles')
        .select('software_background, hardware_background')
        .eq('id', user.id)
        .single();

      if (profileError || !profile) {
        throw new Error("Failed to retrieve user background for personalization.");
      }

      const { personalizedContent: newPersonalizedContent } = await fetchPersonalizedContent(
        {
          originalContent: originalText,
          softwareBackground: profile.software_background || 'beginner', // Default if not set
          hardwareBackground: profile.hardware_background || 'beginner', // Default if not set
        },
        apiBaseUrl || ''
      );

      setPersonalizingContent(newPersonalizedContent);
      setIsPersonalized(true); // Mark as personalized
    } catch (e) {
      console.error("Error during personalization:", e);
      // Display error to user - perhaps in a temporary state
      setPersonalizationError(`Personalization failed: ${e instanceof Error ? e.message : String(e)}`);
      setIsPersonalized(false); // Ensure button state is not personalized
    } finally {
      setIsPersonalizing(false);
    }
  }, [user, supabase, userProfile, apiBaseUrl, isPersonalized, getPageContent]);

  // Show loading state while determining auth status
  if (authLoading) {
    return (
      <Layout>
        <div className={styles.loadingContainer}>Loading authentication...</div>
      </Layout>
    );
  }

  // If user is not logged in, show a login prompt
  if (!user) {
    return (
      <Layout>
        <div className={styles.loginPromptContainer}>
          <h1>Access Denied</h1>
          <p>You must be logged in to view this content.</p>
          <Link to="/login" className="button button--primary">Go to Login Page</Link>
        </div>
      </Layout>
    );
  }

  // Determine content to render: personalized, translated, or original
  let contentToRender;
  if (isPersonalizing) {
    contentToRender = <div className={styles.loadingContainer}>Personalizing content...</div>;
  } else if (isPersonalized && personalizingContent) {
    contentToRender = <div className="markdown" dangerouslySetInnerHTML={{ __html: personalizingContent }} />;
  } else if (isTranslating) {
    contentToRender = <div className={styles.loadingContainer}>Translating content...</div>;
  } else if (translatedContent) {
    contentToRender = <div className="markdown" dangerouslySetInnerHTML={{ __html: translatedContent }} />;
  } else {
    contentToRender = props.children;
  }
  
  // Conditionally render error message within the content if personalization failed
  if (personalizationError) {
    contentToRender = (
      <>
        <div className={styles.errorMessage}>{personalizationError}</div>
        {contentToRender} {/* Fallback to whatever was supposed to render */}
      </>
    );
  }


  return (
    <div className={styles.docPageContainer} ref={contentRef}>
      <div className={styles.personalizationControls}>
        {userProfile && ( // Only show personalize button if profile is loaded
          <PersonalizeButton
            onClick={handlePersonalize}
            isPersonalized={isPersonalized}
            isLoading={isPersonalizing}
            isDisabled={isTranslating || !userProfile.software_background || !userProfile.hardware_background}
          />
        )}
        <div className={styles.languageToggleContainer}>
          <button
            className={styles.translateButton}
            onClick={handleTranslate}
            disabled={isTranslating}
          >
            {isTranslating ? 'Translating...' : (language === 'ur' ? '🇬🇧 English' : '🇵🇰 اردو')}
          </button>
        </div>
      </div>
      <Layout {...props}>
        {contentToRender}
      </Layout>
    </div>
  );
}
