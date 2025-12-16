// docusaurus/src/components/PersonalizeButton/index.js
import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

const PersonalizeButton = ({ onClick, isPersonalized, isLoading, isDisabled }) => {
  return (
    <button
      className={clsx(
        'button button--primary',
        styles.personalizeButton,
        isPersonalized && styles.active,
      )}
      onClick={onClick}
      disabled={isDisabled || isLoading}>
      {isLoading ? 'Personalizing...' : isPersonalized ? 'Revert to Original' : 'Personalize Content'}
    </button>
  );
};

export default PersonalizeButton;
