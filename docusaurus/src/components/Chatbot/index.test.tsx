// docusaurus/src/components/Chatbot/index.test.tsx
import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import Chatbot from './index';
import { fetchChatResponse } from './api';
import { useAuth } from '../../contexts/AuthContext';

// Mock the API call
jest.mock('./api', () => ({
  fetchChatResponse: jest.fn(),
}));

// Mock the AuthContext
jest.mock('../../contexts/AuthContext', () => ({
  useAuth: jest.fn(),
}));

const mockFetchChatResponse = fetchChatResponse as jest.Mock;
const mockUseAuth = useAuth as jest.Mock;

describe('Chatbot Component', () => {
  beforeEach(() => {
    jest.clearAllMocks();
    // Default mock for a logged-out user
    mockUseAuth.mockReturnValue({
      user: null,
      session: null,
      loading: false,
      logout: jest.fn(),
    });
    mockFetchChatResponse.mockResolvedValue({
      response_en: 'Mock chatbot response.',
      citations: [],
    });
  });

  it('renders the chat toggle button', () => {
    render(<Chatbot />);
    expect(screen.getByRole('button', { name: /chat/i })).toBeInTheDocument();
  });

  it('opens and closes the chat window when the toggle button is clicked', () => {
    render(<Chatbot />);
    const toggleButton = screen.getByRole('button', { name: /chat/i });
    
    fireEvent.click(toggleButton);
    expect(screen.getByText('AI Assistant')).toBeInTheDocument();
    
    fireEvent.click(toggleButton);
    expect(screen.queryByText('AI Assistant')).not.toBeInTheDocument();
  });

  it('allows typing in the input field', () => {
    render(<Chatbot />);
    fireEvent.click(screen.getByRole('button', { name: /chat/i }));
    const input = screen.getByPlaceholderText('Ask a question...');
    fireEvent.change(input, { target: { value: 'Hello Docusaurus!' } });
    expect(input).toHaveValue('Hello Docusaurus!');
  });

  it('sends a message and displays response', async () => {
    render(<Chatbot />);
    fireEvent.click(screen.getByRole('button', { name: /chat/i }));
    
    const input = screen.getByPlaceholderText('Ask a question...');
    const sendButton = screen.getByRole('button', { name: /send/i });

    fireEvent.change(input, { target: { value: 'What is Docusaurus?' } });
    fireEvent.click(sendButton);

    expect(screen.getByText('What is Docusaurus?')).toBeInTheDocument();
    expect(input).toHaveValue(''); // Input should be cleared

    await waitFor(() => {
      expect(mockFetchChatResponse).toHaveBeenCalledWith(
        expect.objectContaining({ query: 'What is Docusaurus?', target_language: 'en' })
      );
      expect(screen.getByText('Mock chatbot response.')).toBeInTheDocument();
    });
  });

  it('displays login button for logged out users', () => {
    render(<Chatbot />);
    fireEvent.click(screen.getByRole('button', { name: /chat/i }));
    expect(screen.getByRole('button', { name: /login/i })).toBeInTheDocument();
    expect(screen.queryByRole('button', { name: /logout/i })).not.toBeInTheDocument();
  });

  it('displays logout button for logged in users', () => {
    mockUseAuth.mockReturnValue({
      user: { id: 'user123', email: 'test@example.com' },
      session: { access_token: 'abc' },
      loading: false,
      logout: jest.fn(),
    });
    render(<Chatbot />);
    fireEvent.click(screen.getByRole('button', { name: /chat/i }));
    expect(screen.getByRole('button', { name: /logout/i })).toBeInTheDocument();
    expect(screen.queryByRole('button', { name: /login/i })).not.toBeInTheDocument();
    expect(screen.getByText(/welcome, test@example.com/i)).toBeInTheDocument();
  });
});
