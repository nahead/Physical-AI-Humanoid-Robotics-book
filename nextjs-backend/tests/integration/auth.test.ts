// nextjs-backend/tests/integration/auth.test.ts
import request from 'supertest';
import { NextApiRequest, NextApiResponse } from 'next';
import registerHandler from '../../pages/api/auth/register';
import loginHandler from '../../pages/api/auth/login';
import logoutHandler from '../../pages/api/auth/logout';

// Mock the Supabase client
jest.mock('../../lib/supabase', () => ({
  supabase: {
    auth: {
      signUp: jest.fn(),
      signInWithPassword: jest.fn(),
      signOut: jest.fn(),
    },
    from: jest.fn(() => ({
        select: jest.fn(() => ({
            eq: jest.fn(() => ({
                single: jest.fn(),
            })),
        })),
    })),
  },
}));

import { supabase } from '../../lib/supabase';
const mockSupabaseAuth = supabase.auth as jest.Mocked<typeof supabase.auth>;

describe('Authentication API Endpoints', () => {
  let mockReq: Partial<NextApiRequest>;
  let mockRes: Partial<NextApiResponse>;

  beforeEach(() => {
    jest.clearAllMocks();
    mockReq = {
      method: 'POST',
      headers: {
        'content-type': 'application/json',
      },
    };
    mockRes = {
      status: jest.fn().mockReturnThis(),
      json: jest.fn().mockReturnThis(),
      setHeader: jest.fn().mockReturnThis(),
    };
  });

  // --- Register Endpoint Tests ---
  describe('POST /api/auth/register', () => {
    it('should register a new user successfully', async () => {
      mockReq.body = { email: 'test@example.com', password: 'password123', username: 'testuser' };
      mockSupabaseAuth.signUp.mockResolvedValueOnce({
        data: {
          user: { id: 'some-uuid', email: 'test@example.com' } as any,
          session: null,
        },
        error: null,
      });

      await registerHandler(mockReq as NextApiRequest, mockRes as NextApiResponse);

      expect(mockSupabaseAuth.signUp).toHaveBeenCalledWith({
        email: 'test@example.com',
        password: 'password123',
        options: { data: { username: 'testuser' } }
      });
      expect(mockRes.status).toHaveBeenCalledWith(200);
      expect(mockRes.json).toHaveBeenCalledWith({
        message: 'Registration successful. Please check your email to confirm your account.',
        user: { id: 'some-uuid', email: 'test@example.com' },
      });
    });

    it('should return 400 if email or password is missing', async () => {
      mockReq.body = { email: 'test@example.com' }; // missing password
      await registerHandler(mockReq as NextApiRequest, mockRes as NextApiResponse);
      expect(mockRes.status).toHaveBeenCalledWith(400);
      expect(mockRes.json).toHaveBeenCalledWith({ error: 'Email and password are required.' });
    });

    it('should return 400 if username is missing', async () => {
        mockReq.body = { email: 'test@example.com', password: 'password123' }; // missing username
        await registerHandler(mockReq as NextApiRequest, mockRes as NextApiResponse);
        expect(mockRes.status).toHaveBeenCalledWith(400);
        expect(mockRes.json).toHaveBeenCalledWith({ error: 'Username is required.' });
    });

    it('should return 409 for existing user', async () => {
      mockReq.body = { email: 'existing@example.com', password: 'password123', username: 'existinguser' };
      mockSupabaseAuth.signUp.mockResolvedValueOnce({
        data: { user: null, session: null },
        error: { message: 'User already registered', status: 400 } as any,
      });

      await registerHandler(mockReq as NextApiRequest, mockRes as NextApiResponse);
      expect(mockRes.status).toHaveBeenCalledWith(409);
      expect(mockRes.json).toHaveBeenCalledWith({ error: 'User with this email already exists.' });
    });

    it('should return 500 for other Supabase errors', async () => {
      mockReq.body = { email: 'error@example.com', password: 'password123', username: 'erroruser' };
      mockSupabaseAuth.signUp.mockResolvedValueOnce({
        data: { user: null, session: null },
        error: { message: 'Some other error', status: 500 } as any,
      });

      await registerHandler(mockReq as NextApiRequest, mockRes as NextApiResponse);
      expect(mockRes.status).toHaveBeenCalledWith(500);
      expect(mockRes.json).toHaveBeenCalledWith({ error: 'An unexpected error occurred during registration.' });
    });
  });

  // --- Login Endpoint Tests ---
  describe('POST /api/auth/login', () => {
    it('should log in a user successfully', async () => {
      mockReq.body = { email: 'test@example.com', password: 'password123' };
      mockSupabaseAuth.signInWithPassword.mockResolvedValueOnce({
        data: {
          user: { id: 'some-uuid', email: 'test@example.com' } as any,
          session: { access_token: 'abc', refresh_token: 'def', expires_in: 3600 } as any,
        },
        error: null,
      });

      await loginHandler(mockReq as NextApiRequest, mockRes as NextApiResponse);

      expect(mockSupabaseAuth.signInWithPassword).toHaveBeenCalledWith({
        email: 'test@example.com',
        password: 'password123',
      });
      expect(mockRes.status).toHaveBeenCalledWith(200);
      expect(mockRes.json).toHaveBeenCalledWith(
        expect.objectContaining({ message: 'Login successful.' })
      );
      expect(mockRes.json).toHaveBeenCalledWith(
        expect.objectContaining({ user: { id: 'some-uuid', email: 'test@example.com' } })
      );
    });

    it('should return 400 if email or password is missing', async () => {
      mockReq.body = { email: 'test@example.com' };
      await loginHandler(mockReq as NextApiRequest, mockRes as NextApiResponse);
      expect(mockRes.status).toHaveBeenCalledWith(400);
      expect(mockRes.json).toHaveBeenCalledWith({ error: 'Email and password are required.' });
    });

    it('should return 401 for invalid credentials', async () => {
      mockReq.body = { email: 'bad@example.com', password: 'wrongpassword' };
      mockSupabaseAuth.signInWithPassword.mockResolvedValueOnce({
        data: { user: null, session: null },
        error: { message: 'Invalid login credentials', status: 400 } as any,
      });

      await loginHandler(mockReq as NextApiRequest, mockRes as NextApiResponse);
      expect(mockRes.status).toHaveBeenCalledWith(401);
      expect(mockRes.json).toHaveBeenCalledWith({ error: 'Invalid login credentials.' });
    });
  });

  // --- Logout Endpoint Tests ---
  describe('POST /api/auth/logout', () => {
    it('should log out a user successfully', async () => {
      mockSupabaseAuth.signOut.mockResolvedValueOnce({ error: null });

      await logoutHandler(mockReq as NextApiRequest, mockRes as NextApiResponse);

      expect(mockSupabaseAuth.signOut).toHaveBeenCalled();
      expect(mockRes.status).toHaveBeenCalledWith(200);
      expect(mockRes.json).toHaveBeenCalledWith({ message: 'Logout successful.' });
    });

    it('should return 500 for Supabase errors during logout', async () => {
      mockSupabaseAuth.signOut.mockResolvedValueOnce({
        error: { message: 'Sign out failed', status: 500 } as any,
      });

      await logoutHandler(mockReq as NextApiRequest, mockRes as NextApiResponse);
      expect(mockRes.status).toHaveBeenCalledWith(500);
      expect(mockRes.json).toHaveBeenCalledWith({ error: 'An unexpected error occurred during logout.' });
    });
  });
});
