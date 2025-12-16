// nextjs-backend/tests/__mocks__/lib/supabase.ts
// This is a mock for the supabase client for testing purposes

let mockSignInWithPassword, mockSignUp, mockSignOut, from;

if (process.env.JEST_WORKER_ID) {
  mockSignInWithPassword = jest.fn();
  mockSignUp = jest.fn();
  mockSignOut = jest.fn();
  from = jest.fn(() => ({
    select: jest.fn(() => ({
      eq: jest.fn(() => ({
        single: jest.fn(() => ({
          data: null,
          error: null,
        })),
      })),
    })),
  }));
} else {
  // Provide non-Jest mock functions for other environments if needed
  mockSignInWithPassword = () => {};
  mockSignUp = () => {};
  mockSignOut = () => {};
  from = () => ({ select: () => ({ eq: () => ({ single: () => ({ data: null, error: null }) }) }) });
}

export { mockSignInWithPassword, mockSignUp, mockSignOut };

export const supabase = {
  auth: {
    signInWithPassword: mockSignInWithPassword,
    signUp: mockSignUp,
    signOut: mockSignOut,
  },
  from,
};

