// nextjs-backend/tests/unit/supabase.test.ts
import { supabase } from '../../lib/supabase';

describe('Supabase Client Initialization', () => {
  // Set up mock environment variables for testing
  const OLD_ENV = process.env;

  beforeEach(() => {
    jest.resetModules(); // Clears the cache to reload modules with new env vars
    process.env = {
      ...OLD_ENV,
      NEXT_PUBLIC_SUPABASE_URL: 'http://localhost:54321', // Mock URL
      NEXT_PUBLIC_SUPABASE_ANON_KEY: 'mock_anon_key',     // Mock Anon Key
    };
  });

  afterAll(() => {
    process.env = OLD_ENV; // Restore old environment
  });

  it('should initialize the Supabase client correctly', () => {
    // Check if supabase object exists
    expect(supabase).toBeDefined();
    // In a more detailed test, you might check if the URL and key are part of the client's config
    // (though Supabase client doesn't expose them directly in a public API often).
    // This is a basic check that it doesn't throw and is an object.
    expect(typeof supabase.auth).toBe('object');
    expect(typeof supabase.from).toBe('function');
  });

  it('should throw an error if NEXT_PUBLIC_SUPABASE_URL is missing', () => {
    delete process.env.NEXT_PUBLIC_SUPABASE_URL;
    // We expect the module import itself to throw if env vars are missing
    // Need to wrap the import in a function to catch the throw
    expect(() => { require('../../lib/supabase'); }).toThrow("Missing environment variable: NEXT_PUBLIC_SUPABASE_URL");
  });

  it('should throw an error if NEXT_PUBLIC_SUPABASE_ANON_KEY is missing', () => {
    delete process.env.NEXT_PUBLIC_SUPABASE_ANON_KEY;
    expect(() => { require('../../lib/supabase'); }).toThrow("Missing environment variable: NEXT_PUBLIC_SUPABASE_ANON_KEY");
  });
});
