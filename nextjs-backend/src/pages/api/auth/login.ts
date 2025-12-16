import { NextApiRequest, NextApiResponse } from 'next';
import { supabase } from '../../../../lib/supabase';


export default async function handler(req: NextApiRequest, res: NextApiResponse) {
  if (req.method !== 'POST') {
    res.setHeader('Allow', ['POST']);
    return res.status(405).json({ error: `Method ${req.method} Not Allowed` });
  }

  const { email, password } = req.body;

  if (!email || !password) {
    return res.status(400).json({ error: 'Email and password are required.' });
  }

  try {
    const { data, error } = await supabase.auth.signInWithPassword({
      email,
      password,
    });

    if (error) {
      console.error('Supabase sign in error:', error.message);
      // It's good practice to not expose detailed database errors to the client.
      if (error.message === 'Invalid login credentials') {
        return res.status(401).json({ error: 'Invalid login credentials.' });
      }
      return res.status(500).json({ error: 'An unexpected error occurred during login.' });
    }

    if (data.session && data.user) {
      return res.status(200).json({
        message: 'Login successful.',
        user: {
          id: data.user.id,
          email: data.user.email,
        },
        session: {
          access_token: data.session.access_token,
          refresh_token: data.session.refresh_token,
          expires_in: data.session.expires_in,
        }
      });
    }

    // Fallback in case session or user is null but there was no error
    return res.status(500).json({ error: 'Could not log in.' });

  } catch (err) {
    console.error('Unexpected server error:', err);
    return res.status(500).json({ error: 'Internal Server Error' });
  }
}
