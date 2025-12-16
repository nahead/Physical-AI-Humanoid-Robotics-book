
import { NextApiRequest, NextApiResponse } from 'next';
import { supabase } from '../../../../lib/supabase';
// Corrected import path

export default async function handler(req: NextApiRequest, res: NextApiResponse) {
  if (req.method !== 'POST') {
    res.setHeader('Allow', ['POST']);
    return res.status(405).json({ error: `Method ${req.method} Not Allowed` });
  }

  const { email, password, full_name, software_background, hardware_background } = req.body;

  if (!email || !password || !full_name || !software_background || !hardware_background) {
    return res.status(400).json({ error: 'All fields are required.' });
  }

  try {
    const { data, error } = await supabase.auth.signUp({
      email,
      password,
      options: {
        data: {
          full_name,
          software_background,
          hardware_background,
        }
      }
    });

    if (error) {
      console.error('Supabase sign up error:', error.message);
      if (error.message.includes('User already registered')) {
        return res.status(409).json({ error: 'User with this email already exists.' });
      }
      return res.status(500).json({ error: 'An unexpected error occurred during registration.' });
    }

    if (data.user) {
      return res.status(200).json({
        message: 'Registration successful! Please check your email to confirm your account.',
        user: {
          id: data.user.id,
          email: data.user.email,
        }
      });
    }

    return res.status(500).json({ error: 'Could not create user.' });

  } catch (err) {
    console.error('Unexpected server error:', err);
    return res.status(500).json({ error: 'Internal Server Error' });
  }
}
