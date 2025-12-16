import { NextApiRequest, NextApiResponse } from 'next';
import { supabase } from '../../../../lib/supabase';


export default async function handler(req: NextApiRequest, res: NextApiResponse) {
  if (req.method !== 'POST') {
    res.setHeader('Allow', ['POST']);
    return res.status(405).json({ error: `Method ${req.method} Not Allowed` });
  }

  try {
    const { error } = await supabase.auth.signOut();

    if (error) {
      console.error('Supabase sign out error:', error.message);
      return res.status(500).json({ error: 'An unexpected error occurred during logout.' });
    }

    return res.status(200).json({ message: 'Logout successful.' });

  } catch (err) {
    console.error('Unexpected server error:', err);
    return res.status(500).json({ error: 'Internal Server Error' });
  }
}
