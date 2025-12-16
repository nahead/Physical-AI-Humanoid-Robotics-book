// nextjs-backend/lib/middleware.ts
import { NextApiRequest, NextApiResponse } from 'next';
import Cors from 'cors';

// Initialize the cors middleware
const cors = Cors({
  methods: ['POST', 'GET', 'HEAD', 'OPTIONS'],
  origin: [
    'http://localhost:3000', // Allow requests from your Docusaurus frontend
    'https://physical-ai-humanoid-roboticsbook.vercel.app', // Allow Vercel frontend
  ],
});

// Helper method to wait for a middleware to execute before continuing
// And to throw an error in case of an error in the middleware
function runMiddleware(
  req: NextApiRequest,
  res: NextApiResponse,
  fn: Function
) {
  return new Promise((resolve, reject) => {
    fn(req, res, (result: any) => {
      if (result instanceof Error) {
        return reject(result);
      }
      return resolve(result);
    });
  });
}

export default async function corsMiddleware(req: NextApiRequest, res: NextApiResponse) {
  await runMiddleware(req, res, cors);
}
