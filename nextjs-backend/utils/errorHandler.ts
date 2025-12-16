import { NextApiResponse } from 'next';

export const sendErrorResponse = (res: NextApiResponse, statusCode: number, message: string, error?: any) => {
  res.status(statusCode).json({
    success: false,
    error: message,
    details: error ? (error instanceof Error ? error.message : String(error)) : undefined,
  });
};

export const handleApiError = (res: NextApiResponse, error: unknown, defaultMessage: string = 'Internal Server Error') => {
  if (error instanceof Error) {
    console.error(`API Error: ${error.message}`, error.stack);
    sendErrorResponse(res, 500, defaultMessage, error);
  } else {
    console.error(`Unknown API Error: ${error}`);
    sendErrorResponse(res, 500, defaultMessage, 'An unknown error occurred.');
  }
};
