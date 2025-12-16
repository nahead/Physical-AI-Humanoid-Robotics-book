// nextjs-backend/jest.config.js
module.exports = {
  preset: 'ts-jest',
  testEnvironment: 'node',
  moduleNameMapper: {
    '^@/(.*)$': '<rootDir>/src/$1', // Map @/ to src/ if you use it
  },
  modulePathIgnorePatterns: ['<rootDir>/nextjs-backend/.next'], // Ignore Next.js build output
  globals: {
    'ts-jest': {
      tsconfig: '<rootDir>/tsconfig.jest.json',
    },
  },
};
