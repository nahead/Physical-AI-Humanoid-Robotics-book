// docusaurus/jest.config.js
module.exports = {
  testEnvironment: 'jsdom',
  setupFilesAfterEnv: ['<rootDir>/setupTests.js'],
  testPathIgnorePatterns: [
    '<rootDir>/node_modules/',
    '<rootDir>/.docusaurus/',
    '<rootDir>/build/',
  ],
  transform: {
    '^.+\.(js|jsx|ts|tsx)$': '<rootDir>/node_modules/babel-jest',
    '^.+\.css$': '<rootDir>/cssTransformer.js', // For CSS modules
  },
  moduleNameMapper: {
    '^@theme/(.*)$': '<rootDir>/src/theme/$1', // Map Docusaurus theme aliases
    '^@site/(.*)$': '<rootDir>/src/$1', // Map @site alias
    '\.(css|less|scss|sass)$': 'identity-obj-proxy', // Mock CSS imports
  },
};
