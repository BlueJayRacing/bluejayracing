module.exports = {
    parser: '@typescript-eslint/parser',
    extends: [
      'plugin:react/recommended',
      'plugin:@typescript-eslint/recommended',
      'prettier',
    ],
    plugins: ['react', '@typescript-eslint'],
    env: {
      browser: true,
      es2021: true,
    },
    rules: {
      // Customize your linting rules here
    },
    settings: {
      react: {
        version: 'detect',
      },
    },
  };
  