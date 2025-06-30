# Physical AI Manager

Web UI for Physical AI Platform

## Getting Started

### Prerequisites

- Node.js (v16 or higher)
- npm or yarn

### Installation

```bash
npm install
```

### Development

```bash
npm start
```

Runs the app in development mode. Open [http://localhost:3000](http://localhost:3000) to view it in the browser.

### Testing

#### Run all tests

```bash
npm test
```

#### Run tests in watch mode

```bash
npm run test:watch
```

#### Run tests with coverage

```bash
npm run test:coverage
```

#### Run tests in CI mode

```bash
npm run test:ci
```

#### Test Structure

- **Unit Tests**: Test individual components and functions
  - Components: `src/components/__tests__/`
  - Redux slices: `src/features/**/__tests__/`
  - Hooks: `src/hooks/__tests__/`
- **Integration Tests**: Test component interactions and full workflows
  - `src/__tests__/integration.test.js`
- **Test Utilities**: Shared testing helpers
  - `src/__tests__/testUtils.js`

#### Test Coverage

The project maintains a minimum coverage threshold of 70% for:

- Statements
- Branches
- Functions
- Lines

Coverage reports are generated in the `coverage/` directory.

### Building

```bash
npm run build
```

Builds the app for production to the `build` folder.

### Docker

```bash
docker build -t physical-ai-manager .
docker run -p 3000:80 physical-ai-manager
```

## Available Scripts

- `npm start` - Runs the development server
- `npm test` - Runs the test suite
- `npm run test:watch` - Runs tests in watch mode
- `npm run test:coverage` - Runs tests with coverage report
- `npm run test:ci` - Runs tests in CI mode (no watch)
- `npm run build` - Builds the production bundle
- `npm run eject` - Ejects from Create React App (one-way operation)

## Technology Stack

- **Frontend**: React 19, Redux Toolkit, Tailwind CSS
- **Testing**: Jest, React Testing Library, Testing Library User Event
- **Build Tool**: Create React App
- **State Management**: Redux Toolkit
- **Styling**: Tailwind CSS
- **ROS Communication**: roslib

## Project Structure

```
src/
├── components/          # Reusable UI components
├── pages/              # Page components
├── features/           # Redux slices and related logic
├── hooks/              # Custom React hooks
├── constants/          # Application constants
├── shared/             # Shared utilities
├── store/              # Redux store configuration
└── __tests__/          # Test utilities and integration tests
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Add tests for your changes
4. Ensure all tests pass
5. Submit a pull request

## License

Licensed under the Apache License, Version 2.0
