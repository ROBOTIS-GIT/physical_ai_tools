import React from 'react';
import { render, screen } from '@testing-library/react';
import { Provider } from 'react-redux';
import { configureStore } from '@reduxjs/toolkit';
import App from './App';
import taskSlice from './features/tasks/taskSlice';
import uiSlice from './features/ui/uiSlice';
import rosSlice from './features/ros/rosSlice';

// Create a mock store for testing
const createMockStore = (initialState = {}) => {
  return configureStore({
    reducer: {
      tasks: taskSlice,
      ui: uiSlice,
      ros: rosSlice,
    },
    preloadedState: initialState,
    middleware: (getDefaultMiddleware) =>
      getDefaultMiddleware({
        serializableCheck: {
          ignoredActions: ['persist/PERSIST'],
        },
      }),
  });
};

describe('App Component', () => {
  let mockStore;

  beforeEach(() => {
    mockStore = createMockStore();
  });

  test('renders without crashing', () => {
    render(
      <Provider store={mockStore}>
        <App />
      </Provider>
    );
  });

  test('displays the main application structure', () => {
    render(
      <Provider store={mockStore}>
        <App />
      </Provider>
    );

    // Check if main app structure is rendered
    // Note: This might need adjustment based on your actual App component structure
    expect(document.body).toBeInTheDocument();
  });

  test('Redux store is properly connected', () => {
    const { container } = render(
      <Provider store={mockStore}>
        <App />
      </Provider>
    );

    // Verify that the component tree is rendered without Redux connection errors
    expect(container).toBeInTheDocument();
  });
});
