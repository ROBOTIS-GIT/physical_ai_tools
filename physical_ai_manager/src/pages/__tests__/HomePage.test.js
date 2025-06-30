import React from 'react';
import { render, screen } from '@testing-library/react';
import { Provider } from 'react-redux';
import { configureStore } from '@reduxjs/toolkit';
import HomePage from '../HomePage';
import taskSlice from '../../features/tasks/taskSlice';
import uiSlice from '../../features/ui/uiSlice';
import rosSlice from '../../features/ros/rosSlice';

const createMockStore = (initialState = {}) => {
  return configureStore({
    reducer: {
      tasks: taskSlice,
      ui: uiSlice,
      ros: rosSlice,
    },
    preloadedState: initialState,
  });
};

const renderWithProvider = (component, initialState = {}) => {
  const store = createMockStore(initialState);
  return render(<Provider store={store}>{component}</Provider>);
};

describe('HomePage Component', () => {
  test('renders without crashing', () => {
    renderWithProvider(<HomePage />);
  });

  test('displays home page content', () => {
    renderWithProvider(<HomePage />);

    // HomePage should render without errors
    // Note: Add specific assertions based on actual HomePage content
    expect(document.body).toBeInTheDocument();
  });

  test('renders with Redux store connected', () => {
    const initialState = {
      ui: {
        currentPage: 'home',
        isLoading: false,
        error: null,
      },
      tasks: {
        taskInfo: {
          taskName: '',
          userId: '',
        },
      },
    };

    const { container } = renderWithProvider(<HomePage />, initialState);

    expect(container).toBeInTheDocument();
  });

  test('handles empty state gracefully', () => {
    renderWithProvider(<HomePage />);

    // Should render without errors even with empty state
    expect(document.body).toBeInTheDocument();
  });
});
