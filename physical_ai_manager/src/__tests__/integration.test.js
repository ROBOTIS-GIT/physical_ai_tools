import React from 'react';
import { screen, fireEvent, waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import App from '../App';
import { renderWithProvider, createMockTaskInfo, mockConsole } from './testUtils';
import PageType from '../constants/pageType';
import TaskPhase from '../constants/taskPhases';

// Mock ROSLIB globally
jest.mock('roslib', () => ({
  Ros: jest.fn(() => ({
    isConnected: true,
    on: jest.fn(),
    close: jest.fn(),
  })),
  Service: jest.fn(() => ({
    callService: jest.fn((req, successCb) => {
      successCb({ success: true });
    }),
  })),
  ServiceRequest: jest.fn((req) => req),
  Topic: jest.fn(() => ({
    subscribe: jest.fn(),
    unsubscribe: jest.fn(),
    advertise: jest.fn(),
  })),
  Message: jest.fn(),
}));

describe('Integration Tests', () => {
  let consoleMock;

  beforeEach(() => {
    consoleMock = mockConsole();
  });

  afterEach(() => {
    consoleMock.restore();
    jest.clearAllMocks();
  });

  describe('App Initialization', () => {
    test('renders App component with initial state', () => {
      renderWithProvider(<App />);

      // App should render without crashing
      expect(document.body).toBeInTheDocument();
    });

    test('initializes Redux store correctly', () => {
      const { store } = renderWithProvider(<App />);

      const state = store.getState();

      // Check that all slices are initialized
      expect(state).toHaveProperty('tasks');
      expect(state).toHaveProperty('ui');
      expect(state).toHaveProperty('ros');

      // Check initial values
      expect(state.ui.currentPage).toBe(PageType.HOME);
      expect(state.tasks.taskStatus.phase).toBe(TaskPhase.READY);
    });
  });

  describe('State Management Integration', () => {
    test('updates task info across components', () => {
      const initialState = {
        tasks: {
          taskInfo: createMockTaskInfo({
            taskName: 'integration-test',
            userId: 'test-user',
          }),
        },
      };

      const { store } = renderWithProvider(<App />, initialState);

      const state = store.getState();
      expect(state.tasks.taskInfo.taskName).toBe('integration-test');
      expect(state.tasks.taskInfo.userId).toBe('test-user');
    });

    test('handles page navigation state changes', async () => {
      const { store } = renderWithProvider(<App />);

      // Dispatch page change action
      store.dispatch({ type: 'ui/moveToPage', payload: PageType.RECORD });

      await waitFor(() => {
        const state = store.getState();
        expect(state.ui.currentPage).toBe(PageType.RECORD);
        expect(state.ui.isFirstLoad).toBe(false);
      });
    });

    test('manages loading states correctly', async () => {
      const { store } = renderWithProvider(<App />);

      // Start loading
      store.dispatch({ type: 'ui/setLoading', payload: true });

      await waitFor(() => {
        const state = store.getState();
        expect(state.ui.isLoading).toBe(true);
      });

      // Stop loading
      store.dispatch({ type: 'ui/setLoading', payload: false });

      await waitFor(() => {
        const state = store.getState();
        expect(state.ui.isLoading).toBe(false);
      });
    });
  });

  describe('Error Handling Integration', () => {
    test('handles and displays errors correctly', async () => {
      const { store } = renderWithProvider(<App />);

      const errorMessage = 'Test error message';
      store.dispatch({ type: 'ui/setError', payload: errorMessage });

      await waitFor(() => {
        const state = store.getState();
        expect(state.ui.error).toBe(errorMessage);
      });

      // Clear error
      store.dispatch({ type: 'ui/clearError' });

      await waitFor(() => {
        const state = store.getState();
        expect(state.ui.error).toBeNull();
      });
    });

    test('handles notification system', async () => {
      const { store } = renderWithProvider(<App />);

      const notification = {
        type: 'success',
        message: 'Operation completed successfully',
      };

      store.dispatch({ type: 'ui/addNotification', payload: notification });

      await waitFor(() => {
        const state = store.getState();
        expect(state.ui.notifications).toHaveLength(1);
        expect(state.ui.notifications[0]).toMatchObject(notification);
        expect(state.ui.notifications[0]).toHaveProperty('id');
      });
    });
  });

  describe('Task Workflow Integration', () => {
    test('completes full task setup workflow', async () => {
      const { store } = renderWithProvider(<App />);

      // Set task info
      const taskInfo = createMockTaskInfo({
        taskName: 'workflow-test',
        taskType: 'record',
        userId: 'workflow-user',
      });

      store.dispatch({ type: 'tasks/setTaskInfo', payload: taskInfo });

      // Set robot type
      store.dispatch({ type: 'tasks/setRobotType', payload: 'test-robot' });

      // Add tags
      store.dispatch({ type: 'tasks/addTag', payload: 'workflow-tag' });

      // Update task status
      store.dispatch({
        type: 'tasks/setTaskStatus',
        payload: {
          running: true,
          phase: TaskPhase.RECORDING,
          progress: 50,
        },
      });

      await waitFor(() => {
        const state = store.getState();

        // Verify task info
        expect(state.tasks.taskInfo.taskName).toBe('workflow-test');
        expect(state.tasks.taskInfo.userId).toBe('workflow-user');

        // Verify robot type
        expect(state.tasks.taskStatus.robotType).toBe('test-robot');

        // Verify tags
        expect(state.tasks.taskInfo.tags).toContain('workflow-tag');

        // Verify task status
        expect(state.tasks.taskStatus.running).toBe(true);
        expect(state.tasks.taskStatus.phase).toBe(TaskPhase.RECORDING);
        expect(state.tasks.taskStatus.progress).toBe(50);
      });
    });

    test('handles task reset correctly', async () => {
      const { store } = renderWithProvider(<App />);

      // Set some initial data
      store.dispatch({
        type: 'tasks/setTaskInfo',
        payload: createMockTaskInfo(),
      });

      store.dispatch({
        type: 'tasks/setTaskStatus',
        payload: { running: true, progress: 75 },
      });

      // Reset task info
      store.dispatch({ type: 'tasks/resetTaskInfo' });

      // Reset task status
      store.dispatch({ type: 'tasks/resetTaskStatus' });

      await waitFor(() => {
        const state = store.getState();

        // Verify reset
        expect(state.tasks.taskInfo.taskName).toBe('');
        expect(state.tasks.taskInfo.userId).toBe('');
        expect(state.tasks.taskStatus.running).toBe(false);
        expect(state.tasks.taskStatus.progress).toBe(0);
        expect(state.tasks.taskStatus.phase).toBe(TaskPhase.READY);
      });
    });
  });

  describe('UI State Integration', () => {
    test('manages sidebar state across pages', async () => {
      const { store } = renderWithProvider(<App />);

      // Open sidebar
      store.dispatch({ type: 'ui/setSidebarOpen', payload: true });

      await waitFor(() => {
        const state = store.getState();
        expect(state.ui.sidebarOpen).toBe(true);
      });

      // Change page
      store.dispatch({ type: 'ui/moveToPage', payload: PageType.RECORD });

      await waitFor(() => {
        const state = store.getState();
        expect(state.ui.currentPage).toBe(PageType.RECORD);
        expect(state.ui.sidebarOpen).toBe(true); // Should remain open
      });

      // Toggle sidebar
      store.dispatch({ type: 'ui/toggleSidebar' });

      await waitFor(() => {
        const state = store.getState();
        expect(state.ui.sidebarOpen).toBe(false);
      });
    });

    test('manages modal state correctly', async () => {
      const { store } = renderWithProvider(<App />);

      // Open modal
      store.dispatch({ type: 'ui/setModalOpen', payload: true });

      await waitFor(() => {
        const state = store.getState();
        expect(state.ui.modalOpen).toBe(true);
      });

      // Close modal
      store.dispatch({ type: 'ui/setModalOpen', payload: false });

      await waitFor(() => {
        const state = store.getState();
        expect(state.ui.modalOpen).toBe(false);
      });
    });
  });

  describe('Data Persistence and Consistency', () => {
    test('maintains data consistency across state updates', async () => {
      const { store } = renderWithProvider(<App />);

      const initialTaskInfo = createMockTaskInfo({
        taskName: 'consistency-test',
        tags: ['initial-tag'],
      });

      // Set initial task info
      store.dispatch({ type: 'tasks/setTaskInfo', payload: initialTaskInfo });

      // Add more tags
      store.dispatch({ type: 'tasks/addTag', payload: 'additional-tag1' });
      store.dispatch({ type: 'tasks/addTag', payload: 'additional-tag2' });

      // Update other properties
      store.dispatch({ type: 'tasks/setTaskType', payload: 'inference' });
      store.dispatch({ type: 'tasks/setPolicyPath', payload: '/new/policy/path' });

      await waitFor(() => {
        const state = store.getState();

        // Verify all data is consistent
        expect(state.tasks.taskInfo.taskName).toBe('consistency-test');
        expect(state.tasks.taskInfo.taskType).toBe('inference');
        expect(state.tasks.taskInfo.policyPath).toBe('/new/policy/path');
        expect(state.tasks.taskInfo.tags).toEqual([
          'initial-tag',
          'additional-tag1',
          'additional-tag2',
        ]);

        // Verify other properties remain unchanged
        expect(state.tasks.taskInfo.fps).toBe(30);
        expect(state.tasks.taskInfo.pushToHub).toBe(true);
      });
    });
  });
});
