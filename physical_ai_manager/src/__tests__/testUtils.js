import React from 'react';
import { render } from '@testing-library/react';
import { Provider } from 'react-redux';
import { configureStore } from '@reduxjs/toolkit';
import taskSlice from '../features/tasks/taskSlice';
import uiSlice from '../features/ui/uiSlice';
import rosSlice from '../features/ros/rosSlice';
import PageType from '../constants/pageType';
import TaskPhase from '../constants/taskPhases';

/**
 * Create a mock Redux store for testing
 * @param {Object} initialState - Initial state for the store
 * @returns {Object} Redux store
 */
export const createMockStore = (initialState = {}) => {
  const defaultState = {
    tasks: {
      taskInfo: {
        taskName: '',
        taskType: '',
        taskInstruction: '',
        policyPath: '',
        recordInferenceMode: false,
        userId: '',
        fps: 30,
        tags: [],
        warmupTime: 5,
        episodeTime: 20,
        resetTime: 5,
        numEpisodes: 5,
        token: '',
        pushToHub: true,
        privateMode: false,
        useOptimizedSave: true,
      },
      taskStatus: {
        robotType: '',
        taskName: 'idle',
        running: false,
        phase: TaskPhase.READY,
        progress: 0,
        totalTime: 0,
        proceedTime: 0,
        currentEpisodeNumber: 0,
        userId: '',
        usedStorageSize: 0,
        totalStorageSize: 0,
        usedCpu: 0,
        usedRamSize: 0,
        totalRamSize: 0,
        error: '',
        topicReceived: false,
      },
      availableRobots: [],
      availableCameras: [],
      policyList: [],
      datasetList: [],
    },
    ui: {
      isLoading: false,
      error: null,
      currentPage: PageType.HOME,
      sidebarOpen: false,
      modalOpen: false,
      notifications: [],
      robotTypeList: [],
      isFirstLoad: true,
    },
    ros: {
      rosbridgeUrl: 'ws://localhost:9090',
      connected: false,
      connectionError: null,
    },
    ...initialState,
  };

  return configureStore({
    reducer: {
      tasks: taskSlice,
      ui: uiSlice,
      ros: rosSlice,
    },
    preloadedState: defaultState,
    middleware: (getDefaultMiddleware) =>
      getDefaultMiddleware({
        serializableCheck: {
          ignoredActions: ['persist/PERSIST'],
        },
      }),
  });
};

/**
 * Render component with Redux Provider and mock store
 * @param {ReactNode} component - Component to render
 * @param {Object} initialState - Initial state for the store
 * @param {Object} renderOptions - Additional render options
 * @returns {Object} Render result
 */
export const renderWithProvider = (component, initialState = {}, renderOptions = {}) => {
  const store = createMockStore(initialState);
  const Wrapper = ({ children }) => <Provider store={store}>{children}</Provider>;

  return {
    ...render(component, { wrapper: Wrapper, ...renderOptions }),
    store,
  };
};

/**
 * Create mock task info for testing
 * @param {Object} overrides - Properties to override
 * @returns {Object} Mock task info
 */
export const createMockTaskInfo = (overrides = {}) => ({
  taskName: 'test-task',
  taskType: 'record',
  taskInstruction: 'Test instruction',
  policyPath: '/test/policy',
  recordInferenceMode: false,
  userId: 'test-user',
  fps: 30,
  tags: ['tag1', 'tag2'],
  warmupTime: 5,
  episodeTime: 20,
  resetTime: 5,
  numEpisodes: 5,
  token: 'test-token',
  pushToHub: true,
  privateMode: false,
  useOptimizedSave: true,
  ...overrides,
});

/**
 * Create mock task status for testing
 * @param {Object} overrides - Properties to override
 * @returns {Object} Mock task status
 */
export const createMockTaskStatus = (overrides = {}) => ({
  robotType: 'test-robot',
  taskName: 'test-task',
  running: false,
  phase: TaskPhase.READY,
  progress: 0,
  totalTime: 0,
  proceedTime: 0,
  currentEpisodeNumber: 0,
  userId: 'test-user',
  usedStorageSize: 0,
  totalStorageSize: 1000,
  usedCpu: 25,
  usedRamSize: 512,
  totalRamSize: 2048,
  error: '',
  topicReceived: true,
  ...overrides,
});

/**
 * Create mock UI state for testing
 * @param {Object} overrides - Properties to override
 * @returns {Object} Mock UI state
 */
export const createMockUIState = (overrides = {}) => ({
  isLoading: false,
  error: null,
  currentPage: PageType.HOME,
  sidebarOpen: false,
  modalOpen: false,
  notifications: [],
  robotTypeList: ['robot1', 'robot2'],
  isFirstLoad: true,
  ...overrides,
});

/**
 * Mock ROSLIB for testing
 */
export const mockROSLIB = () => {
  const mockService = {
    callService: jest.fn(),
  };

  const mockRos = {
    isConnected: true,
    on: jest.fn(),
    close: jest.fn(),
  };

  return {
    Ros: jest.fn(() => mockRos),
    Service: jest.fn(() => mockService),
    ServiceRequest: jest.fn((req) => req),
    Topic: jest.fn(),
    Message: jest.fn(),
  };
};

/**
 * Wait for a specified time (for testing async operations)
 * @param {number} ms - Milliseconds to wait
 * @returns {Promise} Promise that resolves after the specified time
 */
export const wait = (ms = 0) => new Promise((resolve) => setTimeout(resolve, ms));

/**
 * Flush all pending promises
 * @returns {Promise} Promise that resolves after all pending promises
 */
export const flushPromises = () => new Promise((resolve) => setImmediate(resolve));

/**
 * Mock console methods to avoid noise in tests
 */
export const mockConsole = () => {
  const consoleMocks = {
    log: jest.spyOn(console, 'log').mockImplementation(() => {}),
    error: jest.spyOn(console, 'error').mockImplementation(() => {}),
    warn: jest.spyOn(console, 'warn').mockImplementation(() => {}),
    info: jest.spyOn(console, 'info').mockImplementation(() => {}),
  };

  return {
    restore: () => {
      Object.values(consoleMocks).forEach((mock) => mock.mockRestore());
    },
  };
};

/**
 * Create a mock event object for testing
 * @param {Object} overrides - Properties to override
 * @returns {Object} Mock event object
 */
export const createMockEvent = (overrides = {}) => ({
  preventDefault: jest.fn(),
  stopPropagation: jest.fn(),
  target: {
    value: '',
    checked: false,
    ...overrides.target,
  },
  ...overrides,
});

/**
 * Helper to test Redux action dispatching
 * @param {Function} action - Redux action creator
 * @param {*} payload - Action payload
 * @returns {Object} Action object
 */
export const testAction = (action, payload) => {
  const result = action(payload);
  expect(result).toHaveProperty('type');
  if (payload !== undefined) {
    expect(result).toHaveProperty('payload', payload);
  }
  return result;
};
