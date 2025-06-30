import { renderHook, act } from '@testing-library/react';
import { Provider } from 'react-redux';
import { configureStore } from '@reduxjs/toolkit';
import { useRosServiceCaller } from '../useRosServiceCaller';
import taskSlice from '../../features/tasks/taskSlice';
import uiSlice from '../../features/ui/uiSlice';
import rosSlice from '../../features/ros/rosSlice';
import PageType from '../../constants/pageType';
import TaskCommand from '../../constants/taskCommand';

// Mock ROSLIB
jest.mock('roslib', () => {
  const mockService = {
    callService: jest.fn(),
  };

  const mockRos = {
    isConnected: true,
    on: jest.fn(),
  };

  return {
    Ros: jest.fn(() => mockRos),
    Service: jest.fn(() => mockService),
    ServiceRequest: jest.fn((req) => req),
  };
});

// Mock console methods to avoid noise in tests
beforeEach(() => {
  jest.spyOn(console, 'log').mockImplementation(() => {});
  jest.spyOn(console, 'error').mockImplementation(() => {});
});

afterEach(() => {
  console.log.mockRestore();
  console.error.mockRestore();
  jest.clearAllMocks();
});

const createMockStore = (initialState = {}) => {
  const defaultState = {
    tasks: {
      taskInfo: {
        taskName: 'test-task',
        userId: 'test-user',
        taskInstruction: 'Test instruction',
        policyPath: '/test/policy',
        recordInferenceMode: false,
        fps: 30,
        tags: ['tag1', 'tag2'],
        warmupTime: 5,
        episodeTime: 20,
        resetTime: 5,
        numEpisodes: 5,
        pushToHub: true,
        privateMode: false,
        useOptimizedSave: true,
      },
    },
    ui: {
      currentPage: PageType.RECORD,
    },
    ros: {
      rosbridgeUrl: 'ws://localhost:9090',
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
  });
};

const renderHookWithProvider = (hook, storeState = {}) => {
  const store = createMockStore(storeState);
  const wrapper = ({ children }) => <Provider store={store}>{children}</Provider>;
  return renderHook(hook, { wrapper });
};

describe('useRosServiceCaller', () => {
  test('should initialize without errors', () => {
    const { result } = renderHookWithProvider(() => useRosServiceCaller());

    expect(result.current).toHaveProperty('sendRecordCommand');
    expect(result.current).toHaveProperty('getImageTopicList');
    expect(typeof result.current.sendRecordCommand).toBe('function');
    expect(typeof result.current.getImageTopicList).toBe('function');
  });

  test('should handle sendRecordCommand with valid command', async () => {
    const ROSLIB = require('roslib');
    const mockService = {
      callService: jest.fn((req, successCb) => {
        successCb({ success: true });
      }),
    };
    ROSLIB.Service.mockReturnValue(mockService);

    const { result } = renderHookWithProvider(() => useRosServiceCaller());

    await act(async () => {
      const response = await result.current.sendRecordCommand('start_record');
      expect(response).toEqual({ success: true });
    });

    expect(mockService.callService).toHaveBeenCalledTimes(1);
    expect(ROSLIB.ServiceRequest).toHaveBeenCalledWith(
      expect.objectContaining({
        command: TaskCommand.START_RECORD,
        task_info: expect.objectContaining({
          task_name: 'test-task',
          task_type: 'record',
          user_id: 'test-user',
        }),
      })
    );
  });

  test('should handle sendRecordCommand with inference page', async () => {
    const ROSLIB = require('roslib');
    const mockService = {
      callService: jest.fn((req, successCb) => {
        successCb({ success: true });
      }),
    };
    ROSLIB.Service.mockReturnValue(mockService);

    const { result } = renderHookWithProvider(() => useRosServiceCaller(), {
      ui: { currentPage: PageType.INFERENCE },
    });

    await act(async () => {
      await result.current.sendRecordCommand('start_inference');
    });

    expect(ROSLIB.ServiceRequest).toHaveBeenCalledWith(
      expect.objectContaining({
        task_info: expect.objectContaining({
          task_type: 'inference',
        }),
      })
    );
  });

  test('should handle different command types', async () => {
    const ROSLIB = require('roslib');
    const mockService = {
      callService: jest.fn((req, successCb) => {
        successCb({ success: true });
      }),
    };
    ROSLIB.Service.mockReturnValue(mockService);

    const { result } = renderHookWithProvider(() => useRosServiceCaller());

    const commands = ['start_record', 'stop', 'next', 'rerecord', 'finish'];
    const expectedCommands = [
      TaskCommand.START_RECORD,
      TaskCommand.STOP,
      TaskCommand.NEXT,
      TaskCommand.RERECORD,
      TaskCommand.FINISH,
    ];

    for (let i = 0; i < commands.length; i++) {
      await act(async () => {
        await result.current.sendRecordCommand(commands[i]);
      });

      expect(ROSLIB.ServiceRequest).toHaveBeenCalledWith(
        expect.objectContaining({
          command: expectedCommands[i],
        })
      );
    }
  });

  test('should handle invalid command', async () => {
    const { result } = renderHookWithProvider(() => useRosServiceCaller());

    await act(async () => {
      await expect(result.current.sendRecordCommand('invalid_command')).rejects.toThrow(
        'Unknown command: invalid_command'
      );
    });
  });

  test('should handle service call failure', async () => {
    const ROSLIB = require('roslib');
    const mockService = {
      callService: jest.fn((req, successCb, errorCb) => {
        errorCb('Service call failed');
      }),
    };
    ROSLIB.Service.mockReturnValue(mockService);

    const { result } = renderHookWithProvider(() => useRosServiceCaller());

    await act(async () => {
      await expect(result.current.sendRecordCommand('start_record')).rejects.toThrow();
    });
  });

  test('should handle getImageTopicList', async () => {
    const ROSLIB = require('roslib');
    const mockService = {
      callService: jest.fn((req, successCb) => {
        successCb({ topics: ['camera1', 'camera2'] });
      }),
    };
    ROSLIB.Service.mockReturnValue(mockService);

    const { result } = renderHookWithProvider(() => useRosServiceCaller());

    await act(async () => {
      const response = await result.current.getImageTopicList();
      expect(response).toEqual({ topics: ['camera1', 'camera2'] });
    });

    expect(mockService.callService).toHaveBeenCalledTimes(1);
  });

  test('should handle ROS connection error', async () => {
    const ROSLIB = require('roslib');
    const mockRos = {
      isConnected: false,
      on: jest.fn((event, callback) => {
        if (event === 'error') {
          // Simulate connection error
          setTimeout(() => callback('Connection failed'), 10);
        }
      }),
    };
    ROSLIB.Ros.mockReturnValue(mockRos);

    const { result } = renderHookWithProvider(() => useRosServiceCaller());

    await act(async () => {
      await expect(result.current.sendRecordCommand('start_record')).rejects.toThrow();
    });
  });

  test('should include all task info fields in service request', async () => {
    const ROSLIB = require('roslib');
    const mockService = {
      callService: jest.fn((req, successCb) => {
        successCb({ success: true });
      }),
    };
    ROSLIB.Service.mockReturnValue(mockService);

    const { result } = renderHookWithProvider(() => useRosServiceCaller());

    await act(async () => {
      await result.current.sendRecordCommand('start_record');
    });

    expect(ROSLIB.ServiceRequest).toHaveBeenCalledWith(
      expect.objectContaining({
        task_info: expect.objectContaining({
          task_name: 'test-task',
          task_type: 'record',
          user_id: 'test-user',
          task_instruction: 'Test instruction',
          policy_path: '/test/policy',
          record_inference_mode: false,
          fps: 30,
          tags: ['tag1', 'tag2'],
          warmup_time_s: 5,
          episode_time_s: 20,
          reset_time_s: 5,
          num_episodes: 5,
          push_to_hub: true,
          private_mode: false,
          use_optimized_save_mode: true,
        }),
      })
    );
  });
});
