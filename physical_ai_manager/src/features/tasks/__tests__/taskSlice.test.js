import taskReducer, {
  setTaskInfo,
  resetTaskInfo,
  setTaskStatus,
  setRobotType,
  selectRobotType,
  resetTaskStatus,
  setTaskType,
  setTaskInstruction,
  setPolicyPath,
  setRecordInferenceMode,
  addTag,
  removeTag,
} from '../taskSlice';
import TaskPhase from '../../../constants/taskPhases';

describe('taskSlice', () => {
  const initialState = {
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
  };

  test('should return the initial state', () => {
    expect(taskReducer(undefined, { type: 'unknown' })).toEqual(initialState);
  });

  describe('taskInfo actions', () => {
    test('should handle setTaskInfo', () => {
      const newTaskInfo = {
        taskName: 'test-task',
        taskType: 'record',
        userId: 'test-user',
      };

      const actual = taskReducer(initialState, setTaskInfo(newTaskInfo));

      expect(actual.taskInfo).toEqual({
        ...initialState.taskInfo,
        ...newTaskInfo,
      });
    });

    test('should handle resetTaskInfo', () => {
      const modifiedState = {
        ...initialState,
        taskInfo: {
          ...initialState.taskInfo,
          taskName: 'test-task',
          userId: 'test-user',
        },
      };

      const actual = taskReducer(modifiedState, resetTaskInfo());

      expect(actual.taskInfo).toEqual(initialState.taskInfo);
    });

    test('should handle setTaskType', () => {
      const actual = taskReducer(initialState, setTaskType('inference'));

      expect(actual.taskInfo.taskType).toBe('inference');
    });

    test('should handle setTaskInstruction', () => {
      const instruction = 'Pick up the red cube';
      const actual = taskReducer(initialState, setTaskInstruction(instruction));

      expect(actual.taskInfo.taskInstruction).toBe(instruction);
    });

    test('should handle setPolicyPath', () => {
      const policyPath = '/path/to/policy';
      const actual = taskReducer(initialState, setPolicyPath(policyPath));

      expect(actual.taskInfo.policyPath).toBe(policyPath);
    });

    test('should handle setRecordInferenceMode', () => {
      const actual = taskReducer(initialState, setRecordInferenceMode(true));

      expect(actual.taskInfo.recordInferenceMode).toBe(true);
    });
  });

  describe('taskStatus actions', () => {
    test('should handle setTaskStatus', () => {
      const newTaskStatus = {
        running: true,
        phase: TaskPhase.RECORDING,
        progress: 50,
      };

      const actual = taskReducer(initialState, setTaskStatus(newTaskStatus));

      expect(actual.taskStatus).toEqual({
        ...initialState.taskStatus,
        ...newTaskStatus,
      });
    });

    test('should handle setRobotType', () => {
      const actual = taskReducer(initialState, setRobotType('robot1'));

      expect(actual.taskStatus.robotType).toBe('robot1');
    });

    test('should handle selectRobotType', () => {
      const actual = taskReducer(initialState, selectRobotType('robot2'));

      expect(actual.taskStatus.robotType).toBe('robot2');
    });

    test('should handle resetTaskStatus', () => {
      const modifiedState = {
        ...initialState,
        taskStatus: {
          ...initialState.taskStatus,
          running: true,
          progress: 75,
        },
      };

      const actual = taskReducer(modifiedState, resetTaskStatus());

      expect(actual.taskStatus).toEqual(initialState.taskStatus);
    });
  });

  describe('tag actions', () => {
    test('should handle addTag', () => {
      const actual = taskReducer(initialState, addTag('test-tag'));

      expect(actual.taskInfo.tags).toContain('test-tag');
      expect(actual.taskInfo.tags).toHaveLength(1);
    });

    test('should not add duplicate tags', () => {
      const stateWithTag = {
        ...initialState,
        taskInfo: {
          ...initialState.taskInfo,
          tags: ['existing-tag'],
        },
      };

      const actual = taskReducer(stateWithTag, addTag('existing-tag'));

      expect(actual.taskInfo.tags).toEqual(['existing-tag']);
      expect(actual.taskInfo.tags).toHaveLength(1);
    });

    test('should handle removeTag', () => {
      const stateWithTags = {
        ...initialState,
        taskInfo: {
          ...initialState.taskInfo,
          tags: ['tag1', 'tag2', 'tag3'],
        },
      };

      const actual = taskReducer(stateWithTags, removeTag('tag2'));

      expect(actual.taskInfo.tags).toEqual(['tag1', 'tag3']);
      expect(actual.taskInfo.tags).not.toContain('tag2');
    });

    test('should handle removing non-existent tag', () => {
      const stateWithTags = {
        ...initialState,
        taskInfo: {
          ...initialState.taskInfo,
          tags: ['tag1', 'tag2'],
        },
      };

      const actual = taskReducer(stateWithTags, removeTag('non-existent'));

      expect(actual.taskInfo.tags).toEqual(['tag1', 'tag2']);
    });
  });

  test('should handle multiple actions in sequence', () => {
    let state = initialState;

    // Add task info
    state = taskReducer(
      state,
      setTaskInfo({
        taskName: 'multi-test',
        userId: 'test-user',
      })
    );

    // Add tags
    state = taskReducer(state, addTag('tag1'));
    state = taskReducer(state, addTag('tag2'));

    // Set task status
    state = taskReducer(
      state,
      setTaskStatus({
        running: true,
        progress: 25,
      })
    );

    expect(state.taskInfo.taskName).toBe('multi-test');
    expect(state.taskInfo.userId).toBe('test-user');
    expect(state.taskInfo.tags).toEqual(['tag1', 'tag2']);
    expect(state.taskStatus.running).toBe(true);
    expect(state.taskStatus.progress).toBe(25);
  });
});
