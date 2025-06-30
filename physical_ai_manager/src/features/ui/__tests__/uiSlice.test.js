import uiReducer, {
  setLoading,
  setError,
  clearError,
  setCurrentPage,
  moveToPage,
  toggleSidebar,
  setSidebarOpen,
  setModalOpen,
  addNotification,
  removeNotification,
  clearNotifications,
  setRobotTypeList,
} from '../uiSlice';
import PageType from '../../../constants/pageType';

describe('uiSlice', () => {
  const initialState = {
    isLoading: false,
    error: null,
    currentPage: PageType.HOME,
    sidebarOpen: false,
    modalOpen: false,
    notifications: [],
    robotTypeList: [],
    isFirstLoad: true,
  };

  test('should return the initial state', () => {
    expect(uiReducer(undefined, { type: 'unknown' })).toEqual(initialState);
  });

  describe('loading and error actions', () => {
    test('should handle setLoading', () => {
      const actual = uiReducer(initialState, setLoading(true));
      expect(actual.isLoading).toBe(true);

      const actualFalse = uiReducer(actual, setLoading(false));
      expect(actualFalse.isLoading).toBe(false);
    });

    test('should handle setError', () => {
      const errorMessage = 'Something went wrong';
      const actual = uiReducer(initialState, setError(errorMessage));

      expect(actual.error).toBe(errorMessage);
    });

    test('should handle clearError', () => {
      const stateWithError = {
        ...initialState,
        error: 'Some error',
      };

      const actual = uiReducer(stateWithError, clearError());

      expect(actual.error).toBeNull();
    });
  });

  describe('page navigation actions', () => {
    test('should handle setCurrentPage', () => {
      const actual = uiReducer(initialState, setCurrentPage(PageType.RECORD));

      expect(actual.currentPage).toBe(PageType.RECORD);
    });

    test('should handle moveToPage', () => {
      const actual = uiReducer(initialState, moveToPage(PageType.INFERENCE));

      expect(actual.currentPage).toBe(PageType.INFERENCE);
      expect(actual.isFirstLoad).toBe(false);
    });

    test('should set isFirstLoad to false when moving to page', () => {
      const actual = uiReducer(initialState, moveToPage(PageType.RECORD));

      expect(actual.isFirstLoad).toBe(false);
    });
  });

  describe('sidebar actions', () => {
    test('should handle toggleSidebar', () => {
      const actual = uiReducer(initialState, toggleSidebar());
      expect(actual.sidebarOpen).toBe(true);

      const actualToggled = uiReducer(actual, toggleSidebar());
      expect(actualToggled.sidebarOpen).toBe(false);
    });

    test('should handle setSidebarOpen', () => {
      const actual = uiReducer(initialState, setSidebarOpen(true));
      expect(actual.sidebarOpen).toBe(true);

      const actualClosed = uiReducer(actual, setSidebarOpen(false));
      expect(actualClosed.sidebarOpen).toBe(false);
    });
  });

  describe('modal actions', () => {
    test('should handle setModalOpen', () => {
      const actual = uiReducer(initialState, setModalOpen(true));
      expect(actual.modalOpen).toBe(true);

      const actualClosed = uiReducer(actual, setModalOpen(false));
      expect(actualClosed.modalOpen).toBe(false);
    });
  });

  describe('notification actions', () => {
    test('should handle addNotification', () => {
      const notification = {
        type: 'success',
        message: 'Operation completed',
      };

      const actual = uiReducer(initialState, addNotification(notification));

      expect(actual.notifications).toHaveLength(1);
      expect(actual.notifications[0]).toMatchObject(notification);
      expect(actual.notifications[0]).toHaveProperty('id');
      expect(typeof actual.notifications[0].id).toBe('number');
    });

    test('should handle multiple notifications', () => {
      let state = initialState;

      state = uiReducer(
        state,
        addNotification({
          type: 'info',
          message: 'First notification',
        })
      );

      state = uiReducer(
        state,
        addNotification({
          type: 'warning',
          message: 'Second notification',
        })
      );

      expect(state.notifications).toHaveLength(2);
      expect(state.notifications[0].message).toBe('First notification');
      expect(state.notifications[1].message).toBe('Second notification');
    });

    test('should handle removeNotification', () => {
      const stateWithNotifications = {
        ...initialState,
        notifications: [
          { id: 1, type: 'info', message: 'First' },
          { id: 2, type: 'warning', message: 'Second' },
          { id: 3, type: 'error', message: 'Third' },
        ],
      };

      const actual = uiReducer(stateWithNotifications, removeNotification(2));

      expect(actual.notifications).toHaveLength(2);
      expect(actual.notifications.find((n) => n.id === 2)).toBeUndefined();
      expect(actual.notifications.map((n) => n.id)).toEqual([1, 3]);
    });

    test('should handle clearNotifications', () => {
      const stateWithNotifications = {
        ...initialState,
        notifications: [
          { id: 1, type: 'info', message: 'First' },
          { id: 2, type: 'warning', message: 'Second' },
        ],
      };

      const actual = uiReducer(stateWithNotifications, clearNotifications());

      expect(actual.notifications).toEqual([]);
    });
  });

  describe('robot type list actions', () => {
    test('should handle setRobotTypeList', () => {
      const robotTypes = ['robot1', 'robot2', 'robot3'];
      const actual = uiReducer(initialState, setRobotTypeList(robotTypes));

      expect(actual.robotTypeList).toEqual(robotTypes);
    });

    test('should update existing robot type list', () => {
      const stateWithRobots = {
        ...initialState,
        robotTypeList: ['robot1', 'robot2'],
      };

      const newRobotTypes = ['robot3', 'robot4'];
      const actual = uiReducer(stateWithRobots, setRobotTypeList(newRobotTypes));

      expect(actual.robotTypeList).toEqual(newRobotTypes);
    });
  });

  test('should handle multiple actions in sequence', () => {
    let state = initialState;

    // Set loading state
    state = uiReducer(state, setLoading(true));

    // Move to record page
    state = uiReducer(state, moveToPage(PageType.RECORD));

    // Open sidebar
    state = uiReducer(state, setSidebarOpen(true));

    // Add notification
    state = uiReducer(
      state,
      addNotification({
        type: 'success',
        message: 'Page loaded successfully',
      })
    );

    // Set robot types
    state = uiReducer(state, setRobotTypeList(['robot1', 'robot2']));

    // Stop loading
    state = uiReducer(state, setLoading(false));

    expect(state.isLoading).toBe(false);
    expect(state.currentPage).toBe(PageType.RECORD);
    expect(state.isFirstLoad).toBe(false);
    expect(state.sidebarOpen).toBe(true);
    expect(state.notifications).toHaveLength(1);
    expect(state.robotTypeList).toEqual(['robot1', 'robot2']);
  });
});
