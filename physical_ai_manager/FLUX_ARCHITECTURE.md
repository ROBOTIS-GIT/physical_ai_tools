# Flux Architecture Refactoring

이 문서는 Physical AI Manager React 애플리케이션의 Flux 패턴 리팩토링에 대해 설명합니다.

## 개요

애플리케이션을 기존의 React hooks 기반 상태 관리에서 Flux 패턴으로 리팩토링하여 다음과 같은 이점을 얻었습니다:

- **단방향 데이터 플로우**: 예측 가능한 상태 변경
- **중앙 집중식 상태 관리**: 모든 상태가 스토어에서 관리됨
- **확장성**: 새로운 기능 추가가 용이함
- **테스트 용이성**: 각 레이어가 독립적으로 테스트 가능함

## Flux 패턴 구조

```
src/flux/
├── actions/
│   ├── ActionTypes.js      # 액션 타입 상수들
│   └── AppActions.js       # 액션 생성자들
├── dispatcher/
│   └── Dispatcher.js       # 중앙 디스패처
├── stores/
│   ├── BaseStore.js        # 스토어 기본 클래스
│   ├── AppStore.js         # 애플리케이션 메인 상태 스토어
│   └── TaskStore.js        # 태스크 관련 상태 스토어
├── hooks/
│   ├── useFluxStore.js     # Flux 스토어 연결 훅
│   ├── useAppStore.js      # AppStore 사용 훅
│   └── useTaskStore.js     # TaskStore 사용 훅
└── index.js                # Flux 모듈 메인 export

src/hooks/
├── useRosTaskStatus.js     # ROS 태스크 상태 관리 (기존)
└── useRosServiceCaller.js  # ROS 서비스 호출 (기존)
```

## 데이터 플로우

1. **View** (React 컴포넌트)에서 사용자 인터랙션 발생
2. **Action**이 생성되어 Dispatcher로 전송
3. **Dispatcher**가 모든 등록된 스토어에 액션 전달
4. **Store**가 액션을 처리하고 상태 업데이트
5. **Store**가 변경 이벤트 발생
6. **View**가 스토어의 변경사항을 감지하고 리렌더링

## 주요 변경사항

### 1. 상태 관리

**이전 (React hooks)**:

```javascript
const [currentPage, setCurrentPage] = useState('home');
const [topics, setTopics] = useState([null, null, null, null]);
```

**현재 (Flux stores)**:

```javascript
const { currentPage, topics } = useAppStore();
AppActions.navigateToPage('record');
AppActions.setTopics(newTopics);
```

### 2. ROS 통신

**기존 훅 활용**:

```javascript
// ROS 상태는 기존 훅 사용
const { taskStatus, taskInfo } = useRosTaskStatus(rosbridgeUrl, '/task/status');
const { sendRecordCommand } = useRosServiceCaller(rosbridgeUrl);

// Flux 스토어와 동기화
useEffect(() => {
  if (taskStatus) AppActions.setTaskStatus(taskStatus);
}, [taskStatus]);
```

### 3. 컴포넌트 props

**이전**:

```javascript
<HomePage
  topics={topics}
  setTopics={setTopics}
  rosHost={rosHost}
  currentRobotType={currentRobotType}
/>
```

**현재**:

```javascript
<HomePage />
// 앱 레벨 상태는 스토어에서, ROS 상태는 props로 전달
<RecordPage
  taskStatus={taskStatus}
  taskInfo={taskInfo}
  updateTaskInfo={updateTaskInfo}
/>
```

## 사용 방법

### 1. 앱 상태 읽기 (Flux)

```javascript
import { useAppStore } from './flux/hooks';

function MyComponent() {
  const { currentPage, rosHost, topics } = useAppStore();

  return (
    <div>
      <p>Current page: {currentPage}</p>
      <p>Topics: {topics.length}</p>
    </div>
  );
}
```

### 2. 앱 상태 변경 (Flux)

```javascript
import AppActions from './flux/actions/AppActions';

function MyComponent() {
  const handleNavigate = () => {
    AppActions.navigateToPage('record');
  };

  const handleTopicChange = (newTopics) => {
    AppActions.setTopics(newTopics);
  };

  return <button onClick={handleNavigate}>Go to Record Page</button>;
}
```

### 3. ROS 통신 (기존 훅)

```javascript
import { useRosServiceCaller } from './hooks/useRosServiceCaller';

function MyComponent() {
  const { sendRecordCommand } = useRosServiceCaller(rosbridgeUrl);

  const handleStart = async () => {
    try {
      const result = await sendRecordCommand('start_record', taskInfo);
      console.log('Command executed:', result);
    } catch (error) {
      console.error('Command failed:', error);
    }
  };

  return <button onClick={handleStart}>Start</button>;
}
```

## 스토어 구조

### AppStore

- 애플리케이션 레벨 상태 관리
- 네비게이션, ROS 호스트, 로봇 타입, 토픽, YAML 설정 등

### TaskStore

- ROS에서 받은 태스크 상태를 Flux 스토어에 동기화
- 기존 `useRosTaskStatus` 훅과 연동

## 설계 철학

### 전통적인 Flux 패턴 준수

- **Services 폴더 없음**: 전통적인 Flux에는 Services 레이어가 없음
- **기존 훅 활용**: 잘 작동하는 기존 ROS 훅을 그대로 사용
- **단순함**: 불필요한 추상화 없이 명확한 구조 유지

### 하이브리드 접근 방식

1. **애플리케이션 상태**: Flux 패턴으로 관리

   - 네비게이션, 설정, UI 상태 등

2. **ROS 통신 상태**: 기존 훅으로 관리

   - 연결 관리, 실시간 데이터, 서비스 호출 등

3. **동기화**: ROS 상태를 Flux 스토어와 동기화

## 마이그레이션 가이드

기존 컴포넌트를 이 패턴으로 마이그레이션하려면:

1. **앱 상태**: Flux 스토어 사용

   ```javascript
   const { currentPage, topics } = useAppStore();
   AppActions.setTopics(newTopics);
   ```

2. **ROS 상태**: 기존 훅 계속 사용

   ```javascript
   const { taskStatus } = useRosTaskStatus(rosbridgeUrl);
   const { sendRecordCommand } = useRosServiceCaller(rosbridgeUrl);
   ```

3. **동기화**: 필요시 ROS 상태를 Flux 스토어에 동기화
   ```javascript
   useEffect(() => {
     if (taskStatus) AppActions.setTaskStatus(taskStatus);
   }, [taskStatus]);
   ```

## 장점

1. **전통적인 Flux**: 표준 패턴을 따라 이해하기 쉬움
2. **기존 코드 활용**: 잘 작동하는 ROS 훅을 그대로 사용
3. **점진적 마이그레이션**: 필요한 부분만 Flux로 변환
4. **단순성**: 불필요한 복잡성 제거
5. **유지보수성**: 명확한 책임 분리

## 주의사항

1. 앱 상태 변경은 반드시 Flux 액션 사용
2. ROS 통신은 기존 훅 사용
3. 필요시에만 ROS 상태를 Flux 스토어와 동기화
4. Props drilling을 피하되, ROS 상태는 필요시 props로 전달 가능

이 접근 방식을 통해 전통적인 Flux 패턴의 장점을 얻으면서도 기존 코드를 최대한 활용할 수 있습니다.
