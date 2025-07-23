# Inference Manager Refactoring

이 문서는 inference 관리자의 새로운 구조와 사용법을 설명합니다.

## 개요

기존의 `InferenceManager`는 LeRobot 정책에만 특화되어 있었습니다. 새로운 구조에서는:

1. **`InferenceBase`**: 모든 inference 관리자가 구현해야 하는 추상 기본 클래스
2. **`LeRobotInference`**: LeRobot 정책을 위한 구체적인 구현
3. **`PhysicalIntelligenceInference`**: Physical Intelligence 정책을 위한 구현 (템플릿)
4. **`GrootInference`**: NVIDIA GR00T 정책을 위한 구현 (템플릿)
5. **`InferenceFactory`**: 다양한 inference 관리자를 생성하는 팩토리 클래스

## 사용법

### 기본 사용법

```python
from physical_ai_server.inference import InferenceFactory

# LeRobot inference 관리자 생성
lerobot_manager = InferenceFactory.create_inference_manager('lerobot')

# Physical Intelligence inference 관리자 생성
pi_manager = InferenceFactory.create_inference_manager('physical_intelligence')

# GR00T inference 관리자 생성
groot_manager = InferenceFactory.create_inference_manager('groot')
```

### 편의 함수 사용

```python
from physical_ai_server.inference import create_inference_manager

# 기본적으로 LeRobot 사용
manager = create_inference_manager()

# 또는 특정 프레임워크 지정
pi_manager = create_inference_manager('physical_intelligence')
```

### 정책 로드 및 사용

```python
# 정책 검증
is_valid, message = manager.validate_policy('/path/to/policy')
if is_valid:
    print(f"Policy validation successful: {message}")
    
    # 정책 로드
    if manager.load_policy():
        print("Policy loaded successfully")
        
        # 추론 수행
        action = manager.predict(
            images={'camera1': image_array},
            state=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
            task_instruction="pick up the red cube"
        )
        print(f"Predicted action: {action}")
        
        # 청크 기반 추론 (여러 스텝)
        action_chunk = manager.predict_chunk(
            images={'camera1': image_array},
            state=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
            task_instruction="pick up the red cube"
        )
        print(f"Predicted action chunk shape: {action_chunk.shape}")
else:
    print(f"Policy validation failed: {message}")
```

### 사용 가능한 프레임워크 및 정책 조회

```python
from physical_ai_server.inference import InferenceFactory

# 사용 가능한 프레임워크 목록
frameworks = InferenceFactory.get_available_frameworks()
print(f"Available frameworks: {frameworks}")

# 모든 프레임워크의 사용 가능한 정책 조회
all_policies = InferenceFactory.get_all_available_policies()
for framework, policies in all_policies.items():
    print(f"{framework}: {policies}")

# 저장된 정책 조회
saved_policies = InferenceFactory.get_all_saved_policies()
for framework, (paths, types) in saved_policies.items():
    print(f"{framework} saved policies: {len(paths)} found")
```

### 자동 프레임워크 감지

```python
# 정책 경로에서 프레임워크 자동 감지
framework = InferenceFactory.detect_framework_from_path('/path/to/policy')
if framework:
    print(f"Detected framework: {framework}")
    manager = InferenceFactory.create_inference_manager(framework)
else:
    print("Could not detect framework")
```

## 하위 호환성

기존 코드와의 호환성을 위해 `InferenceManager`는 여전히 사용할 수 있습니다:

```python
# 기존 방식 (여전히 작동함)
from physical_ai_server.inference import InferenceManager
manager = InferenceManager()
```

이는 내부적으로 `LeRobotInference`와 동일합니다.

## 새로운 프레임워크 추가

새로운 AI 프레임워크를 지원하려면:

1. `InferenceBase`를 상속받는 새 클래스 생성
2. 모든 추상 메서드 구현
3. 팩토리에 등록

```python
from physical_ai_server.inference import InferenceBase, InferenceFactory

class MyCustomInference(InferenceBase):
    def validate_policy(self, policy_path: str):
        # 구현
        pass
    
    def load_policy(self):
        # 구현
        pass
    
    # 다른 메서드들도 구현...

# 팩토리에 등록
InferenceFactory.register_inference_manager('my_framework', MyCustomInference)

# 사용
manager = InferenceFactory.create_inference_manager('my_framework')
```

## 장점

1. **확장성**: 새로운 AI 프레임워크를 쉽게 추가할 수 있음
2. **유연성**: 각 프레임워크에 특화된 최적화 가능
3. **일관성**: 모든 프레임워크가 동일한 인터페이스 사용
4. **하위 호환성**: 기존 코드 수정 없이 사용 가능
5. **모듈성**: 필요한 프레임워크만 임포트 가능

## 주의사항

- `PhysicalIntelligenceInference`와 `GrootInference`는 현재 템플릿 구현입니다
- 실제 사용하려면 해당 프레임워크의 라이브러리를 설치하고 구현을 완성해야 합니다
- 각 프레임워크마다 다른 의존성이 필요할 수 있습니다
