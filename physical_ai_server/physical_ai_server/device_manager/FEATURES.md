# Device Manager Module - FEATURES

## Overview
Utility module for system resource (CPU, RAM, Storage) monitoring.

---

## Classes

### CPUChecker
**File**: `cpu_checker.py`

Moving average-based CPU usage monitoring.

#### Attributes
| Attribute | Type | Default | Description |
|-----------|------|---------|-------------|
| `window_size` | int | 30 | Moving average window size |
| `cpu_samples` | deque | - | CPU sample buffer |

#### Methods
| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `get_cpu_usage` | - | float | Get moving average CPU usage (%) |

#### Usage
```python
from physical_ai_server.device_manager.cpu_checker import CPUChecker

checker = CPUChecker(window_size=30)

# Call periodically for moving average
for _ in range(100):
    cpu_usage = checker.get_cpu_usage()
    print(f"CPU: {cpu_usage:.1f}%")
    time.sleep(0.1)
```

---

### RAMChecker
**File**: `ram_checker.py`

System RAM usage monitoring.

#### Static Methods
| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `get_ram_gb` | - | tuple[float, float] | (total RAM, used RAM) GB |
| `get_free_ram_gb` | - | float | Available RAM (GB) |

#### Usage
```python
from physical_ai_server.device_manager.ram_checker import RAMChecker

# Total and used RAM
total, used = RAMChecker.get_ram_gb()
print(f"RAM: {used:.1f}/{total:.1f} GB")

# Available RAM
free = RAMChecker.get_free_ram_gb()
print(f"Free RAM: {free:.1f} GB")

# RAM limit check in DataManager
if RAMChecker.get_free_ram_gb() < 2:  # RAM_LIMIT_GB
    print("Low memory warning!")
```

---

### StorageChecker
**File**: `storage_checker.py`

Disk storage usage monitoring.

#### Static Methods
| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `get_storage_gb` | path | tuple[float, float] | (total capacity, used) GB |

#### Usage
```python
from physical_ai_server.device_manager.storage_checker import StorageChecker

# Root partition storage
total, used = StorageChecker.get_storage_gb('/')
print(f"Storage: {used:.1f}/{total:.1f} GB")

# Specific path partition
total, used = StorageChecker.get_storage_gb('/data')
free = total - used
print(f"Available: {free:.1f} GB")
```

---

## Dependencies

### External
| Package | Components |
|---------|------------|
| `psutil` | CPU, RAM, disk info |

---

## Usage in DataManager

```python
# Example usage in data_manager.py

from physical_ai_server.device_manager.cpu_checker import CPUChecker
from physical_ai_server.device_manager.ram_checker import RAMChecker
from physical_ai_server.device_manager.storage_checker import StorageChecker

class DataManager:
    RAM_LIMIT_GB = 2  # Minimum required RAM

    def __init__(self):
        self._cpu_checker = CPUChecker()

    def record(self, images, state, action):
        # Auto-save on low RAM
        if RAMChecker.get_free_ram_gb() < self.RAM_LIMIT_GB:
            self.record_early_save()

    def get_current_record_status(self):
        status = TaskStatus()

        # CPU usage (moving average)
        status.used_cpu = float(self._cpu_checker.get_cpu_usage())

        # RAM usage
        ram_total, ram_used = RAMChecker.get_ram_gb()
        status.used_ram_size = float(ram_used)
        status.total_ram_size = float(ram_total)

        # Storage usage
        total_storage, used_storage = StorageChecker.get_storage_gb('/')
        status.used_storage_size = float(used_storage)
        status.total_storage_size = float(total_storage)

        return status
```

---

## Notes
- CPUChecker calls `psutil.cpu_percent(interval=None)` on initialization
- Moving average smooths noisy CPU measurements
- All checkers return 0.0 on exception (fail-safe)
- RAM check used as auto-save trigger in DataManager
