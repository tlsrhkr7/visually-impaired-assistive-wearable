# 시각장애인 보조 웨어러블 — SLAM 모듈

ROS2 Humble + ORB-SLAM3 IMU_RGBD, Jetson Orin + Intel RealSense D435I

**목표:** 한 공간을 매핑해서 맵을 저장하고, 나중에 켜도 그 맵으로 위치 추정(localization)만 하는 시스템

---

## 하드웨어

- **플랫폼:** Jetson Orin (aarch64, L4T R36.5.0, Linux 5.15.185-tegra)
- **카메라:** Intel RealSense D435I (USB 3.0 포트 필수)
  - RGB 640x480 @ 30fps
  - Depth 640x480 @ 30fps
  - IMU 200Hz (가속도계 + 자이로, HID/IIO 서브시스템)

---

## 현재 상태 (2026-05-09)

### 완료된 것
- IMU_RGBD 모드 구현 완료 (ORB-SLAM3 sensor type 5)
- RGB + Depth `ApproximateTimeSynchronizer`로 동기화
- IMU 하드웨어 보정 완료 (값 EEPROM에 저장, config에 반영)
- Atlas 저장/불러오기 구현 (`maps/d435i_map`)
- `localization_mode` launch 인자 — 저장된 맵 불러와서 위치 추정만 수행
- Sophus NaN 크래시 수정 (abort → 잡히는 예외로 변환)
- 트래킹 워치독: 60프레임 연속 실패 시 자동 리셋
- IIO udev 규칙으로 재연결 시 IMU 권한 자동 수정
- IMU BA2 초기화 완료 후 RViz에 포인트클라우드 표시
- HID sensor 커널 모듈 빌드 및 systemd 서비스 등록 (재부팅 시 자동 로드)

### 미완료 / 알려진 문제
- 루프 클로징 비활성화 상태 (`loopClosing: 0`)
  - **원인:** 루프 클로징 후 타임스탬프 점프 → `CreateMapInAtlas()` → IMU 버퍼 비어있음 → `mpImuPreintegratedFrame`이 null → `PredictStateIMU()` 에서 null 역참조 → SIGSEGV
  - **수정 방법:** `ORB_SLAM3/src/Tracking.cc`의 `PredictStateIMU()` 함수 1754번째, 1780번째 줄 앞에 null 체크 추가 후 ORB-SLAM3 라이브러리 재빌드
  - **영향:** 큰 공간이나 루프 경로에서 드리프트(누적 오차) 발생

---

## 켤 때마다 해야 하는 세팅

재부팅 후 HID sensor 모듈은 systemd가 자동으로 로드합니다.
**IIO 권한만 수동으로 적용**하면 됩니다:

```bash
# 1. IIO 권한 적용 (IMU 접근 허용)
sudo /usr/local/bin/fix-iio-perms.sh /sys/bus/iio/devices/iio:device0
sudo /usr/local/bin/fix-iio-perms.sh /sys/bus/iio/devices/iio:device1

# 2. ROS2 워크스페이스 소싱
source /opt/ros/humble/setup.bash
source /home/a/ros2_ws/install/setup.bash

# 3. 토픽 정상 수신 확인 (launch 후)
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/depth/image_rect_raw
ros2 topic hz /camera/camera/imu
```

**로그에 `No HID info provided, IMU is disabled` 뜨면:**
- USB 3.0 포트(파란색)에 꽂혀 있는지 확인
- IIO 권한 수동 적용 후 launch 재시작

**udev가 안 먹힐 때:**
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=iio
```

---

## 실행 방법

### 매핑 (새 맵 만들기)

```bash
ros2 launch orb_slam3_ros2 mapping.launch.py
```

- 실행 후 카메라를 **15초 이상 천천히 움직여야** IMU 초기화 완료
- 로그에 `Inertial BA1 complete` → `Inertial BA2 complete` 뜨면 포인트클라우드 나옴
- **Ctrl+C로 종료** → 맵이 `maps/d435i_map`에 자동 저장

### 로컬라이제이션 (저장된 맵으로 위치 추정)

```bash
ros2 launch orb_slam3_ros2 mapping.launch.py localization_mode:=true
```

### 로스백 녹화 (나중에 오프라인 매핑용)

```bash
ros2 launch orb_slam3_ros2 mapping.launch.py record_bag:=true
```

`bags/ORB_SLAM3_YYYY-MM-DD_HH-MM-SS/`에 저장됨. 녹화 토픽:
- `/camera/camera/imu`
- `/camera/camera/color/image_raw`
- `/camera/camera/depth/image_rect_raw`

### 로스백 재생

```bash
ros2 launch orb_slam3_ros2 mapping.launch.py playback_bag:=<백폴더이름>
```

---

## IMU 초기화 조건

| 단계 | 조건 | 결과 |
|------|------|------|
| BA1 | 5초 이상 움직임 | 거친 IMU 초기화 |
| BA2 | 15초 이상 움직임 | 완전 초기화, 포인트클라우드 표시 |
| 리셋 | 처음 10초 내 2cm 이하 이동 | 리셋 후 재시도 |

**실행 직후 카메라를 최소 15초 이상 움직여야 합니다.**

---

## 맵 저장 위치

```
maps/d435i_map
```

`config/RGBD-Inertial/RealSense_D435i.yaml`의 `System.SaveAtlasToFile` 경로.

**Ctrl+C 정상 종료 시에만 저장됨.** SIGKILL이나 크래시로 죽으면 저장 안 됨.

---

## 설정 파일

| 파일 | 용도 |
|------|------|
| `config/RGBD-Inertial/RealSense_D435i.yaml` | 매핑용 메인 설정 |
| `config/RGBD-Inertial/RealSense_D435i_localization.yaml` | 로컬라이제이션용 설정 (저장된 맵 불러옴) |

주요 파라미터:
```yaml
loopClosing: 0              # 비활성화 상태 — Tracking.cc null ptr 버그 수정 후 1로 변경
IMU.Frequency: 200.0
RGBD.DepthMapFactor: 1000.0  # D435I depth 단위는 mm
System.SaveAtlasToFile: "/home/a/ros2_ws/src/ORB_SLAM3_ROS2/maps/d435i_map"
```

---

## IIO udev 규칙 (IMU 권한 자동 수정)

파일: `/etc/udev/rules.d/99-hid-sensor-iio.rules`
```
SUBSYSTEM=="iio", ACTION=="add", ATTR{name}=="accel_3d", RUN+="/usr/local/bin/fix-iio-perms.sh /sys%p"
SUBSYSTEM=="iio", ACTION=="add", ATTR{name}=="gyro_3d",  RUN+="/usr/local/bin/fix-iio-perms.sh /sys%p"
```

D435I 꽂을 때 자동으로 실행됨. 안 되면:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=iio
```

---

## 빌드

```bash
cd /home/a/ros2_ws
colcon build --packages-select orb_slam3_ros2
source install/setup.bash
```

`ORB_SLAM3` 라이브러리 자체를 수정했을 때 (예: `Tracking.cc`):
```bash
cd /home/a/ros2_ws/src/ORB_SLAM3_ROS2/ORB_SLAM3
make -j2
cd /home/a/ros2_ws
colcon build --packages-select orb_slam3_ros2
source install/setup.bash
```

---

## 다른 머신에서 처음 셋업할 때 (성호 노트북 등)

이 repo는 `ORB_SLAM3`를 **너 fork(`tlsrhkr7/ORB_SLAM3`)에서 서브모듈로** 가져옴 (Jetson SIGSEGV 수정 포함).
경로는 `setup.sh`가 clone 위치에 맞춰 자동 조정.

### 의존성 (Ubuntu 22.04)

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-imu-filter-madgwick \
  ros-humble-realsense2-camera \
  ros-humble-realsense2-description \
  librealsense2-dev librealsense2-utils \
  libeigen3-dev libopencv-dev libpangolin-dev \
  libssl-dev libboost-all-dev libglew-dev \
  python3-colcon-common-extensions \
  build-essential cmake git
```

`libpangolin-dev`가 apt에 없으면 소스 빌드: https://github.com/stevenlovegrove/Pangolin

### 클론 → 셋업 → 빌드

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

# 1. clone (--recursive로 서브모듈 같이)
git clone --recursive https://github.com/tlsrhkr7/visually-impaired-assistive-wearable.git
cd visually-impaired-assistive-wearable

# 2. setup.sh 한 번 실행
#    - 서브모듈 초기화 (혹시 누락된 경우)
#    - YAML의 /home/a/... 경로를 현재 클론 위치 기준으로 치환
#    - maps/ 디렉토리 생성
#    - ORB_SLAM3 라이브러리 빌드 (10~20분)
./setup.sh

# 3. ROS2 패키지 빌드
cd ~/ros2_ws
colcon build --packages-select orb_slam3_ros2
source install/setup.bash
```

### 매핑 → 맵 파일 → 다른 머신 로컬라이제이션

매핑하면 `~/ros2_ws/src/visually-impaired-assistive-wearable/maps/d435i_map.osa`에 저장됨.
이 파일 하나를 다른 머신의 같은 경로에 복사하면 그 머신에서 로컬라이제이션 가능:

```bash
ros2 launch orb_slam3_ros2 mapping.launch.py localization_mode:=true
```

### 호환성 주의

- ORB_SLAM3 라이브러리 빌드 버전이 양쪽에서 같아야 atlas 파일이 호환됨 (이 repo + fork 사용하면 자동 보장)
- x86_64 ↔ aarch64 atlas 파일 호환은 이론상 OK이지만 보장 못 함 → 작은 맵으로 한 번 테스트 후 본격 매핑 권장
