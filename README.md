# UZH-FPV `indoor_forward_9_davis` — VINS-Mono / VINS-Mono+LC

`indoor_forward_9_davis_with_gt.bag`에 대해 VINS-Mono(루프 클로저 없음)와
VINS-Mono+LC(루프 클로저 있음)를 Docker로 돌리고, ground truth 대비 오차와
3D 궤적 그래프를 얻기 위한 파이프라인입니다.

**실행은 amd64 리눅스 서버에서** 하는 것을 기준으로 합니다 (ROS Kinetic 기반
공식 VINS-Mono Docker 이미지를 그대로 사용).

## 폴더 구조

```
data/
  indoor_forward_9_davis_with_gt.bag             # git-lfs로 추적 (repo에 포함됨)
  indoor_forward_9_davis_with_gt/                # bag을 풀어놓은 사본 (git 추적 안 함, .gitignore)
    groundtruth.txt                              # evaluate.py가 이 파일을 사용함
vins_ws/VINS-Mono/                     # VINS-Mono 공식 저장소 (git submodule, 아래 0번 참고)
  docker/Dockerfile                    # 공식 Dockerfile (ros:kinetic-perception, Ceres 1.12.0), 수정 안 함
config/
  uzh_fpv_davis_no_loop.yaml           # VINS-Mono (loop_closure: 0)
  uzh_fpv_davis_loop.yaml              # VINS-Mono+LC (loop_closure: 1)
scripts/
  run_vins.sh                          # 컨테이너 안에서 roscore+roslaunch+rosbag play 실행
  evaluate.py                          # 호스트에서 evo로 GT 대비 오차/3D 궤적 그래프 생성
docker-compose.yml                     # 빌드 + 볼륨 마운트 정의
output/                                # (자동 생성, git 추적 안 함) VINS-Mono 결과 CSV
results/                               # (자동 생성, git 추적 안 함) 3D 궤적 그래프, 오차 요약 CSV
```

`vins_ws/VINS-Mono/`는 git submodule로 연결되어 있고, `data/indoor_forward_9_davis_with_gt/`
(ASCII 사본)는 `.bag` 안에 이미 들어있는 내용의 중복이라(`events.txt`만
960MB) git 추적에서 제외했습니다. 새로 clone한 서버에서는 최초 1회 아래처럼
준비하세요:

## 0. 최초 설정 (새로 clone한 서버에서)

```bash
git clone --recurse-submodules https://github.com/sungho2574/uzh-fpv-vio.git
# 이미 --recurse-submodules 없이 clone했다면:
git submodule update --init --recursive
```

`data/indoor_forward_9_davis_with_gt/groundtruth.txt`는 evaluate.py 실행에
필요합니다. bag을 직접 풀거나(`rosbag`으로 groundtruth 관련 토픽을 export),
원본 UZH-FPV 데이터셋 페이지(https://fpv.ifi.uzh.ch/datasets/)에서
`indoor_forward_9_davis_with_gt.txt` 계열 ground truth 파일을 받아 같은
경로에 두면 됩니다.

## 1. 이미지 빌드

```bash
docker compose build
```

`vins_ws/VINS-Mono/docker/Dockerfile`을 그대로 사용해서 ROS Kinetic + Ceres
1.12.0 + VINS-Mono를 빌드합니다.

## 2. VINS-Mono 실행 (두 번)

`docker compose run`은 **매번 새 컨테이너를 띄워서 안에서 명령을 실행하고,
끝나면 컨테이너를 종료**합니다. `data/`, `config/`, `output/`, `scripts/`는
`docker-compose.yml`에 정의된 바인드 마운트라서, 컨테이너가 종료돼도 결과
파일은 호스트의 `output/`에 그대로 남습니다.

```bash
# VINS-Mono (loop closure 없음)
docker compose run vins-mono bash /root/scripts/run_vins.sh no_loop

# VINS-Mono+LC (loop closure 있음)
docker compose run vins-mono bash /root/scripts/run_vins.sh loop
```

각 실행은 bag(~78초)을 기본적으로 실시간(`--rate 1.0`)으로 재생합니다.
서버 성능이 부족해서 실시간 처리가 밀리면 속도를 낮출 수 있습니다:

```bash
docker compose run vins-mono bash /root/scripts/run_vins.sh no_loop 0.5
```

완료되면 다음 파일이 생성됩니다:
- `output/no_loop/vins_result_no_loop.csv`
- `output/loop/vins_result_no_loop.csv` (loop 실행에서도 같이 생성됨, 참고용)
- `output/loop/vins_result_loop.csv`

컨테이너 안에 직접 들어가서 확인하고 싶다면:
```bash
docker compose run vins-mono bash
```

## 3. 오차 평가 + 3D 궤적 그래프 (호스트에서 실행)

`evo`는 Python ≥3.10이 필요해서 ROS Kinetic 컨테이너(Python 2.7) 밖,
즉 서버 호스트에서 별도 venv로 돌립니다.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install evo numpy pandas matplotlib

python3 scripts/evaluate.py \
  --no-loop-csv output/no_loop/vins_result_no_loop.csv \
  --loop-csv output/loop/vins_result_loop.csv \
  --gt data/indoor_forward_9_davis_with_gt/groundtruth.txt \
  --out results
```

결과:
- `results/trajectory_3d.png` — GT + VINS-Mono + VINS-Mono+LC 3D 궤적 비교
- `results/vins_mono_ape.png`, `results/vins_mono_lc_ape.png` — 절대 위치 오차(APE) 시각화
- `results/vins_mono_rpe.png`, `results/vins_mono_lc_rpe.png` — 상대 위치 오차(RPE) 시각화
- `results/*.zip` — evo의 raw 결과 (stats.json 포함)
- `results/summary.csv` — 두 방식의 APE/RPE RMSE·mean·median·std 요약 비교표
- `results/tum/*.tum` — VINS-Mono CSV를 TUM 포맷으로 변환한 중간 결과

## 참고사항

- Ground truth는 전체 비행(~77.6초) 중 모캡 볼륨 안에 있던 중간 ~28.8초
  구간만 있습니다. 오차 지표는 이 구간에 대해서만 계산됩니다.
- 첫 실행에서 궤적이 발산하거나 이상하게 나오면 카메라 왜곡 계수 순서나
  카메라-IMU 외부 파라미터 부호를 의심해보세요 (`config/*.yaml` 상단 주석에
  캘리브레이션 출처가 적혀 있습니다).
