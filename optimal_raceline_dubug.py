import matplotlib.pyplot as plt
import csv
import os

def load_path(filename):
    """CSV 파일에서 x, y 좌표를 읽어오는 함수"""
    x = []
    y = []
    if not os.path.exists(filename):
        print(f"[경고] {filename} 파일이 없습니다.")
        return [], []
        
    with open(filename, 'r') as f:
        reader = csv.reader(f)
        # 헤더가 있는지 확인하고 건너뛰기
        header = next(reader, None)
        
        for row in reader:
            if not row: continue
            try:
                # 데이터 파싱 (x, y가 첫 번째, 두 번째 컬럼이라고 가정)
                x.append(float(row[0]))
                y.append(float(row[1]))
            except ValueError:
                continue
    return x, y

# 1. 두 개의 경로 데이터 로드
# C++ 코드에서 저장한 파일명과 일치해야 합니다.
file1 = 'path_1_centerline.csv'
file2 = 'path_2_optimized.csv'

x1, y1 = load_path(file1)
x2, y2 = load_path(file2)

# 데이터가 잘 로드되었는지 확인
if not x1 or not x2:
    print("데이터를 불러오지 못했습니다. 경로와 파일명을 확인해주세요.")
else:
    # 2. 그래프 그리기
    plt.figure(figsize=(12, 10))

    # (1) 중심선 그리기 (파란색 점선)
    plt.plot(x1, y1, color='blue', linestyle='--', linewidth=1, label='Centerline (Original)')

    # (2) 최적화된 경로 그리기 (빨간색 실선, 조금 더 두껍게)
    plt.plot(x2, y2, color='red', linestyle='-', linewidth=2, label='Optimized (Race Line)')

    # (3) 시작점 표시
    plt.scatter(x1[0], y1[0], color='green', s=100, label='Start Point', zorder=5)

    # 3. 그래프 스타일 설정
    plt.title(f"Raceline Optimization Comparison\n({file1} vs {file2})")
    plt.xlabel("World X (m)")
    plt.ylabel("World Y (m)")
    plt.axis('equal')  # 비율 고정 (트랙 모양 왜곡 방지)
    plt.grid(True, which='both', linestyle=':', alpha=0.6)
    plt.legend()

    # 4. 출력
    print(f"Centerline Points: {len(x1)}")
    print(f"Optimized Points: {len(x2)}")
    plt.show()