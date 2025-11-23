import matplotlib.pyplot as plt
import csv
import numpy as np

filename = 'raceline_with_speed.csv'
x, y, v = [], [], []

try:
    with open(filename, 'r') as f:
        reader = csv.reader(f)
        next(reader)
        for row in reader:
            if not row: continue
            try:
                x.append(float(row[0]))
                y.append(float(row[1]))
                v.append(float(row[2])) # 3번째 컬럼이 속도
            except: continue
            
    # 산점도(Scatter Plot)로 속도 시각화
    plt.figure(figsize=(12, 10))
    # c=v: 속도값에 따라 색상 변경, cmap='jet': 파랑(느림)~빨강(빠름)
    sc = plt.scatter(x, y, c=v, cmap='jet', s=10) 
    plt.colorbar(sc, label='Speed (m/s)')
    
    plt.title(f"Raceline Velocity Profile\n(Max: {max(v):.2f} m/s, Min: {min(v):.2f} m/s)")
    plt.axis('equal')
    plt.grid(True)
    plt.show()

except Exception as e:
    print("Error:", e)