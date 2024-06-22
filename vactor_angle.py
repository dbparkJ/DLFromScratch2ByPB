import numpy as np
import matplotlib.pyplot as plt

# 벡터 a와 b 정의
a = np.array([2, 3])
b = np.array([4, 1])

# 벡터의 크기와 각도 계산
a_magnitude = np.linalg.norm(a)
b_magnitude = np.linalg.norm(b)
cos_theta = np.dot(a, b) / (a_magnitude * b_magnitude)
theta = np.arccos(cos_theta)

# 벡터 시각화
plt.figure()
plt.quiver(0, 0, a[0], a[1], angles='xy', scale_units='xy', scale=1, color='r', label='a')
plt.quiver(0, 0, b[0], b[1], angles='xy', scale_units='xy', scale=1, color='b', label='b')

# 벡터의 각도 표시
plt.text((a[0]+b[0])/2, (a[1]+b[1])/2, f"$\\theta = {np.degrees(theta):.2f}^\\circ$", fontsize=12, ha='center')

# 그래프 설정
plt.xlim(-1, 5)
plt.ylim(-1, 5)
plt.axhline(0, color='grey', lw=0.5)
plt.axvline(0, color='grey', lw=0.5)
plt.grid()
plt.legend()
plt.title('Vector a and b with angle θ')

# 그래프를 이미지 파일로 저장
plt.savefig("vector_angle.png")

# 그래프 표시
plt.show()
