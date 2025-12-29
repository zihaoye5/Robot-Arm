import numpy as np
import matplotlib.pyplot as plt

# 定义 alignment_cos 的取值范围
alignment_cos = np.linspace(-1.0, 2.0, 1000)

# 定义平滑评分函数
smooth_score = 2 * (1 / (1 + np.exp(-20 * (alignment_cos - 0.866)))) - 1

# 绘图
plt.figure()
plt.plot(alignment_cos, smooth_score)
plt.xlabel("alignment_cos")
plt.ylabel("smooth_score")
plt.title("Smooth Sigmoid-based Alignment Score")
plt.grid(True)

plt.show()
