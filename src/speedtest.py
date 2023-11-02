import numpy as np
from collections import deque
import time


np_list = np.zeros((100, ))
start = time.time()
for i in range(1000):
    np_list[0:99] = np_list[1:100]
    np_list[99] = i
end = time.time()
print(end-start)
deq = deque([0] * 100)
start = time.time()
for i in range(1000):
    deq.popleft()
    deq.append(i)
end = time.time()
print(end-start)

initial_np = np.array([[10, 12], [11, 12]])
ans_np = np.repeat(initial_np, 10, axis = 0).reshape(np.shape(initial_np)[0], -1, 2)
print(ans_np)

x, y = [1, 2]