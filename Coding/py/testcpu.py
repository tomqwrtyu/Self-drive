# Test computer processing speed with a simple Python script
from multiprocessing import Pool, cpu_count
from datetime import datetime
import numpy as np

def func1(args):
    cpu, value = args
    start_time = datetime.now()
    for i in range(value):
        value = value * i
    print(f"cpu: {cpu} time: {datetime.now() - start_time}")

def func2(args):
    cpu, value = args
    start_time = datetime.now()
    n = value
    a = np.ones(n, dtype = np.float64)
    b = np.ones(n, dtype = np.float32)
    for i in range(n):
        a[i]+= 1
    print(f"cpu: {cpu} time: {datetime.now() - start_time}")
'''
'''

if __name__ == '__main__':
    start_time = datetime.now()
    cpu_count = cpu_count()
    with Pool(cpu_count) as mp_pool:
        mp_pool.map(func1, [(cpu, 100000000) for cpu in range(cpu_count)])
        #mp_pool.map(func2, [(cpu, 10000000) for cpu in range(cpu_count)])
    print(f"total: {datetime.now() - start_time}")
'''
--- on VM-Ubuntu 20.04, 2021.5.31
jinn@liu:~$ python3 testp.py
--- func1
cpu: 2 time: 0:00:12.851317
cpu: 0 time: 0:00:12.857562
cpu: 1 time: 0:00:13.067250
cpu: 3 time: 0:00:13.358835
total: 0:00:13.481573

--- func2
cpu: 1 time: 0:00:15.343026
cpu: 0 time: 0:00:16.033291
cpu: 2 time: 0:00:16.766987
cpu: 3 time: 0:00:17.297938
total: 0:00:17.340200

--- func1
cpu: 0 time: 0:00:15.338024
cpu: 7 time: 0:00:15.342162
cpu: 3 time: 0:00:15.392249
cpu: 4 time: 0:00:15.395545
cpu: 5 time: 0:00:15.403318
cpu: 6 time: 0:00:15.416429
cpu: 2 time: 0:00:15.435696
cpu: 1 time: 0:00:15.471469
total: 0:00:15.529418

--- func2
cpu: 2 time: 0:00:25.639947
cpu: 7 time: 0:00:25.645879
cpu: 4 time: 0:00:25.743383
cpu: 5 time: 0:00:25.870723
cpu: 3 time: 0:00:25.902692
cpu: 6 time: 0:00:25.888276
cpu: 1 time: 0:00:26.145605
cpu: 0 time: 0:00:26.162983
total: 0:00:26.217074
--- even worse with 8 cores?

--- on Ubuntu 16.04, 2021.5.31
jinn@Liu:~/Zothers$ python testcpu.py
--- func1
cpu: 6 time: 0:00:08.660149
cpu: 1 time: 0:00:08.663551
cpu: 4 time: 0:00:08.664745
cpu: 2 time: 0:00:08.667490
cpu: 0 time: 0:00:08.667975
cpu: 5 time: 0:00:08.670892
cpu: 3 time: 0:00:08.675839
cpu: 7 time: 0:00:08.676109
total: 0:00:08.690582

--- func2
cpu: 0 time: 0:00:07.658340
cpu: 4 time: 0:00:07.723914
cpu: 7 time: 0:00:07.792990
cpu: 5 time: 0:00:07.807492
cpu: 6 time: 0:00:07.814357
cpu: 2 time: 0:00:07.832937
cpu: 3 time: 0:00:07.840193
cpu: 1 time: 0:00:07.847434
total: 0:00:07.862683
--- it got 8 cores!
'''
