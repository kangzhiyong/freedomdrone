import msgpack
import numpy as np
def dumps(x, y, z):
    data = msgpack.dumps([x, y, z])
    print(data)
    print(np.array(data))
    # return np.array(data)
    return "xxxxx"

def add_num(a, b):
    return a+b