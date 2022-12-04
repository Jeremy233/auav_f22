# from multiprocessing import Process, Queue, Pipe
# from sub_rgbd_image import f

# if __name__ == "__main__":
# 	parent_conn, child_conn = Pipe()
# 	p = Process(target=f, args=(child_conn))
# 	p.start
# 	while True:
# 		print(parent_conn.recv())

import pickle
import numpy as np

if __name__ == "__main__":
	while True:	
		fp = open("shared.npy", "rb")
		shared = np.load(fp)
		print(shared.shape)
	