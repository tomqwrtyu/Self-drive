import pickle
import h5py
import os
import numpy as np

def input_data_generator(data_dir:list, batch_size:int=4, out_seperate:bool=False, no_rnn_state:bool=False):
  if isinstance(data_dir, str):
      data_dir = [data_dir]

  total_size = 0
  total_size += len(data_dir) - 1 # adding file bridge
  for d in data_dir:
      with open(d + "/fake_y.pickle", "rb") as fy:
        total_size += len(pickle.load(fy))
        
  def gen(data_dir:list, batch_size:int, out_seperate:bool, no_rnn_state:bool):
    nonlocal total_size
    presicion = np.float32
    Ximgs  = np.zeros((batch_size, 12, 128, 256), dtype=presicion)   # Is YUV input img uint8? No. See float8 ysf = convert_float8(ys) in loadyuv.cl
    Xin1   = np.zeros((batch_size, 8), dtype=presicion)     # DESIRE_LEN = 8
    Xin2   = np.zeros((batch_size, 2), dtype=presicion)     # TRAFFIC_CONVENTION_LEN = 2
    Xin3   = np.zeros((batch_size, 512), dtype=presicion)   # rnn state
    Ytrue  = None

    # Xin1[:, 0] = 1.0   # go straight? desire_state_prob[0] = 1.0
    Xin2[:, 0] = 1.0   # traffic_convention[0] = 1.0 = left hand drive like in Taiwan

    dir_size = len(data_dir)
    max_steps = total_size // batch_size
    steps = 0
    last_YUV = None
    
    while True:
      data_from_prev_dir_count = 0
      
      for didx, d in enumerate(data_dir):
        with h5py.File(d + "/yuv.h5", "r") as yuv, open(d + "/fake_y.pickle", "rb") as fy:
          fake_y = pickle.load(fy)
          yuvX = yuv['X']
          data_len = len(fake_y)
          start = 0

          if didx != 0: # Add data for bridging files
            with open(data_dir[didx - 1] + "/file_bridge.pickle", "rb") as ff:
              outs = pickle.load(ff).get(d, None)

            if outs is None:
              raise ValueError(f"No suitable bridge between {data_dir[didx - 1]} and {d}, terminated.")
            
            Ximgs[data_from_prev_dir_count] = np.vstack((last_YUV, yuvX[0]))
            Xin3[data_from_prev_dir_count] = outs[-1][0]

            if data_from_prev_dir_count == 0:
              Ytrue = outs
            else:
              for k in range(len(Ytrue)):
                Ytrue[k] = np.vstack([Ytrue[k], outs[k]])

            data_from_prev_dir_count += 1 # the bridge data

          else:
            Xin3_temp = fake_y[0][-1][0]

          # Previous file left some data that cannot form a batch
          if data_from_prev_dir_count != 0:
            start = batch_size - data_from_prev_dir_count

            for j in range(data_from_prev_dir_count, batch_size):
              Ximgs[j] = np.vstack((yuvX[j  - data_from_prev_dir_count], yuvX[j  - data_from_prev_dir_count + 1]))
              Xin3[j] = Xin3_temp

              # if j != 0:
              for k in range(len(Ytrue)):
                Ytrue[k] = np.vstack([Ytrue[k], fake_y[j - data_from_prev_dir_count][k]])

              Xin3_temp = fake_y[j][-1][0]

            steps += 1
            if not no_rnn_state:
              yield [Ximgs, Xin1, Xin2, Xin3], Ytrue if out_seperate else np.hstack(Ytrue)
            else:
              yield [Ximgs, Xin1, Xin2], Ytrue[:-1] if out_seperate else np.hstack(Ytrue[:-1])

          for i in range(start, data_len, batch_size):       
            end = batch_size

            if data_len - i < batch_size:
              if didx != dir_size - 1:
                data_from_prev_dir_count = data_len - i
                end = data_len - i
              else:
                continue # skip rest data

            for j in range(end):
              Ximgs[j] = np.vstack((yuvX[(i + j)], yuvX[(i + j) + 1]))
              Xin3[j] = Xin3_temp

              if j != 0:
                for k in range(len(Ytrue)):
                  Ytrue[k] = np.vstack([Ytrue[k], fake_y[i + j][k]])
              else:
                Ytrue = fake_y[i]

              Xin3_temp = fake_y[i + j][-1][0]

            if end == batch_size:
              steps += 1
              if not no_rnn_state:
                yield [Ximgs, Xin1, Xin2, Xin3], Ytrue if out_seperate else np.hstack(Ytrue)
              else:
                yield [Ximgs, Xin1, Xin2], Ytrue[:-1] if out_seperate else np.hstack(Ytrue[:-1])

              if steps >= max_steps:
                steps = 0

          last_YUV = yuvX[-1]

  return total_size, gen(data_dir, batch_size, out_seperate, no_rnn_state)

def cat_input_data_generator(data_dir:list, batch_size:int=4):
    if isinstance(data_dir, str):
        data_dir = [data_dir]

    total_size = 0
    for d in data_dir:
        with open(d + "/fake_cat_y.pickle", "rb") as fy:
            total_size += len(pickle.load(fy))
        
    def gen(data_dir:list, batch_size:int):
        nonlocal total_size
        presicion = np.float32
        Ximg0  = np.zeros((batch_size, 6, 128, 256), dtype=presicion)   # Is YUV input img uint8? No. See float8 ysf = convert_float8(ys) in loadyuv.cl
        Ximg1  = np.zeros((batch_size, 6, 128, 256), dtype=presicion)
        Xin1   = np.zeros((batch_size, 8), dtype=presicion)     # DESIRE_LEN = 8
        Xin2   = np.zeros((batch_size, 2), dtype=presicion)     # TRAFFIC_CONVENTION_LEN = 2
        Ytrue  = None

        # Xin1[:, 0] = 1.0   # go straight? desire_state_prob[0] = 1.0
        Xin2[:, 0] = 1.0   # traffic_convention[0] = 1.0 = left hand drive like in Taiwan

        dir_size = len(data_dir)
        max_steps = total_size // batch_size
        steps = 0
        
        while True:
            data_from_prev_dir_count = 0
            
            for didx, d in enumerate(data_dir):
                with h5py.File(d + "/yuv.h5", "r") as yuv, open(d + "/fake_cat_y.pickle", "rb") as fy:
                    fake_y = pickle.load(fy)
                    yuvX = yuv['X']
                    data_len = len(fake_y)
                    start = 0

                    # Previous file left some data that cannot form a batch
                    if data_from_prev_dir_count != 0:
                        start = batch_size - data_from_prev_dir_count

                        for j in range(data_from_prev_dir_count, batch_size):
                            Ximg0[j] = yuvX[j - data_from_prev_dir_count]
                            Ximg1[j] = yuvX[j - data_from_prev_dir_count + 1]

                            for k in range(len(Ytrue)):
                                Ytrue[k] = np.vstack([Ytrue[k], fake_y[j - data_from_prev_dir_count][k]])

                        steps += 1
                        yield [Ximg0, Ximg1, Xin1, Xin2], np.hstack(Ytrue)

                    for i in range(start, data_len, batch_size):

                        end = batch_size 
                        if data_len - i < batch_size:
                            if didx != dir_size - 1:
                                data_from_prev_dir_count = data_len - i
                                end = data_len - i
                            else:
                                continue # skip rest data

                        for j in range(end):
                            Ximg0[j] = yuvX[i + j]
                            Ximg1[j] = yuvX[i + j + 1]

                            if j != 0:
                                for k in range(len(Ytrue)):
                                    Ytrue[k] = np.vstack([Ytrue[k], fake_y[i + j][k]])
                            else:
                                Ytrue = fake_y[i]


                        if end == batch_size:
                            steps += 1
                            yield [Ximg0, Ximg1, Xin1, Xin2], np.hstack(Ytrue)

                        if steps >= max_steps:
                            steps = 0

    return total_size, gen(data_dir, batch_size)