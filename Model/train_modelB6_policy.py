"""   YPL, YJW, JLL, 2021.9.8 - 2024.2.21
for 230826, 240124, 240207
from /home/jinn/openpilot/aJLLold/Model/train_modelB6b.py

1. Use the output of supercombo079.keras as ground truth data to train modelB6
2. Tasks: Path Prediction + Lane Detection + Lead Car Detection
   y_true[2383] = (Ytrue0, Ytrue1, Ytrue2, ..., Ytrue10, Ytrue11)
   y_pred[2383] = outs[0] + ... + outs[11] in sim_output.txt.

Input:
  /home/jinn/dataB6/UHD--2018-08-02--08-34-47--32/yuv.h5
  /home/jinn/dataB6/UHD--2018-08-02--08-34-47--33/yuv.h5
  /home/jinn/dataB6/UHD--2018-08-02--08-34-47--37/yuv.h5
Output:
  /home/jinn/openpilot/aJLL/Model/saved_model/B6.keras
  /home/jinn/openpilot/aJLL/Model/saved_model/B6BW.hdf5
  /home/jinn/openpilot/aJLL/Model/output/B6Loss.png

Train and Validate Model: Run on 3 terminals
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ python serverB6.py --port 5557
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ python serverB6.py --port 5558 --validation
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ python train_modelB6.py --port 5557 --port_val 5558

Test Model Step 1: /output/B6Sim.png
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ python simulatorB6.py
Test Model Step 2: /output/B6Y.png
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ python train_modelB6.py

Important Hyper-Parameters (tune these for better results):
BATCH_SIZE, STEPS, EPOCHS, learning_rate, decay_steps, decay_rate, weight_decay, clipvalue

Set BATCH_SIZE, STEPS, EPOCHS in serverB6.py. Set larger BATCH_SIZE if you have more GPUs.
Epochs (JLL) != EPOCHS (Keras); Steps (JLL) != STEPS (Keras); Run datagen_debug() for these.
"""
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '1' 
import time
import h5py
import pickle
import argparse
import numpy as np
import tf_keras as keras
import tf_keras.backend as KB
from tf_keras.callbacks import Callback, ModelCheckpoint, ReduceLROnPlateau
# keras.mixed_precision.set_global_policy('mixed_float16')
# devices = tf.config.experimental.list_physical_devices('GPU')
# tf.config.experimental.set_memory_growth(devices[0], True)

from glob import glob
from modelB6 import get_policy_model, custom_losses

def custom_loss(y_true, y_pred):
  loss_MSE = keras.losses.mse(y_true, y_pred)  # MC5
  return loss_MSE

def maxae(y_true, y_pred):
  return KB.max(KB.abs(y_pred - y_true), axis=-1)

class PrintLearningRate(Callback):
  def on_epoch_begin(self, epoch, logs={}):
    lr = KB.eval(self.model.optimizer.lr)
    print(f'\n epoch: {epoch}, LR: {lr:.3e}')

def scheduler(epoch):
  return KB.get_value(model.optimizer.lr)

def lr_CosineDecayWarmup(global_epoch, total_epoches, warmup_epoches, hold, target_lr=1e-3):
  if global_epoch < warmup_epoches:
    learning_rate = target_lr * (global_epoch / warmup_epoches)  # linear warmup from 0 to target_lr
  elif global_epoch >= warmup_epoches and global_epoch < warmup_epoches + hold:
    learning_rate = target_lr
  else:
    learning_rate = 0.5 * target_lr * ( 1 + np.cos( np.pi * float(global_epoch / total_epoches) ) )
  return learning_rate

class CosineDecayWarmup(Callback):
  def __init__(self, target_lr=1e-3, total_epoches=0, warmup_epoches=0, hold=0):
    super(CosineDecayWarmup, self).__init__()
    self.target_lr = target_lr
    self.total_epoches = total_epoches
    self.warmup_epoches = warmup_epoches
    self.hold = hold
    self.global_epoch = 0
    self.lrs = []

  def on_epoch_begin(self, epoch, logs=None):
    self.global_epoch = KB.get_value(epoch)

  def on_batch_begin(self, batch, logs=None):
    lr = lr_CosineDecayWarmup(global_epoch=self.global_epoch, total_epoches=self.total_epoches,
                              warmup_epoches=self.warmup_epoches, hold=self.hold, target_lr=self.target_lr)
    #print("#--- self.global_epoch =", self.global_epoch)
    KB.set_value(self.model.optimizer.lr, lr)

  def on_batch_end(self, batch, logs=None):
    lr = model.optimizer.lr.numpy()
    self.lrs.append(lr)

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
              Ximgs[j] = np.vstack((yuvX[(j)], yuvX[(j) + 1]))
              Xin3[j] = Xin3_temp

              # if j != 0:
              for k in range(len(Ytrue)):
                Ytrue[k] = np.vstack([Ytrue[k], fake_y[j][k]])

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

if __name__=="__main__":
  start = time.time()
  AP = argparse.ArgumentParser(description='Training modelB6 causal attention model.')
  AP.add_argument('--epochs', '-e', type=int, default=60, help='Model will be trained by watching n times of inputs.')
  AP.add_argument('--batch_size', '-b', type=int, default=16, help='Port of server for validation dataset.')
  args = AP.parse_args()

  # Get input train & valid data generator
  no_rnn_state = True
  model_name = "B6p.keras"
  weights_name = "B6p.weights.h5"
  input_dirs = [os.path.dirname(d) for d in glob(os.path.join(os.path.expanduser('~'), 'dataB6', "*", "fake_y.pickle"))]
  train_size = round(len(input_dirs) * 0.8)
  train_data_size, train_xy = input_data_generator(input_dirs[:train_size], args.batch_size, no_rnn_state=no_rnn_state)
  valid_data_size, valid_xy = input_data_generator(input_dirs[train_size:], args.batch_size, no_rnn_state=no_rnn_state)
  train_steps = train_data_size // args.batch_size
  valid_steps = valid_data_size // args.batch_size

  # Build model
  img_shape = (12, 128, 256)
  desire_shape = (8)
  traffic_convention_shape = (2)
  rnn_state_shape = (512)
  num_classes = 6
  model = get_policy_model(img_shape, desire_shape, traffic_convention_shape, rnn_state_shape, num_classes, 
                          out_seperate=False, no_rnn_state=no_rnn_state, memorize_unit="casual_attention")

  # Compile model
  filepath = os.path.join(os.path.expanduser('~'), 'aJLL/Model/saved_model', weights_name)  # BW: Best Weights
  checkpoint = ModelCheckpoint(filepath, monitor='val_maxae', verbose=1, 
                               save_best_only=True, save_weights_only=True, mode='min')

  total_epochs, warmup_epochs, hold = args.epochs, int(0.1 * args.epochs), 4  # must: total_epochs > warmup_epoches + hold
  # lr_schedule = CosineDecayWarmup(target_lr=1e-3, total_epoches=total_epochs, warmup_epoches=warmup_epochs, hold=hold)
  lr_schedule = ReduceLROnPlateau(monitor = "maxae",
                                  factor = np.exp(-0.8),
                                  patience = min(int(0.05 * args.epochs), 3),
                                  verbose = 1,
                                  mode = "min",
                                  min_delta = 0.0001,
                                  cooldown = 0,
                                  min_lr = 1e-20)
  # printlr = PrintLearningRate()
  # updatelr = keras.callbacks.LearningRateScheduler(scheduler)

  callbacks_list = [checkpoint, lr_schedule]

  optimizer = keras.optimizers.AdamW(learning_rate=1e-3, weight_decay=0.004, clipvalue=1.0)
  loss_funcs = custom_losses(args.batch_size, no_rnn_state=True)
  model.load_weights("saved_model/B6pBWNRS.weights.h5")
  model.compile(optimizer=optimizer, loss=loss_funcs, metrics=[maxae])
  model.summary()

  try:
    history = model.fit(
              train_xy,
              steps_per_epoch=train_steps, 
              epochs=args.epochs,
              validation_data=valid_xy,
              validation_steps=valid_steps, 
              verbose=1, callbacks=callbacks_list)
  except KeyboardInterrupt:
    del model
    exit()

  model.save(os.path.join(os.path.expanduser('~'), 'aJLL/Model/saved_model', model_name))  # rename and save "long" trained .keras

  end = time.time()
  hours, rem = divmod(end-start, 3600)
  minutes, seconds = divmod(rem, 60)
  print("#--- Training Time: {:0>2}:{:0>2}:{:05.2f}".format(int(hours),int(minutes),seconds))