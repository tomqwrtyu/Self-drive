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
# 0 = all messages are logged (default behavior)
# 1 = INFO messages are not printed
# 2 = INFO and WARNING messages are not printed
# 3 = INFO, WARNING, and ERROR messages are not printed
import time
import h5py
import pickle
import argparse
import numpy as np
import tf_keras as keras
import tf_keras.backend as KB
from tf_keras.callbacks import Callback, ModelCheckpoint, ReduceLROnPlateau

from modelB6 import get_vision_model

def custom_loss(y_true, y_pred):
    #--- y_true.shape = (None, None)
    #--- y_pred.shape = (None, 2383)
    #--- y_true.shape = (16, 2383)
    #--- y_pred.shape = (16, 2383)
    #--- max of y_true, max index = 266.34155 (array([0]), array([769]))    # at y_true(0, 769); 769-385=384=outs[1][384] in sim_output.txt
    #--- min of y_true, min index = -139.88947 (array([0]), array([1168]))  # MDN_GROUP_SIZE*LEAD_MDN_N+SELECTION=11*5+3=58
    # 1168-385-386-386=1168-1157=11=outs[3][11] (1st of 11, 2nd of 5); -139.889=data[1*11] in driving079.cc; why -???
  # loss_CS = keras.losses.cosine_similarity(y_true, y_pred, axis=-1)  # MC4 (Md Case 4)
  loss_MSE = keras.losses.mse(y_true, y_pred)  # MC5
  return loss_MSE
  # return 0.5 * loss_CS + 0.5 * loss_MSE  # MC5

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

def input_data_generator(data_dir:list, batch_size:int=4):
  if isinstance(data_dir, str):
      data_dir = [data_dir]

  total_size = 0
  for d in data_dir:
      with open(d + "/fake_effnet_y.pickle", "rb") as fy:
        total_size += len(pickle.load(fy))
        
  def gen(data_dir:list, batch_size:int):
    nonlocal total_size
    presicion = np.float32
    Ximgs  = np.zeros((batch_size, 12, 128, 256), dtype=presicion)   # Is YUV input img uint8? No. See float8 ysf = convert_float8(ys) in loadyuv.cl
    Ytrue  = None

    dir_size = len(data_dir)
    max_steps = total_size // batch_size
    steps = 0
    
    while True:
      data_from_prev_dir_count = 0
      
      for didx, d in enumerate(data_dir):
        with h5py.File(d + "/yuv.h5", "r") as yuv, open(d + "/fake_effnet_y.pickle", "rb") as fy:
          fake_y = pickle.load(fy)
          yuvX = yuv['X']
          data_len = len(fake_y)
          start = 0

          # Previous file left some data that cannot form a batch
          if data_from_prev_dir_count != 0:
            start = batch_size - data_from_prev_dir_count

            for j in range(data_from_prev_dir_count, batch_size):
              Ximgs[j] = np.vstack((yuvX[(j)], yuvX[(j) + 1]))
              Ytrue = np.vstack([Ytrue, fake_y[j].numpy()])

            steps += 1
            yield Ximgs, Ytrue

          for i in range(start, data_len, batch_size):       
            end = batch_size

            if data_len - i < batch_size:
              if didx != dir_size - 1: # not the last file
                data_from_prev_dir_count = data_len - i
                end = data_len - i
              else:
                continue # skip rest data

            # wrap data into a batch
            for j in range(end):
              Ximgs[j] = np.vstack((yuvX[(i + j)], yuvX[(i + j) + 1]))

              if j != 0:
                Ytrue = np.vstack([Ytrue, fake_y[i + j].numpy()])
              else:
                Ytrue = fake_y[i].numpy()

            # if data size is enough for a batch, yield it
            if end == batch_size:
              steps += 1
              yield Ximgs, Ytrue

              if steps >= max_steps:
                steps = 0

  return total_size, gen(data_dir, batch_size)

if __name__=="__main__":
  start = time.time()
  AP = argparse.ArgumentParser(description='Training modelB6 vision part')
  AP.add_argument('--epochs', '-e', type=int, default=60, help='Model will be trained by watching n times of inputs.')
  AP.add_argument('--batch_size', '-b', type=int, default=16, help='Port of server for validation dataset.')
  args = AP.parse_args()

  # Get input train & valid data generator
  model_name = "B6v.keras"
  weights_name = "B6vBW.weights.h5"
  input_dirs = [os.path.join(os.path.expanduser('~'), 'dataB6', d) for d in os.listdir(os.path.join(os.path.expanduser('~'), 'dataB6'))]
  train_size = round(len(input_dirs) * 0.8)
  train_data_size, train_xy = input_data_generator(input_dirs[:train_size], args.batch_size)
  valid_data_size, valid_xy = input_data_generator(input_dirs[train_size:], args.batch_size)
  train_steps = train_data_size // args.batch_size
  valid_steps = valid_data_size // args.batch_size

  # Build model
  img_shape = (12, 128, 256)
  desire_shape = (8)
  traffic_convention_shape = (2)
  rnn_state_shape = (512)
  num_classes = 6
  # model = get_model_EffNetB3(img_shape, desire_shape, traffic_convention_shape, rnn_state_shape, num_classes)
  model = get_vision_model(img_shape)

  # Compile model
  filepath = os.path.join(os.path.expanduser('~'), 'aJLL/Model/saved_model', weights_name)  # BW: Best Weights
  checkpoint = ModelCheckpoint(filepath, monitor='val_loss', verbose=1, 
                               save_best_only=True, save_weights_only=True, mode='min')

  total_epochs, warmup_epochs, hold = args.epochs, int(0.1 * args.epochs), 4  # must: total_epochs > warmup_epoches + hold
  # lr_schedule = CosineDecayWarmup(target_lr=1e-3, total_epoches=total_epochs, warmup_epoches=warmup_epochs, hold=hold)
  lr_schedule = ReduceLROnPlateau(monitor = "loss",
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
  # loss_funcs = custom_losses(args.batch_size, no_rnn_state=True)
  model.compile(optimizer=optimizer, loss=custom_loss, metrics=[maxae])
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

  # model.save(os.path.join(os.path.expanduser('~'), 'aJLL/Model/saved_model', model_name))  # rename and save "long" trained .keras

  end = time.time()
  hours, rem = divmod(end-start, 3600)
  minutes, seconds = divmod(rem, 60)
  print("#--- Training Time: {:0>2}:{:0>2}:{:05.2f}".format(int(hours),int(minutes),seconds))