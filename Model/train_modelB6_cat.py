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
from modelB6 import get_causal_attention_model, custom_losses
from datagen_catB6 import cat_input_data_generator

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

if __name__=="__main__":
  start = time.time()
  AP = argparse.ArgumentParser(description='Training modelB6 causal attention model.')
  AP.add_argument('--epochs', '-e', type=int, default=60, help='Model will be trained by watching n times of inputs.')
  AP.add_argument('--batch_size', '-b', type=int, default=16, help='Batch size for training.')
  args = AP.parse_args()

  # Get input train & valid data generator
  model_name = "catB6.keras"
  weights_name = "catB6.weights.h5"
  input_dirs = [os.path.dirname(d) for d in glob("/home/neil/dataOp/*/*/fake_cat_y.pickle")][:5]
  np.random.shuffle(input_dirs)
  train_size = round(len(input_dirs) * 0.8)
  print(f"#--- Train on {train_size} samples, Valid on {len(input_dirs)-train_size} samples.")
  train_data_size, train_xy = cat_input_data_generator(input_dirs[:train_size], args.batch_size)
  valid_data_size, valid_xy = cat_input_data_generator(input_dirs[train_size:], args.batch_size)
  print(f"#--- Train data: {train_data_size}, Valid data: {valid_data_size}.")
  train_steps = train_data_size // args.batch_size
  valid_steps = valid_data_size // args.batch_size

  # Build model
  img_shape = (6, 128, 256)
  desire_shape = (8)
  traffic_convention_shape = (2)
  model = get_causal_attention_model(img_shape, desire_shape, traffic_convention_shape)

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
  loss_funcs = custom_losses(args.batch_size, no_rnn_state=True, cat=True)
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