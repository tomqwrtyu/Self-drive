'''  YJW, SLT, JLL, 2021.9.9 - 2022.4.26, 2023.8.26
for 230826
from /home/jinn/YPN/B5/modelB5.py
modelB6 = EfficientNet + RNN + PoseNet => modelB6.dlc = supercombo079.dlc
supercombo: https://drive.google.com/file/d/1L8sWgYKtH77K6Kr3FQMETtAWeQNyyb8R/view
output.txt: https://github.com/JinnAIGroup/OPNet/blob/main/output.txt

Input:
  2 YUV images with 6 channels = (none, 12, 128, 256)
  inputs[ 0 ].shape = (1, 12, 128, 256)   # 12 = 2 frames x 6 channels (YUV_I420: Y=4, U=1, V=1)
  inputs[ 1 ].shape = (1, 8)
  inputs[ 2 ].shape = (1, 2)
  inputs[ 3 ].shape = (1, 512)
Output:
  outs[ 0 ].shape = (1, 385)
  outs[ 1 ].shape = (1, 386)
  outs[ 2 ].shape = (1, 386)
  outs[ 3 ].shape = (1, 58)
  outs[ 4 ].shape = (1, 200)
  outs[ 5 ].shape = (1, 200)
  outs[ 6 ].shape = (1, 200)
  outs[ 7 ].shape = (1, 8)
  outs[ 8 ].shape = (1, 4)
  outs[ 9 ].shape = (1, 32)
  outs[ 10 ].shape = (1, 12)
  outs[ 11 ].shape = (1, 512)
Run:
  (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ python modelB6.py

RT 1: 220208 Road Test Error on modelB5C2C.dlc:
  1. WARNING: This branch is not tested
     <= EventName.startupMaster (events.py) <= def get_startup_event (car_helpers.py)
     <= tested_branch = False (version.py <= manager.py)
     <= get_startup_event (controlsd.py)
RT 2: 220426 Road Test Error on B5YJ0421.dlc:
  1. WARNING: This branch is not tested
  2. openpilot Canceled  No close lead car
     <= EventName.noTarget: ET.NO_ENTRY : NoEntryAlert("No Close Lead Car") (<= events.py)
     <= self.events.add(EventName.noTarget) (<= controlsd.py)
  3. openpilot Unavailable  No Close Lead Car
  4. openpilot Unavailable  Planner Solution Error
'''
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '1' 
# 0 = all messages are logged (default behavior)
# 1 = INFO messages are not printed
# 2 = INFO and WARNING messages are not printed
# 3 = INFO, WARNING, and ERROR messages are not printed
import tensorflow as tf
import tf_keras as keras
import numpy as np

from typing import Literal
from tf_keras import layers
from collections import namedtuple

def stem(x0):
    #--- x0.shape = (None, 128, 256, 12)
  x = layers.Conv2D(32, 3, strides=2, padding="same", name="stem_conv")(x0)
    #--- x.shape = (None, 64, 128, 32)
  x = layers.BatchNormalization(axis=3)(x)
  x = layers.Activation("elu", name="stem_activation")(x)
  return x

def block(x0):
  count = "1"
  x = layers.DepthwiseConv2D(3, depth_multiplier=1, padding="same", name="block"+count+"a_dwconv")(x0)
    #print("tmp.shape=", x.shape.as_list())
    # tmp.shape= [None, 64, 128, 32]
  x = layers.BatchNormalization(axis=3)(x)
  x = layers.Activation("elu", name="block"+count+"a_activation")(x)
  x = layers.Conv2D(16, 1, padding="same", name="block"+count+"a_project_conv")(x)
  x = layers.BatchNormalization(axis=3)(x)

  tmp = layers.DepthwiseConv2D(3, depth_multiplier=1, padding="same", name="block"+count+"b_dwconv")(x)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  tmp = layers.Activation("elu", name="block"+count+"b_activation")(tmp)
  tmp = layers.Conv2D(16, 1, padding ="same", name="block"+count+"b_project_conv")(tmp)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  x = layers.add([tmp, x], name = "block"+count+"b_add")

  count = "2"
  x = layers.Conv2D(96, 1, padding="same", name="block"+count+"a_expand_conv")(x)
  x = layers.BatchNormalization(axis=3)(x)
  x = layers.Activation("elu", name="block"+count+"a_expand_activation")(x)
  x = layers.DepthwiseConv2D(3, depth_multiplier=1, strides=2, padding="same", name="block"+count+"a_dwconv")(x)
  x = layers.BatchNormalization(axis=3)(x)
  x = layers.Activation("elu", name="block"+count+"a_activation")(x)
  x = layers.Conv2D(24, 1, padding="same", name="block"+count+"a_project_conv")(x)
  x = layers.BatchNormalization(axis=3)(x)

  tmp = layers.Conv2D(144, 1, padding="same", name="block"+count+"b_expand_conv")(x)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  tmp = layers.Activation("elu", name="block"+count+"b_expand_activation")(tmp)
  tmp = layers.DepthwiseConv2D(3, depth_multiplier=1, padding="same", name="block"+count+"b_dwconv")(tmp)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  tmp = layers.Activation("elu", name="block"+count+"b_activation")(tmp)
  tmp = layers.Conv2D(24, 1, padding="same", name="block"+count+"b_project_conv")(tmp)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  x = layers.add([tmp,x], name="block"+count+"b_add")

  tmp = layers.Conv2D(144, 1, padding="same", name="block"+count+"c_expand_conv")(x)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  tmp = layers.Activation("elu", name="block"+count+"c_expand_activation")(tmp)
  tmp = layers.DepthwiseConv2D(3, depth_multiplier=1, padding="same", name="block"+count+"c_dwconv")(tmp)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  tmp = layers.Activation("elu", name="block"+count+"c_activation")(tmp)
  tmp = layers.Conv2D(24, 1, padding="same", name="block"+count+"c_project_conv")(tmp)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  x = layers.add([tmp,x], name="block"+count+"c_add")

  count="3"
  x = layers.Conv2D(144, 1, padding="same", name="block"+count+"a_expand_conv")(x)
  x = layers.BatchNormalization(axis=3)(x)
  x = layers.Activation("elu", name="block"+count+"a_expand_activation")(x)
  x = layers.DepthwiseConv2D(5, depth_multiplier=1,strides=2, padding="same", name="block"+count+"a_dwconv")(x)
  x = layers.BatchNormalization(axis=3)(x)
  x = layers.Activation("elu", name="block"+count+"a_activation")(x)
  x = layers.Conv2D(48, 1, padding="same", name="block"+count+"a_project_conv")(x)
  x = layers.BatchNormalization(axis=3)(x)

  tmp = layers.Conv2D(288, 1, padding="same", name="block"+count+"b_expand_conv")(x)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  tmp = layers.Activation("elu", name="block"+count+"b_expand_activation")(tmp)
  tmp = layers.DepthwiseConv2D(5, depth_multiplier=1, padding="same", name="block"+count+"b_dwconv")(tmp)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  tmp = layers.Activation("elu", name="block"+count+"b_activation")(tmp)
  tmp = layers.Conv2D(48, 1, padding="same", name="block"+count+"b_project_conv")(tmp)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  x = layers.add([tmp,x], name = "block"+count+"b_add")

  tmp = layers.Conv2D(288, 1, padding="same", name="block"+count+"c_expand_conv")(x)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  tmp = layers.Activation("elu", name="block"+count+"c_expand_activation")(tmp)
  tmp = layers.DepthwiseConv2D(5, depth_multiplier=1, padding="same", name="block"+count+"c_dwconv")(tmp)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  tmp = layers.Activation("elu", name="block"+count+"c_activation")(tmp)
  tmp = layers.Conv2D(48, 1, padding="same", name="block"+count+"c_project_conv")(tmp)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  x = layers.add([tmp,x], name = "block"+count+"c_add")

  count="4"
  x = layers.Conv2D(288, 1, padding="same", name="block"+count+"a_expand_conv")(x)
  x = layers.BatchNormalization(axis=3)(x)
  x = layers.Activation("elu", name="block"+count+"a_expand_activation")(x)
  x = layers.DepthwiseConv2D(3, depth_multiplier=1,strides=2, padding="same", name="block"+count+"a_dwconv")(x)
  x = layers.BatchNormalization(axis=3)(x)
  x = layers.Activation("elu", name="block"+count+"a_activation")(x)
  x = layers.Conv2D(88, 1, padding="same", name="block"+count+"a_project_conv")(x)
  x = layers.BatchNormalization(axis=3)(x)

  tmp = layers.Conv2D(528, 1, padding="same", name="block"+count+"b_expand_conv")(x)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  tmp = layers.Activation("elu", name="block"+count+"b_expand_activation")(tmp)
  tmp = layers.DepthwiseConv2D(3, depth_multiplier=1, padding="same", name="block"+count+"b_dwconv")(tmp)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  tmp = layers.Activation("elu", name="block"+count+"b_activation")(tmp)
  tmp = layers.Conv2D(88, 1, padding="same", name="block"+count+"b_project_conv")(tmp)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  x = layers.add([tmp,x], name = "block"+count+"b_add")

  tmp = layers.Conv2D(528, 1, padding="same", name="block"+count+"c_expand_conv")(x)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  tmp = layers.Activation("elu", name="block"+count+"c_expand_activation")(tmp)
  tmp = layers.DepthwiseConv2D(3, depth_multiplier=1, padding="same", name="block"+count+"c_dwconv")(tmp)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  tmp = layers.Activation("elu", name="block"+count+"c_activation")(tmp)
  tmp = layers.Conv2D(88, 1, padding="same", name="block"+count+"c_project_conv")(tmp)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  x = layers.add([tmp,x], name = "block"+count+"c_add")

  tmp = layers.Conv2D(528, 1, padding="same", name="block"+count+"d_expand_conv")(x)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  tmp = layers.Activation("elu", name="block"+count+"d_expand_activation")(tmp)
  tmp = layers.DepthwiseConv2D(3, depth_multiplier=1, padding="same", name="block"+count+"d_dwconv")(tmp)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  tmp = layers.Activation("elu", name="block"+count+"d_activation")(tmp)
  tmp = layers.Conv2D(88, 1, padding="same", name="block"+count+"d_project_conv")(tmp)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  x = layers.add([tmp,x], name = "block"+count+"d_add")

  count = "5"
  x = layers.Conv2D(528, 1, padding="same", name="block"+count+"a_expand_conv")(x)
  x = layers.BatchNormalization(axis=3)(x)
  x = layers.Activation("elu", name="block"+count+"a_expand_activation")(x)
  x = layers.DepthwiseConv2D(5, depth_multiplier=1, padding="same", name="block"+count+"a_dwconv")(x)
  x = layers.BatchNormalization(axis=3)(x)
  x = layers.Activation("elu", name="block"+count+"a_activation")(x)
  x = layers.Conv2D(120, 1, padding="same", name="block"+count+"a_project_conv")(x)
  x = layers.BatchNormalization(axis=3)(x)

  tmp = layers.Conv2D(720, 1, padding="same", name="block"+count+"b_expand_conv")(x)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  tmp = layers.Activation("elu", name="block"+count+"b_expand_activation")(tmp)
  tmp = layers.DepthwiseConv2D(5, depth_multiplier=1, padding="same", name="block"+count+"b_dwconv")(tmp)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  tmp = layers.Activation("elu", name="block"+count+"b_activation")(tmp)
  tmp = layers.Conv2D(120, 1, padding="same", name="block"+count+"b_project_conv")(tmp)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  x = layers.add([tmp,x], name = "block"+count+"b_add")

  tmp = layers.Conv2D(720, 1, padding="same", name="block"+count+"c_expand_conv")(x)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  tmp = layers.Activation("elu", name="block"+count+"c_expand_activation")(tmp)
  tmp = layers.DepthwiseConv2D(5, depth_multiplier=1, padding="same", name="block"+count+"c_dwconv")(tmp)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  tmp = layers.Activation("elu", name="block"+count+"c_activation")(tmp)
  tmp = layers.Conv2D(120, 1, padding="same", name="block"+count+"c_project_conv")(tmp)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  x = layers.add([tmp,x], name = "block"+count+"c_add")

  tmp = layers.Conv2D(720, 1, padding="same", name="block"+count+"d_expand_conv")(x)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  tmp = layers.Activation("elu", name="block"+count+"d_expand_activation")(tmp)
  tmp = layers.DepthwiseConv2D(5, depth_multiplier=1, padding="same", name="block"+count+"d_dwconv")(tmp)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  tmp = layers.Activation("elu", name="block"+count+"d_activation")(tmp)
  tmp = layers.Conv2D(120, 1, padding="same", name="block"+count+"d_project_conv")(tmp)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  x = layers.add([tmp,x], name = "block"+count+"d_add")

  count = "6"
  x = layers.Conv2D(720, 1, padding="same", name="block"+count+"a_expand_conv")(x)
  x = layers.BatchNormalization(axis=3)(x)
  x = layers.Activation("elu", name="block"+count+"a_expand_activation")(x)
  x = layers.DepthwiseConv2D(5, depth_multiplier=1,strides = 2, padding="same", name="block"+count+"a_dwconv")(x)
  x = layers.BatchNormalization(axis=3)(x)
  x = layers.Activation("elu", name="block"+count+"a_activation")(x)
  x = layers.Conv2D(208, 1, padding="same", name="block"+count+"a_project_conv")(x)
  x = layers.BatchNormalization(axis=3)(x)

  tmp = layers.Conv2D(1248, 1, padding="same", name="block"+count+"b_expand_conv")(x)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  tmp = layers.Activation("elu", name="block"+count+"b_expand_activation")(tmp)
  tmp = layers.DepthwiseConv2D(5, depth_multiplier=1, padding="same", name="block"+count+"b_dwconv")(tmp)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  tmp = layers.Activation("elu", name="block"+count+"b_activation")(tmp)
  tmp = layers.Conv2D(208, 1, padding="same", name="block"+count+"b_project_conv")(tmp)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  x = layers.add([tmp,x], name = "block"+count+"b_add")

  tmp = layers.Conv2D(1248, 1, padding="same", name="block"+count+"c_expand_conv")(x)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  tmp = layers.Activation("elu", name="block"+count+"c_expand_activation")(tmp)
  tmp = layers.DepthwiseConv2D(5, depth_multiplier=1, padding="same", name="block"+count+"c_dwconv")(tmp)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  tmp = layers.Activation("elu", name="block"+count+"c_activation")(tmp)
  tmp = layers.Conv2D(208, 1, padding="same", name="block"+count+"c_project_conv")(tmp)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  x = layers.add([tmp,x], name = "block"+count+"c_add")

  tmp = layers.Conv2D(1248, 1, padding="same", name="block"+count+"d_expand_conv")(x)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  tmp = layers.Activation("elu", name="block"+count+"d_expand_activation")(tmp)
  tmp = layers.DepthwiseConv2D(5, depth_multiplier=1, padding="same", name="block"+count+"d_dwconv")(tmp)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  tmp = layers.Activation("elu", name="block"+count+"d_activation")(tmp)
  tmp = layers.Conv2D(208, 1, padding="same", name="block"+count+"d_project_conv")(tmp)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  x = layers.add([tmp,x], name = "block"+count+"d_add")

  tmp = layers.Conv2D(1248, 1, padding="same", name="block"+count+"e_expand_conv")(x)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  tmp = layers.Activation("elu", name="block"+count+"e_expand_activation")(tmp)
  tmp = layers.DepthwiseConv2D(5, depth_multiplier=1, padding="same", name="block"+count+"e_dwconv")(tmp)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  tmp = layers.Activation("elu", name="block"+count+"e_activation")(tmp)
  tmp = layers.Conv2D(208, 1, padding="same", name="block"+count+"e_project_conv")(tmp)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  x = layers.add([tmp,x], name = "block"+count+"e_add")

  count = "7"
  x = layers.Conv2D(1248, 1, padding="same", name="block"+count+"a_expand_conv")(x)
  x = layers.BatchNormalization(axis=3)(x)
  x = layers.Activation("elu", name="block"+count+"a_expand_activation")(x)
  x = layers.DepthwiseConv2D(3, depth_multiplier=1, padding="same", name="block"+count+"a_dwconv")(x)
  x = layers.BatchNormalization(axis=3)(x)
  x = layers.Activation("elu", name="block"+count+"a_activation")(x)
  x = layers.Conv2D(352, 1, padding="same", name="block"+count+"a_project_conv")(x)
  x = layers.BatchNormalization(axis=3)(x)

  tmp = layers.Conv2D(2112, 1, padding="same", name="block"+count+"b_expand_conv")(x)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  tmp = layers.Activation("elu", name="block"+count+"b_expand_activation")(tmp)
  tmp = layers.DepthwiseConv2D(3, depth_multiplier=1, padding="same", name="block"+count+"b_dwconv")(tmp)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  tmp = layers.Activation("elu", name="block"+count+"b_activation")(tmp)
  tmp = layers.Conv2D(352, 1, padding="same", name="block"+count+"b_project_conv")(tmp)
  tmp = layers.BatchNormalization(axis=3)(tmp)
  x = layers.add([tmp,x], name = "block"+count+"b_add")
  return x

def top(x0):
    #--- x0.shape = (None, 4, 8, 352)
  x = layers.Conv2D(1408, 1, padding="same", name="top_conv")(x0)
  x = layers.Activation("elu", name="top_activation")(x)
    #--- x.shape = (None, 4, 8, 1408)
  return x

def RNN(x, desire, traffic_convection, rnn_state):
  desire1 = layers.Dense(use_bias=False, units=8)(desire)
  traffic_convection1 = layers.Dense(use_bias=False, units = 2)(traffic_convection)
  x_concate = layers.Concatenate(axis=-1)([desire1, traffic_convection1, x])
  x_dense = layers.Dense(use_bias=False, units=1024)(x_concate)
    #--- x_dense.shape = (None, 1024)
  x_1 = layers.Activation("relu")(x_dense)

  rnn_rz = layers.Dense(use_bias=False, units=512)(rnn_state)
  rnn_rr = layers.Dense(use_bias=False, units=512)(rnn_state)
  snpe_pleaser = layers.Dense(use_bias=False, units=512)(rnn_state)
  rnn_rh = layers.Dense(use_bias=False, units = 512)(snpe_pleaser)

  rnn_z = layers.Dense(use_bias=False, units=512)(x_1)
  rnn_h = layers.Dense(use_bias=False, units=512)(x_1)
  rnn_r = layers.Dense(use_bias=False, units=512)(x_1)

  add = layers.add([rnn_rz , rnn_z])
  activation_1 = layers.Activation("sigmoid")(add)
  add_1 = layers.add([rnn_rr , rnn_r])

  activation = layers.Activation("sigmoid")(add_1)
  multiply = rnn_rh*activation
  add_2 = layers.add([rnn_h , multiply])

  activation_2 = layers.Activation("tanh")(add_2)
  one_minus = layers.Dense(use_bias=False, units=512)(activation_1)
  multiply_2 = one_minus*activation_2
  multiply_1 = snpe_pleaser*activation_1
  out11 = layers.Add(name="feedback")([multiply_1 , multiply_2])
  return out11

def fork1(x, no_desire_state = False):
  xp = layers.Dense(256, activation='relu', name="1_path")(x)
  xp = layers.Dense(256, activation='relu', name="2_path")(xp)
  xp = layers.Dense(256, activation='relu', name="3_path")(xp)
  x0 = layers.Dense(128, activation='relu', name="final_path")(xp)

  xll = layers.Dense(256, activation='relu', name="1_left_lane")(x)
  xll = layers.Dense(256, activation='relu', name="2_left_lane")(xll)
  xll = layers.Dense(256, activation='relu', name="3_left_lane")(xll)
  x1 = layers.Dense(128, activation='relu', name="final_left_lane")(xll)

  xrl = layers.Dense(256, activation='relu', name="1_right_lane")(x)
  xrl = layers.Dense(256, activation='relu', name="2_right_lane")(xrl)
  xrl = layers.Dense(256, activation='relu', name="3_right_lane")(xrl)
  x2 = layers.Dense(128, activation='relu', name="final_right_lane")(xrl)

  xl = layers.Dense(256, activation='relu', name="1_lead")(x)
  xl = layers.Dense(256, activation='relu', name="2_lead")(xl)
  xl = layers.Dense(256, activation='relu', name="3_lead")(xl)
  x3 = layers.Dense(128, activation='relu', name="final_lead")(xl)

  xlx = layers.Dense(256, activation='relu', name="1_long_x")(x)
  xlx = layers.Dense(256, activation='relu', name="2_long_x")(xlx)
  xlx = layers.Dense(256, activation='relu', name="3_long_x")(xlx)
  x4 = layers.Dense(128, activation='relu', name="final_long_x")(xlx)

  xla = layers.Dense(256, activation='relu', name="1_long_a")(x)
  xla = layers.Dense(256, activation='relu', name="2_long_a")(xla)
  xla = layers.Dense(256, activation='relu', name="3_long_a")(xla)
  x5 = layers.Dense(128, activation='relu', name="final_long_a")(xla)

  xlv = layers.Dense(256, activation='relu', name="1_long_v")(x)
  xlv = layers.Dense(256, activation='relu', name="2_long_v")(xlv)
  xlv = layers.Dense(256, activation='relu', name="3_long_v")(xlv)
  x6 = layers.Dense(128, activation='relu', name="final_long_v")(xlv)

  if not no_desire_state:
    xds = layers.Dense(128, activation='relu', name="1_desire_state")(x)
    x7 = layers.Dense(8, name="final_desire_state")(xds)

  out0 = layers.Dense(385, name="path")(x0) #
  out1 = layers.Dense(386, name="left_lane")(x1) #
  out2 = layers.Dense(386, name="right_lane")(x2) #
  out3 = layers.Dense(58, name="lead")(x3) #
  out4 = layers.Dense(200, name="long_x")(x4) #
  out5 = layers.Dense(200, name="long_a")(x5) #
  out6 = layers.Dense(200, name="long_v")(x6) #

  if not no_desire_state:
    out7 = layers.Softmax(axis=-1, name="desire_state")(x7) #
    return out0, out1, out2, out3, out4, out5, out6, out7
  else:
    return out0, out1, out2, out3, out4, out5, out6

def fork2(x):
  x1 = layers.Dense(256, activation='relu', name="meta0")(x)
  out8 = layers.Dense(4, activation='sigmoid', name="meta")(x1) # 
  dp1 = layers.Dense(32, name="desire_final_dense")(x1)
  dp2 = layers.Reshape((4, 8), name="desire_reshape")(dp1)
  dp3 = layers.Softmax(axis=-1, name="desire_pred")(dp2)
  out9 = layers.Flatten(name="desire_pred_2")(dp3) # 
  return out8, out9

def fork3(x):
  x = layers.Dense(64, activation='elu')(x)
  x = layers.Dense(32, activation='elu')(x)
  out10 = layers.Dense(12, name="pose")(x) # 
  return out10

def EffNet(x0):
    #--- x0.shape = (None, 128, 256, 12)
  x = stem(x0)
    #--- x.shape = (None, 64, 128, 32)
  x = block(x)
    #--- x.shape = (None, 4, 8, 352)
  x = top(x)
    #--- x.shape = (None, 4, 8, 1408)
  x = layers.Conv2D(32, 1, padding="same")(x)
    #--- x.shape = (None, 4, 8, 32)
  x = layers.Activation("elu")(x)
    #--- x.shape = (None, 4, 8, 32)
  x_to_RNNfk2fk3 = layers.Flatten()(x)
    #--- x_to_RNNfk2fk3.shape = (None, 1024)
  return x_to_RNNfk2fk3

class SmoothL1Loss(keras.losses.Loss):
    def __init__(self, reduction='sum_over_batch_size', beta=1.0):
        super().__init__(reduction=reduction)
        self.beta = beta

    def call(self, y_true, y_pred):
        diff = tf.abs(y_true - y_pred)
        smooth_l1_loss = tf.where(diff < self.beta, 
                                  0.5 * diff**2 / self.beta, 
                                  diff - 0.5 * self.beta)
        
        if self.reduction == 'sum_over_batch_size':
            return tf.reduce_mean(smooth_l1_loss)
        elif self.reduction == 'sum':
            return tf.reduce_sum(smooth_l1_loss)
        else:
            return smooth_l1_loss

class PlanMhpLoss(keras.losses.Loss):
    def __init__(self, sigma_clamp=1e-3, loss_clamp=1000.):
        super().__init__()
        self.sigma_clamp = sigma_clamp
        self.loss_clamp = loss_clamp

    def path_laplacian_nll_loss(self, mean_true, mean_pred, sigma):
        err = tf.abs(mean_true - mean_pred)
        sigma_min = tf.maximum(sigma, tf.math.log(self.sigma_clamp))
        sigma_max = tf.maximum(sigma, tf.math.log(1e-6 + err / self.loss_clamp))
        nll = err * tf.exp(-sigma_max) + sigma_min
        return tf.reduce_sum(nll, axis=(1, 2))

    def mean_std(self, tensor):
        mean = tf.reduce_mean(tensor, axis=(2, 3))
        std = tf.math.reduce_std(tensor, axis=(2, 3))
        return mean, std

    def call(self, y_true, y_pred):
        plan_pred, plan_gt, plan_prob_gt = y_pred[0], y_true[0], y_true[1]

        batch_size = tf.shape(plan_pred)[0]

        best_gt_plan_idx = tf.argmax(plan_prob_gt, axis=1)

        paths = tf.reshape(plan_pred, (-1, 5, 991))
        path1_pred = tf.reshape(paths[:, 0, :-1], (-1, 2, 33, 15))
        path2_pred = tf.reshape(paths[:, 1, :-1], (-1, 2, 33, 15))
        path3_pred = tf.reshape(paths[:, 2, :-1], (-1, 2, 33, 15))
        path4_pred = tf.reshape(paths[:, 3, :-1], (-1, 2, 33, 15))
        path5_pred = tf.reshape(paths[:, 4, :-1], (-1, 2, 33, 15))
        path_pred_prob = paths[:, :, -1]

        path_gt = tf.gather_nd(plan_gt, tf.stack([tf.range(tf.shape(plan_gt)[0]), best_gt_plan_idx], axis=1))
        mean_gt_path, _ = self.mean_std(path_gt)

        mean_pred_path1, std_pred_path1 = self.mean_std(path1_pred)
        mean_pred_path2, std_pred_path2 = self.mean_std(path2_pred)
        mean_pred_path3, std_pred_path3 = self.mean_std(path3_pred)
        mean_pred_path4, std_pred_path4 = self.mean_std(path4_pred)
        mean_pred_path5, std_pred_path5 = self.mean_std(path5_pred)

        path1_loss = self.path_laplacian_nll_loss(mean_gt_path, mean_pred_path1, std_pred_path1)
        path2_loss = self.path_laplacian_nll_loss(mean_gt_path, mean_pred_path2, std_pred_path2)
        path3_loss = self.path_laplacian_nll_loss(mean_gt_path, mean_pred_path3, std_pred_path3)
        path4_loss = self.path_laplacian_nll_loss(mean_gt_path, mean_pred_path4, std_pred_path4)
        path5_loss = self.path_laplacian_nll_loss(mean_gt_path, mean_pred_path5, std_pred_path5)

        # MHP loss
        path_head_loss = tf.stack([path1_loss, path2_loss, path3_loss, path4_loss, path5_loss], axis=1)

        idx = tf.argmin(path_head_loss, axis=1)
        best_path_mask = tf.one_hot(idx, depth=5)
        mask = tf.cast(tf.fill((batch_size, 5), 1e-6), dtype=tf.float32)
        path_perhead_loss = tf.reduce_sum(tf.multiply(path_head_loss, mask), axis=1)
        path_perhead_loss = tf.reduce_mean(path_perhead_loss)

        cross_entropy_loss = keras.losses.CategoricalCrossentropy(from_logits=False, reduction='mean')
        path_prob_loss = cross_entropy_loss(best_path_mask, path_pred_prob)

        plan_loss = path_perhead_loss + path_prob_loss
        return plan_loss
    
PATH_IDX   = 0      # o0:  192*2+1 = 385
LL_IDX     = 385    # o1:  192*2+2 = 386
RL_IDX     = 771    # o2:  192*2+2 = 386
LEAD_IDX   = 1157   # o3:  11*5+3 = 58
LONG_X_IDX = 1215   # o4:  100*2 = 200
LONG_V_IDX = 1415   # o5:  100*2 = 200
LONG_A_IDX = 1615   # o6:  100*2 = 200
DESIRE_IDX = 1815   # o7:  8
META_IDX   = 1823   # o8:  4
PRED_IDX   = 1827   # o9:  32
POSE_IDX   = 1859   # o10: 12
STATE_IDX  = 1871   # o11: 512
OUTPUT_IDX = 2383

class MaskedCausalAttention(layers.Layer):
    def __init__(self, h_dim, max_T, n_heads, drop_p, name=None, **kwargs):
        super(MaskedCausalAttention, self).__init__(name=name, **kwargs)
        self.n_heads = n_heads
        self.max_T = max_T
        self.h_dim = h_dim

        self.q_net = keras.layers.Dense(h_dim)
        self.k_net = keras.layers.Dense(h_dim)
        self.v_net = keras.layers.Dense(h_dim)

        self.att_drop = keras.layers.Dropout(drop_p)
        self.proj_drop = keras.layers.Dropout(drop_p)

        self.proj_net = keras.layers.Dense(h_dim)

        # Create a mask
        ones = tf.ones((max_T, max_T), dtype=tf.float32)
        mask = tf.linalg.band_part(ones, -1, 0)  # lower triangular part
        mask = tf.expand_dims(tf.expand_dims(mask, axis=0), axis=0)  # add batch and head dimensions
        self.mask = tf.Variable(mask * -1e9, trainable=False)  # Fill with a large negative value

    def call(self, x):
        _, T, C = x.shape # batch size, seq length, h_dim * n_heads

        N, D = self.n_heads, C // self.n_heads # N = num heads, D = attention dim

        # Rearrange q, k, v as (B, N, T, D)
        q = tf.reshape(self.q_net(x), [-1, T, N, D])
        k = tf.reshape(self.k_net(x), [-1, T, N, D])
        v = tf.reshape(self.v_net(x), [-1, T, N, D])

        # Transpose q, k, v for attention calculation
        q = tf.transpose(q, perm=[0, 2, 1, 3])
        k = tf.transpose(k, perm=[0, 2, 1, 3])
        v = tf.transpose(v, perm=[0, 2, 1, 3])

        # Calculate attention weights
        weights = tf.matmul(q, k, transpose_b=True) / tf.math.sqrt(tf.cast(D, dtype=tf.float32))
        weights += self.mask[:, :, :T, :T]  # Apply causal mask
        normalized_weights = tf.nn.softmax(weights, axis=-1)
        normalized_weights = self.att_drop(normalized_weights)

        # Apply attention to values
        attention = tf.matmul(normalized_weights, v)

        # Rearrange attention for output
        attention = tf.transpose(attention, perm=[0, 2, 1, 3])
        attention = tf.reshape(attention, (-1, T, self.n_heads * D))

        # Project attention output
        out = self.proj_drop(self.proj_net(attention))

        return out
    
class MaskedCausalAttentionBlock(layers.Layer):
    def __init__(self, h_dim, max_T, n_heads, drop_p, name=None, **kwargs):
        super(MaskedCausalAttentionBlock, self).__init__(name=name, **kwargs)
        self.attn = MaskedCausalAttention(h_dim, max_T, n_heads, drop_p)
        self.mlp = keras.Sequential([
            layers.Dense(h_dim * 4),
            layers.Activation("gelu"),
            layers.Dense(h_dim),
            layers.Dropout(drop_p)
        ])
        self.norm1 = layers.LayerNormalization()
        self.norm2 = layers.LayerNormalization()

    def call(self, x):
        x = x + self.attn(self.norm1(x))
        x = x + self.mlp(self.norm2(x))
        return x
    
class custom_losses(keras.losses.Loss):
  #  __slots__ = ["path", "left_lane", "right_lane", "lead", "long_x", "long_a", 
  #              "long_v", "desire_state", "meta", "desire_pred", "pose", "feedback"]
  def __init__(self, batch_size:int, no_rnn_state:bool, cat:bool = False,
               weights:list[float] = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]) -> None:
    super().__init__(name="multiple_losses",reduction="none")
    self.batch_size = batch_size
    self.no_rnn_state = no_rnn_state
    self.cat = cat
    self.smoothL1 = SmoothL1Loss()
    self.categorical_crossentropy = keras.losses.CategoricalCrossentropy(reduction="none")
    self.kld = keras.losses.KLDivergence()
    self.weights = namedtuple('Weights', ['path', 'left_lane', 'right_lane', 'lead', 'long_x', 'long_a',
                                          'long_v', 'desire_state', 'meta', 'desire_pred', 'pose', 'feedback'])
    
    x_sq = np.multiply(np.linspace(1.0, 2.0, num=192), np.linspace(1.0, 2.0, num=192))
    self.path_penalty = np.ones(385, dtype=np.float32)
    self.path_penalty[:192] = x_sq
    self.path_penalty = tf.convert_to_tensor(self.path_penalty, dtype=tf.float32)

    self.lane_penalty = np.ones(386, dtype=np.float32)
    self.lane_penalty[:192] = x_sq
    self.lane_penalty = tf.convert_to_tensor(self.lane_penalty, dtype=tf.float32) 

    if no_rnn_state:
       weights[-1] = 0 # feedback loss is not used if no rnn state is used

    self.w = self.weights(*weights)

  def call(self, y_true, y_pred):
    path_loss = self.smoothL1.call(tf.math.multiply(y_true[:, PATH_IDX:   LL_IDX], self.path_penalty),
                                   tf.math.multiply(y_pred[:, PATH_IDX:   LL_IDX], self.path_penalty))  #--- o0.shape = (1, 385)

    left_lane_loss = self.smoothL1.call(tf.math.multiply(y_true[:, LL_IDX:     RL_IDX], self.lane_penalty), 
                                        tf.math.multiply(y_pred[:, LL_IDX:     RL_IDX], self.lane_penalty))

    right_lane_loss = self.smoothL1.call(tf.math.multiply(y_true[:, RL_IDX:     LEAD_IDX], self.lane_penalty), 
                                         tf.math.multiply(y_pred[:, RL_IDX:     LEAD_IDX], self.lane_penalty))

    lead_loss = self.smoothL1.call(y_true[:, LEAD_IDX:   LONG_X_IDX], y_pred[:, LEAD_IDX:   LONG_X_IDX])

    long_x_loss = self.smoothL1.call(y_true[:, LONG_X_IDX: LONG_V_IDX], y_pred[:, LONG_X_IDX: LONG_V_IDX])

    long_a_loss = self.smoothL1.call(y_true[:, LONG_V_IDX: LONG_A_IDX], y_pred[:, LONG_V_IDX: LONG_A_IDX])

    long_v_loss = self.smoothL1.call(y_true[:, LONG_A_IDX: DESIRE_IDX], y_pred[:, LONG_A_IDX: DESIRE_IDX])

    if not self.cat:
      desire_state_loss = self.categorical_crossentropy.call(y_true[:, DESIRE_IDX: META_IDX], y_pred[:, DESIRE_IDX: META_IDX])
      
      meta_loss = self.smoothL1.call(y_true[:, META_IDX:   PRED_IDX], y_pred[:, META_IDX:   PRED_IDX])
      
      desire_pred_loss = self.categorical_crossentropy.call(y_true[:, PRED_IDX:   POSE_IDX], y_pred[:, PRED_IDX:   POSE_IDX])
      
      pose_loss = self.smoothL1.call(y_true[:, POSE_IDX:   STATE_IDX], y_pred[:, POSE_IDX:   STATE_IDX])
      
      feedback_loss = 0 if self.no_rnn_state else self.smoothL1.call(y_true[:, STATE_IDX:  OUTPUT_IDX], y_pred[:, STATE_IDX:  OUTPUT_IDX])

      return path_loss*self.w[0] + left_lane_loss*self.w[1] + right_lane_loss*self.w[2] + \
            lead_loss*self.w[3] + long_x_loss*self.w[4] + long_a_loss*self.w[5] + \
            long_v_loss*self.w[6] + desire_state_loss*self.w[7] + meta_loss*self.w[8] + \
            desire_pred_loss*self.w[9] + pose_loss*self.w[10] + feedback_loss*self.w[11]
    else:
      return path_loss*self.w[0] + left_lane_loss*self.w[1] + right_lane_loss*self.w[2] + \
             lead_loss*self.w[3] + long_x_loss*self.w[4] + long_a_loss*self.w[5] + \
             long_v_loss*self.w[6]

# def losses():
#   ret = {"path": SmoothL1Loss(),
#           "left_lane": SmoothL1Loss(),
#           "right_lane": SmoothL1Loss(),
#           "lead": SmoothL1Loss(),
#           "long_x": SmoothL1Loss(),
#           "long_a": SmoothL1Loss(),
#           "long_v": SmoothL1Loss(),
#           "desire_state": keras.losses.SparseCategoricalCrossentropy(),
#           "meta": SmoothL1Loss(),
#           "desire_pred": SmoothL1Loss(),
#           "pose": SmoothL1Loss(),
#           "feedback": SmoothL1Loss()}
#   return ret
def gru(feature, desire, traffic_convection):
  x = layers.Concatenate(axis=-1)([desire, traffic_convection, feature])
  x = layers.Dense(units=1024)(x)
  x = layers.Reshape((1, -1))(x)
  out11 = keras.layers.GRU(512, name="feedback")(x)
  return out11

def casual_attention(feature, desire, traffic_convection):
  x = layers.Concatenate(axis=-1)([desire, traffic_convection, feature])
  # x = layers.Dense(units=512)(x)
  x = layers.Reshape((1, -1))(x)
  x = layers.Dense(units=1024)(x)
  x = MaskedCausalAttentionBlock(h_dim=1024, max_T=2, n_heads=4, drop_p=0.1)(x)
  x = MaskedCausalAttentionBlock(h_dim=1024, max_T=2, n_heads=4, drop_p=0.1)(x)
  out11 = tf.reshape(x, [-1, 1024])
  return out11

def get_model(img_shape, desire_shape, traffic_convection_shape, rnn_state_shape, num_classes, 
              out_seperate:bool = False, no_rnn_state:bool=False, memorize_unit:Literal["gru", "casual_attention"]="gru"):
  imgs = keras.Input(shape=img_shape, name="imgs")
    #--- imgs.shape = (None, 12, 128, 256)
  in0 = layers.Permute((2, 3, 1))(imgs)
    #--- in0.shape = (None, 128, 256, 12)
  in1 = keras.Input(shape=desire_shape, name="desire")
  in2 = keras.Input(shape=traffic_convection_shape, name="traffic_convection")
  if not no_rnn_state:
    in3 = keras.Input(shape=rnn_state_shape, name="rnn_state")

  x_to_RNNfk2fk3 = EffNet(in0)

  if not no_rnn_state:
    out11 = RNN(x_to_RNNfk2fk3, in1, in2, in3)
  else:
    if memorize_unit == "gru":
      out11 = gru(x_to_RNNfk2fk3, in1, in2)
    elif memorize_unit == "casual_attention":
      out11 = casual_attention(x_to_RNNfk2fk3, in1, in2)
    else:
      raise ValueError("memorize_unit must be 'gru' or 'casual_attention'")

  out0, out1, out2, out3, out4, out5, out6, out7 = fork1(out11)
  out8, out9 = fork2(x_to_RNNfk2fk3)
  out10      = fork3(x_to_RNNfk2fk3)
  
  if not no_rnn_state:
    outs = [out0, out1, out2, out3, out4, out5, out6, out7, out8, out9, out10, out11] if out_seperate \
          else layers.Concatenate(axis=-1, name="outputs")([out0, out1, out2, out3, out4, out5, out6, out7, out8, out9, out10, out11])
  else:
    outs = [out0, out1, out2, out3, out4, out5, out6, out7, out8, out9, out10] if out_seperate \
          else layers.Concatenate(axis=-1, name="outputs")([out0, out1, out2, out3, out4, out5, out6, out7, out8, out9, out10])

    # Define the model
  model = keras.Model(inputs=[imgs, in1, in2, in3] if not no_rnn_state else [imgs, in1, in2], 
                      outputs=outs, 
                      name='modelB6')
  return model

def get_vision_model(img_shape):
  img_shape = np.array(img_shape)
  imgs = keras.Input(shape=img_shape, name="imgs")
    #--- imgs.shape = (None, 12, 128, 256)
  in0 = layers.Permute((2, 3, 1))(imgs)
  out = EffNet(in0)
  model = keras.Model(inputs=imgs, 
                      outputs=out, 
                      name='modelB6')
  return model

def get_policy_model(img_shape, desire_shape, traffic_convection_shape, rnn_state_shape, num_classes, 
                     out_seperate:bool = False, no_rnn_state:bool=False, memorize_unit:Literal["gru", "casual_attention"]="gru",
                     vision_weights:str = "/home/neil/aJLL/Model/saved_model/B6vBW.weights.h5"):
  vision_model = get_vision_model(img_shape)
  vision_model.load_weights(vision_weights)
  vision_model.trainable = False

  in1 = keras.Input(shape=desire_shape, name="desire")
  in2 = keras.Input(shape=traffic_convection_shape, name="traffic_convection")
  if not no_rnn_state:
    in3 = keras.Input(shape=rnn_state_shape, name="rnn_state")

  if not no_rnn_state:
    out11 = RNN(vision_model.output, in1, in2, in3)
  else:
    if memorize_unit == "gru":
      out11 = gru(vision_model.output, in1, in2)
    elif memorize_unit == "casual_attention":
      out11 = casual_attention(vision_model.output, in1, in2)
    else:
      raise ValueError("memorize_unit must be 'gru' or 'casual_attention'")

  out0, out1, out2, out3, out4, out5, out6, out7 = fork1(out11)
  out8, out9 = fork2(vision_model.output)
  out10      = fork3(vision_model.output)
  
  if not no_rnn_state:
    outs = [out0, out1, out2, out3, out4, out5, out6, out7, out8, out9, out10, out11] if out_seperate \
          else layers.Concatenate(axis=-1, name="outputs")([out0, out1, out2, out3, out4, out5, out6, out7, out8, out9, out10, out11])
  else:
    outs = [out0, out1, out2, out3, out4, out5, out6, out7, out8, out9, out10] if out_seperate \
          else layers.Concatenate(axis=-1, name="outputs")([out0, out1, out2, out3, out4, out5, out6, out7, out8, out9, out10])

    # Define the model
  model = keras.Model(inputs=[vision_model.input, in1, in2, in3] if not no_rnn_state else [vision_model.input, in1, in2], 
                      outputs=outs, 
                      name='modelB6')
  return model

def casual_attention_2feature(feature0, feature1, desire, traffic_convection):
  x0 = layers.Concatenate(axis=-1)([desire, traffic_convection, feature0])
  x0 = layers.Reshape((1, -1))(x0)
  x1 = layers.Concatenate(axis=-1)([desire, traffic_convection, feature1])
  x1 = layers.Reshape((1, -1))(x1)
  shape_to_1024 = layers.Dense(units=1024)
  x = layers.Concatenate(axis=1)([shape_to_1024(x0), shape_to_1024(x1)])

  x = MaskedCausalAttentionBlock(h_dim=1024, max_T=2, n_heads=4, drop_p=0.1)(x)
  # x = MaskedCausalAttentionBlock(h_dim=1024, max_T=2, n_heads=4, drop_p=0.1)(x)
  out11 = tf.reshape(x, [-1, 1024])
  return out11

def top1(x0, idx):
    #--- x0.shape = (None, 4, 8, 1408)
  x = layers.Conv2D(32, 1, padding="same", name=f"top_conv{idx}")(x0)
    #--- x0.shape = (None, 4, 8, 32)
  x = layers.Activation("elu", name=f"top_activation{idx}")(x)
  x = layers.Flatten()(x)
    #--- x0.shape = (None, 1024)
  return x

def get_causal_attention_model(img_shape, desire_shape, traffic_convection_shape):
  img_shape = np.array(img_shape)
  img0 = keras.Input(shape=img_shape, name="img0")
  img1 = keras.Input(shape=img_shape, name="img1")
    #--- imgs.shape = (None, 6, 128, 256)
  in00 = layers.Permute((2, 3, 1))(img0)
  in01 = layers.Permute((2, 3, 1))(img1)

  vm = EffNetB2(name="effnetb2")

  feature00 = top1(vm(in00), 0)
  feature01 = top1(vm(in01), 1)

  in1 = keras.Input(shape=desire_shape, name="desire")
  in2 = keras.Input(shape=traffic_convection_shape, name="traffic_convection")

  out11 = casual_attention_2feature(feature00, feature01, in1, in2)
  out0, out1, out2, out3, out4, out5, out6 = fork1(out11, no_desire_state=True)
  # out8, out9 = fork2(features)
  # out10      = fork3(features)

  outs = layers.Concatenate(axis=-1, name="outputs")([
         out0, out1, out2, out3, out4, out5, out6]) #, out8, out9, out10

  model = keras.Model(inputs=[img0, img1, in1, in2], 
                      outputs=outs, 
                      name='test_model')
  return model

class EffNetB2(layers.Layer):
    def __init__(self, name=None, **kwargs):
        super(EffNetB2, self).__init__(name=name, **kwargs)
        self.stem = keras.Sequential([
                    layers.Conv2D(32, 3, strides=2, padding="same", name="stem_conv"),
                    layers.BatchNormalization(axis=3),
                    layers.Activation("elu", name="stem_activation")])
        self.block1a = keras.Sequential([
           layers.DepthwiseConv2D(3, depth_multiplier=1, padding="same", name="block1a_dwconv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block1a_activation"),
           layers.Conv2D(16, 1, padding="same", name="block1a_project_conv"),
           layers.BatchNormalization(axis=3)])
        self.block1b = keras.Sequential([
           layers.DepthwiseConv2D(3, depth_multiplier=1, padding="same", name="block1b_dwconv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block1b_activation"),
           layers.Conv2D(16, 1, padding ="same", name="block1b_project_conv"),
           layers.BatchNormalization(axis=3)])
        count = "2"
        self.block2a = keras.Sequential([
          layers.Conv2D(96, 1, padding="same", name="block"+count+"a_expand_conv"),
          layers.BatchNormalization(axis=3),
          layers.Activation("elu", name="block"+count+"a_expand_activation"),
          layers.DepthwiseConv2D(3, depth_multiplier=1, strides=2, padding="same", name="block"+count+"a_dwconv"),
          layers.BatchNormalization(axis=3),
          layers.Activation("elu", name="block"+count+"a_activation"),
          layers.Conv2D(24, 1, padding="same", name="block"+count+"a_project_conv"),
          layers.BatchNormalization(axis=3)])
        self.block2b = keras.Sequential([
          layers.Conv2D(144, 1, padding="same", name="block"+count+"b_expand_conv"),
          layers.BatchNormalization(axis=3),
          layers.Activation("elu", name="block"+count+"b_expand_activation"),
          layers.DepthwiseConv2D(3, depth_multiplier=1, padding="same", name="block"+count+"b_dwconv"),
          layers.BatchNormalization(axis=3),
          layers.Activation("elu", name="block"+count+"b_activation"),
          layers.Conv2D(24, 1, padding="same", name="block"+count+"b_project_conv"),
          layers.BatchNormalization(axis=3)])
        self.block2c = keras.Sequential([
          layers.Conv2D(144, 1, padding="same", name="block"+count+"c_expand_conv"),
          layers.BatchNormalization(axis=3),
          layers.Activation("elu", name="block"+count+"c_expand_activation"),
          layers.DepthwiseConv2D(3, depth_multiplier=1, padding="same", name="block"+count+"c_dwconv"),
          layers.BatchNormalization(axis=3),
          layers.Activation("elu", name="block"+count+"c_activation"),
          layers.Conv2D(24, 1, padding="same", name="block"+count+"c_project_conv"),
          layers.BatchNormalization(axis=3)])
        count="3"
        self.block3a = keras.Sequential([
           layers.Conv2D(144, 1, padding="same", name="block"+count+"a_expand_conv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"a_expand_activation"),
           layers.DepthwiseConv2D(5, depth_multiplier=1, strides=2, padding="same", name="block"+count+"a_dwconv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"a_activation"),
           layers.Conv2D(48, 1, padding="same", name="block"+count+"a_project_conv"),
           layers.BatchNormalization(axis=3)])
        self.block3b = keras.Sequential([
           layers.Conv2D(288, 1, padding="same", name="block"+count+"b_expand_conv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"b_expand_activation"),
           layers.DepthwiseConv2D(5, depth_multiplier=1, padding="same", name="block"+count+"b_dwconv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"b_activation"),
           layers.Conv2D(48, 1, padding="same", name="block"+count+"b_project_conv"),
           layers.BatchNormalization(axis=3)])
        self.block3c = keras.Sequential([
           layers.Conv2D(288, 1, padding="same", name="block"+count+"c_expand_conv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"c_expand_activation"),
           layers.DepthwiseConv2D(5, depth_multiplier=1, padding="same", name="block"+count+"c_dwconv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"c_activation"),
           layers.Conv2D(48, 1, padding="same", name="block"+count+"c_project_conv"),
           layers.BatchNormalization(axis=3)])
        count="4"
        self.block4a = keras.Sequential([
           layers.Conv2D(288, 1, padding="same", name="block"+count+"a_expand_conv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"a_expand_activation"),
           layers.DepthwiseConv2D(3, depth_multiplier=1, strides=2, padding="same", name="block"+count+"a_dwconv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"a_activation"),
           layers.Conv2D(88, 1, padding="same", name="block"+count+"a_project_conv"),
           layers.BatchNormalization(axis=3)])
        self.block4b = keras.Sequential([
           layers.Conv2D(528, 1, padding="same", name="block"+count+"b_expand_conv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"b_expand_activation"),
           layers.DepthwiseConv2D(3, depth_multiplier=1, padding="same", name="block"+count+"b_dwconv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"b_activation"),
           layers.Conv2D(88, 1, padding="same", name="block"+count+"b_project_conv"),
           layers.BatchNormalization(axis=3)])
        self.block4c = keras.Sequential([
           layers.Conv2D(528, 1, padding="same", name="block"+count+"c_expand_conv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"c_expand_activation"),
           layers.DepthwiseConv2D(3, depth_multiplier=1, padding="same", name="block"+count+"c_dwconv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"c_activation"),
           layers.Conv2D(88, 1, padding="same", name="block"+count+"c_project_conv"),
           layers.BatchNormalization(axis=3)])
        self.block4d = keras.Sequential([
           layers.Conv2D(528, 1, padding="same", name="block"+count+"d_expand_conv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"d_expand_activation"),
           layers.DepthwiseConv2D(3, depth_multiplier=1, padding="same", name="block"+count+"d_dwconv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"d_activation"),
           layers.Conv2D(88, 1, padding="same", name="block"+count+"d_project_conv"),
           layers.BatchNormalization(axis=3)])
        count = "5"
        self.block5a = keras.Sequential([
           layers.Conv2D(528, 1, padding="same", name="block"+count+"a_expand_conv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"a_expand_activation"),
           layers.DepthwiseConv2D(5, depth_multiplier=1, strides=2, padding="same", name="block"+count+"a_dwconv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"a_activation"),
           layers.Conv2D(120, 1, padding="same", name="block"+count+"a_project_conv"),
           layers.BatchNormalization(axis=3)])
        self.block5b = keras.Sequential([
           layers.Conv2D(720, 1, padding="same", name="block"+count+"b_expand_conv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"b_expand_activation"),
           layers.DepthwiseConv2D(5, depth_multiplier=1, padding="same", name="block"+count+"b_dwconv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"b_activation"),
           layers.Conv2D(120, 1, padding="same", name="block"+count+"b_project_conv"),
           layers.BatchNormalization(axis=3)])
        self.block5c = keras.Sequential([
           layers.Conv2D(720, 1, padding="same", name="block"+count+"c_expand_conv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"c_expand_activation"),
           layers.DepthwiseConv2D(5, depth_multiplier=1, padding="same", name="block"+count+"c_dwconv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"c_activation"),
           layers.Conv2D(120, 1, padding="same", name="block"+count+"c_project_conv"),
           layers.BatchNormalization(axis=3)])
        self.block5d = keras.Sequential([
           layers.Conv2D(720, 1, padding="same", name="block"+count+"d_expand_conv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"d_expand_activation"),
           layers.DepthwiseConv2D(5, depth_multiplier=1, padding="same", name="block"+count+"d_dwconv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"d_activation"),
           layers.Conv2D(120, 1, padding="same", name="block"+count+"d_project_conv"),
           layers.BatchNormalization(axis=3)])
        count = "6"
        self.block6a = keras.Sequential([
           layers.Conv2D(720, 1, padding="same", name="block"+count+"a_expand_conv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"a_expand_activation"),
           layers.DepthwiseConv2D(5, depth_multiplier=1, strides=2, padding="same", name="block"+count+"a_dwconv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"a_activation"),
           layers.Conv2D(208, 1, padding="same", name="block"+count+"a_project_conv"),
           layers.BatchNormalization(axis=3)])
        self.block6b = keras.Sequential([
           layers.Conv2D(1248, 1, padding="same", name="block"+count+"b_expand_conv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"b_expand_activation"),
           layers.DepthwiseConv2D(5, depth_multiplier=1, padding="same", name="block"+count+"b_dwconv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"b_activation"),
           layers.Conv2D(208, 1, padding="same", name="block"+count+"b_project_conv"),
           layers.BatchNormalization(axis=3)])
        self.block6c = keras.Sequential([
           layers.Conv2D(1248, 1, padding="same", name="block"+count+"c_expand_conv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"c_expand_activation"),
           layers.DepthwiseConv2D(5, depth_multiplier=1, padding="same", name="block"+count+"c_dwconv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"c_activation"),
           layers.Conv2D(208, 1, padding="same", name="block"+count+"c_project_conv"),
           layers.BatchNormalization(axis=3)])
        self.block6d = keras.Sequential([
           layers.Conv2D(1248, 1, padding="same", name="block"+count+"d_expand_conv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"d_expand_activation"),
           layers.DepthwiseConv2D(5, depth_multiplier=1, padding="same", name="block"+count+"d_dwconv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"d_activation"),
           layers.Conv2D(208, 1, padding="same", name="block"+count+"d_project_conv"),
           layers.BatchNormalization(axis=3)])
        self.block6e = keras.Sequential([
           layers.Conv2D(1248, 1, padding="same", name="block"+count+"e_expand_conv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"e_expand_activation"),
           layers.DepthwiseConv2D(5, depth_multiplier=1, padding="same", name="block"+count+"e_dwconv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"e_activation"),
           layers.Conv2D(208, 1, padding="same", name="block"+count+"e_project_conv"),
           layers.BatchNormalization(axis=3)])
        count = "7"
        self.block7a = keras.Sequential([
           layers.Conv2D(1248, 1, padding="same", name="block"+count+"a_expand_conv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"a_expand_activation"),
           layers.DepthwiseConv2D(3, depth_multiplier=1, strides=2, padding="same", name="block"+count+"a_dwconv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"a_activation"),
           layers.Conv2D(352, 1, padding="same", name="block"+count+"a_project_conv"),
           layers.BatchNormalization(axis=3)])
        self.block7b = keras.Sequential([
           layers.Conv2D(2112, 1, padding="same", name="block"+count+"b_expand_conv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"b_expand_activation"),
           layers.DepthwiseConv2D(3, depth_multiplier=1, padding="same", name="block"+count+"b_dwconv"),
           layers.BatchNormalization(axis=3),
           layers.Activation("elu", name="block"+count+"b_activation"),
           layers.Conv2D(352, 1, padding="same", name="block"+count+"b_project_conv"),
           layers.BatchNormalization(axis=3)])
        self.top = keras.Sequential([
           layers.Conv2D(1408, 1, padding="same", name="top_conv"),
           layers.Activation("elu", name="top_activation")])
    
    def call(self, x):
        x = self.stem(x)
          #--- x.shape = (None, 64, 128, 32)
        x = self.block1a(x)
        tmp = self.block1b(x)
        x = layers.add([tmp,x], name = "block1b_add")

        x = self.block2a(x)
        tmp = self.block2b(x)
        x = layers.add([tmp,x], name="block2b_add")
        tmp = self.block2c(x)
        x = layers.add([tmp,x], name="block2c_add")

        x = self.block3a(x)
        tmp = self.block3b(x)
        x = layers.add([tmp,x], name="block3b_add")
        tmp = self.block3c(x)
        x = layers.add([tmp,x], name="block3c_add")

        x = self.block4a(x)
        tmp = self.block4b(x)
        x = layers.add([tmp,x], name="block4b_add")
        tmp = self.block4c(x)
        x = layers.add([tmp,x], name="block4c_add")
        tmp = self.block4d(x)
        x = layers.add([tmp,x], name="block4d_add")

        x = self.block5a(x)
        tmp = self.block5b(x)
        x = layers.add([tmp,x], name="block5b_add")
        tmp = self.block5c(x)
        x = layers.add([tmp,x], name="block5c_add")
        tmp = self.block5d(x)
        x = layers.add([tmp,x], name="block5d_add")

        x = self.block6a(x)
        tmp = self.block6b(x)
        x = layers.add([tmp,x], name="block6b_add")
        tmp = self.block6c(x)
        x = layers.add([tmp,x], name="block6c_add")
        tmp = self.block6d(x)
        x = layers.add([tmp,x], name="block6d_add")
        tmp = self.block6e(x)
        x = layers.add([tmp,x], name="block6e_add")

        x = self.block7a(x)
        tmp = self.block7b(x)
        x = layers.add([tmp,x], name="block7b_add")

          #--- x.shape = (None, 4, 8, 352)
        out = self.top(x)
          #--- x.shape = (None, 4, 8, 1408)
        # x = layers.Conv2D(32, 1, padding="same")(x)
          #--- x.shape = (None, 4, 8, 32)
        # x = layers.Activation("elu")(x)
          #--- x.shape = (None, 4, 8, 32)
        # out = layers.Flatten()(x)
          #--- out.shape = (None, 1024)
        return out

if __name__=="__main__":
    # Build model
  img_shape = (12, 128, 256)
  desire_shape = (8)
  traffic_convection_shape = (2)
  rnn_state_shape = (512)
  num_classes = 6

  model = get_model(img_shape, desire_shape, traffic_convection_shape, rnn_state_shape, num_classes)

  model.summary()
  #model.save('./saved_model/modelB6.h5')
  #print('#--- x0.shape =', x0.shape)


