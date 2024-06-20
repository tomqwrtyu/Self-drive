CB 6  Jinn-Liang Liu  2023.7.19 - 8.29  All Rights Reserved. © Copyright 2023

========== OP Coding Notes: Run, Change, Read ==========

========== Summary of Coding Notes ==========

230826: NG RT 2: PC: Run "modelB6", "ModelOld"
--- Run "ModelOld" files
--- Download aJLL.zip and unzip to /openpilot
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/ModelOld$ pip3 install h5py
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/ModelOld$ pip3 install tensorflow
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/ModelOld$ pip show tensorflow
--- Run "modelB6"
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ python serverB6.py --port 5557
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ python serverB6.py --port 5558 --validation
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ python train_modelB6.py --port 5557 --port_val 5558
--- OK: Training Output
--- 220419 Training Output of "modelB5":
    1. 4/11: try w/o normalization
    4. 4/19: try the whole 2383 vector
       $ python simulatorB5.py => NG but better
      Conclusion:
         A: important to move rnn_1st = 0 up
         B: 1, 1148, 2 is needed
         C: 2383 is better than 50
    5. 4/19: try Huber loss with delta=1.0
      Conclusion: Huber loss with delta=1.0 is the best.
--- Run "train_plot.py"
--- OK: Output: /saved_model/modelB6_outs.png
--- Run "pose_plot.py"
--- OK: Output: pose_plot_path1.png
--- Run "simulatorB6.py"
--- OK: Output: simulatorB6_fig.png
--- RT 1: 220208 Road Test Error on modelB5C2C.dlc:

230825: OK PC: Add "PYTHONPATH" to ".bashrc" for running "aJLL/ModelOld/dbcJLL.py" (not /openpilot)
--- Add this to the end of .bashrc
    export PYTHONPATH=/home/jinn/openpilot
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Coding/py$ python zc.py
--- Run /openpilot/aJLL/ModelOld/dbcJLL.py
    (sconsvenv) jinn@Liu:~/openpilot$ python aJLL/ModelOld/dbcJLL.py

230822: OK UI: CBQ 6-8: PC: scons Run "./replayJLL --demo --data_dir" + "uiJLL.py", "uiviewJLL.py", or "/ui/ui"
--- For AM 1 + CBQ 2 + Lcz 1: 230619 Pipe 1f + 230725 Pipe 3a
    Reset "self.path_xyz =, modelV2.position, .orientation, .orientationRate" as in plantLatJLL.py
--- CBQ 6: How to draw "self.path_xyz" on PC ???
      draw_path(m.position, color, img, calibration, top_down, RED, 1.22)
--- A: by "ui.py", "uiviewJLL.py", or "/ui/ui"
--- Run "./replay --demo"
--- clean up "comma_download_cache"
--- Q: Need to scons "./replayJLL" again as 230311 ??? A: Yes.
--- Copy /tools/replay/"SConscript", "SConscriptJLL" to SConscriptLP092, "SConscript"
--- Change "SConstruct" to
    SConscript(['common/transformations/SConscript'])
--- scons "replayJLL"
--- OK: scons: done building targets.
--- Run "uiviewJLL.py" + "./replayJLL --data_dir"
--- Run "/ui/ui" + "./replayJLL --data_dir"
--- Create Run "uiJLL.py"
--- Install OpenCV
--- Run "uiJLL.py" + "./replayJLL --data_dir"
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiJLL.py
    (sconsvenv) jinn@Liu:~/openpilot/tools/replay$ ./replayJLL --data_dir dataC "8bfda98c9c9e4291|2020-05-11--03-00-57--61"
--- Output 1: /home/jinn/Screenshots/PC/230822/2023-08-24 17-39-06_UI_TW.png
--- Run "uiJLL.py" + "./replayJLL --demo"
--- Output 2: /home/jinn/Screenshots/PC/230822/2023-08-25 10-31-01_UI_USA.png
--- CBQ 7: Why Output 1 != Output 2 (Error in def draw_path) ???
--- CBQ 8: How to draw CB's "self.path_xyz" on C2 ???
--- Read "draw_path"

230814: OK PC: Run "plantLonJLL.py", "plantLatJLL.py"
--- For Lcz 1: Change "self.path_xyz = ... md.position.x" to "straight" for CBQ 2
--- I need to create a fake "md.position.x" as in /selfdrive/test/longitudinal_maneuvers/plant.py
    # ******** publish a fake model going straight and fake calibration ********
--- Run Read "plantLonJLL.py"
--- Pipe 1a: 'modelV2' > 'LongitudinalPlanner': self.planner = LongitudinalPlanner() >
    def plant_thread() > plant.step() > def step() > model.modelV2.position = position >
    sm = {'radarState': radar.radarState, 'carState': car_state.carState,
          'controlsState': control.controlsState, 'modelV2': model.modelV2} >
    self.planner.update(sm) > self.speed = self.planner.v_desired_filter.x,
    self.acceleration = self.planner.a_desired
--- Read "longitudinal_planner.py", "long_mpc.py"
--- Pipe 1b: class LongitudinalPlanner > self.mpc = LongitudinalMpc() > def update(self, sm) >
    x,,, = self.parse_model(sm['modelV2'], ) > self.mpc.update(sm['radarState'],,x,,,) >
    self.v_desired_filter.x
--- Write "plantLatJLL.py":
--- Pipe 2: 'modelV2' > 'LateralPlanner': self.planner = LateralPlanner() > def plant_thread() >
    plant.step() > def step() > model.modelV2.position = position >
    sm = {'carState': car_state.carState, 'controlsState': control.controlsState, 'modelV2': model.modelV2} >
    self.planner.update(sm) > self.curvature = self.planner.desired_curvature,
    self.curvature_rate = self.planner.desired_curvature_rate
--- Read 230619: "test_lateral_mpcJLL.py", "lateral_plannerJLL.py", "lat_mpc.py"
--- 230725 Pipe 3a: pm.send('lateralPlan', ) > def state_control() > lat_plan = self.sm['lateralPlan'] >
    self.desired_curvature,  = get_lag_adjusted_curvature(,,,lat_plan.curvatures,)

230725: OK AM 1: NG CBQ 2: XJ 8: Lcz 1: Pipe 1-6: PC+C2+body: Change "controlsd.py", "carcontroller.py"
--- Do AM 1: AR 2 + Md 1 (Path)
--- ToDo 1: 'modelV2' > 'lateralPlan': Run model_replay.py, /controls/lib/tests/test_latcontrol.py
--- Note: "llk" = 'liveLocationKalman' is not used by latcontrol_angle.py but by latcontrol_torque.py
--- Pipe 1a: car.capnp > CC.orientationNED > log.capnp > llk.orientationNED
--- Pipe 1b: locationd.cc > pm({"liveLocationKalman"}) > paramsd.py > sm.updated['liveLocationKalman'] >
    ParamsLearner > pm.send('liveParameters',) > controlsd.py > sm['liveParameters'] >
    pm.send('controlsState',) > "measured_curvature" > lateral_planner.py >
    LateralPlanner > 230619 Pipe 1f > lat_mpc.run() > x0[3] "measured_yaw_rate" >
    pm.send('lateralPlan', ) > plannerd.py > controlsd.py > sm['lateralPlan']
--- Pipe 1c: locationd.cc > paramsd.py > controlsd.py > lateral_planner.py > plannerd.py > controlsd.py
--- Pipe 1d: "liveLocationKalman" > ParamsLearner > 'liveParameters' > 'controlsState' >
    'measured_curvature' > LateralPlanner > LateralMpc > x0[3] "measured_yaw_rate"
--- XJ 8a: Change "latcontrol.py", "controlsd.py" from /body to /toyota for "self.LaC"
--- Pipe 2: /toyota, /body: latcontrol.py > def update() > latcontrol_angle.py > def update()
--- Pipe 3a: 230619 Pipe 1f > pm.send('lateralPlan', ) > def state_control() >
--- Pipe 3b: "measured_curvature" > LateralPlanner > LateralMpc > x0[3] "measured_yaw_rate" >
--- Pipe 3c: ParamsLearner > liveParameters.angleOffsetDeg > curvature > measured_curvature
--- ToDo 2: Write [Liu23b]: get_lag_adjusted_curvature(), get_steer_from_curvature()
--- Pipe 3d: paramsd.py > learner.kf.x > liveParameters.angleOffsetDeg > controlsd.py > controlsState.curvature
--- ToDo 3: for CBQ 2: Change class LatControlAngle update()
--- OK: "body" can standstill in hall way but keep going most of the time
--- XJ 8b: Change "/body/carcontroller.py"
--- Pipe 4a: self.CI, = get_car() > def data_sample() > CS = self.CI.update(self.CC, can_strs) >
--- Pipe 4b: def get_car() > return CarInterface(CP, CarController, CarState), CP
--- Pipe 4c: CarInterfaceBase(ABC) > def update(self, c: car.CarControl,
--- Pipe 4d: CarInterfaceBase(ABC) > def _update(): pass > CarInterface() > def _update(, c) >
--- Pipe 4e: CarState > def update(, cp) > ret.vEgo, ret.aEgo = update_speed_kf(ret.vEgoRaw)
--- Note: ret.vEgoCluster not updated in /body
--- ToDo 4: Run selfdrive/controls/tests/test_cruise_speed.py, /test_following_distance.py
--- ToDo 5: Run aOPold/dbcJL.py
--- XJ 8b 3): for CBQ 2:
--- NG: not straight, not standstill (going forward)
--- 4) for standstill:
--- Undo XJ 8b. Go back to XJ 8a 5A) 6). Save to controlsdXJ8.py, carcontrollerXJ8.py
--- OK but still like Ctr 3: (0,0) to (3.24, -0.57)m => ToDo 6 solution ???
--- ToDo 6: PIDController + CC.orientationNED + lat_mpc.py + test_lateral_mpcJLL.py (test_straight())
--- Pipe 5a: log = sm[service] > handle_msg(log) > get_message_bytes() > pm.send("liveLocationKalman",,) >
--- Pipe 5b: initLiveLocationKalman() > build_live_location(liveLoc) >
--- Pipe 5c: gpsOK > get_position_geodetic() > kf->get_x() > fix_ecef > fix_ecef_ecef >
--- Pipe 5d: log.capnp > coordinates.hpp > live_kf.h > locationd.cc > controlsd.py > carcontroller.py > controlsd.py
--- ToDo 7: Run /locationd/test/test_locationd.py, /tools/gpstest/test_gps.py
--- Pipe 6a: modeld.cc > main() > run_model() > model_publish() > fill_model() > pm.send("modelV2",)
--- Pipe 6b: fill_model() > best_plan > fill_plan(framed, best_plan) > pos_x[i] = plan.mean[i].position.x >
    fill_xyzt(,,pos_x,,,,,,) > xyzt.setX(to_kj_array_ptr(x))
--- Lcz 1: Change "self.path_xyz = ... md.position.x" to "straight w/o md" for CBQ 2

230719: OK AR 2: Md 1: XJ 7: Pipe 1: PC+C2+body: Change "controlsd.py", Rewrite "longcontrol.py", "latcontrol.py"
--- Md 1: 'modelV2' + Path: 'longitudinalPlan' + 'lateralPlan'
--- Do AM 1: AR 2 w/o Md 1 (Path)
--- Change 230521 Pipe 3 a) b) c) d) to
--- Pipe 1: def state_control(CS) > CC = car.CarControl > actuators = CC.actuators >
    a1) actuators.accel = 1.5*clip(), a2) steer = clip(), actuators.steer = steer
    b) self.LoC > actuators.accel = self.LoC.update() >
         longcontrol.py > def update() > a1) > self.pid.update() > pid.py > def update()
    c) self.LaC > actuators.steer = self.LaC.update() >
         latcontrol.py > def update() > a2) > latcontrol_angle.py > def update()
    d) unchanged
--- Note 1: "actuators.steer" is for body not cars
--- Note 2: "lac_log.output", "LateralAngleState", "LateralDebugState"
--- Note 3: "steerControlType" > "LatControlAngle"
--- Note 4: Key: ".steer =", ".accel ="
--- XJ 7a: Do Pipe 1 b) > a1): self.LoC: Change "controlsd.py", Rewrite "longcontrol.py"
    1) save C2/../longcontrol.py to LPXJ/../longcontrolLP092.py
--- Q: What are dashcamOnly, self.read_only, passive, controller_available, "OpenpilotEnabledToggle" for ???
    A: dashcamOnly > self.read_only > dashcam mode; not openpilot_enabled_toggle > passive >
--- XJ 7b: Do Pipe 1 c) > a2): self.LaC: Change "controlsd.py", Rewrite "latcontrol.py"
--- 1) Save C2/../latcontrol.py to LPXJ/../latcontrolLP092.py; Rewrite "latcontrol.py" to
--- OK: Run "body"

===== Details of Coding Notes =====

230826: NG RT 2: PC: Run "modelB6", "ModelOld"
--- Run "ModelOld" files
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/ModelOld$ python dbcJLL.py
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/ModelOld$ python test_clustering_tutor.py
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/ModelOld$ python test_clustering.py
--- Download aJLL.zip and unzip to /openpilot
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ pip3 install h5py
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ pip3 install tensorflow
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ pip show tensorflow
      Version: 2.13.0
--- Run "modelB6"
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ python serverB6.py --port 5557
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ python serverB6.py --port 5558 --validation
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ python train_modelB6.py --port 5557 --port_val 5558
--- OK: Training Output on Terminal 3
      2023-08-26 15:38:42.901855: I tensorflow/tsl/cuda/cudart_stub.cc:28] Could not find cuda drivers on your machine, GPU will not be used.
      2023-08-26 15:38:42.944989: I tensorflow/core/platform/cpu_feature_guard.cc:182]
      This TensorFlow binary is optimized to use available CPU instructions in performance-critical operations.
      To enable the following instructions: AVX2 FMA, in other operations, rebuild TensorFlow with the appropriate compiler flags.
      2023-08-26 15:38:43.620406: W tensorflow/compiler/tf2tensorrt/utils/py_utils.cc:38] TF-TRT Warning: Could not find TensorRT
      tmp.shape= [None, 64, 128, 32]
      Epoch 1/2
      1/1 [==============================] - 0s 77ms/step:24:59
      - loss: 11.0009 - rmse: 25.6188 - mae: 11.3915
      - loss: 8.5005 - rmse: 21.2714 - mae: 8.9040
      - loss: 14.3867 - rmse: 26.7530 - mae: 14.8167
      - loss: 9.2213 - rmse: 147.7048 - mae: 9.4866
      - loss: 9.2136 - rmse: 147.5780 - mae: 9.4788
      Epoch 1: val_loss improved from inf to 1.40289, saving model to ./saved_model/modelB6-BestWeights.hdf5
      /home/jinn/sconsvenv/lib/python3.8/site-packages/keras/src/engine/training.py:3000:
      UserWarning: You are saving your model as an HDF5 file via `model.save()`.
      This file format is considered legacy. We recommend using instead the native Keras format,
      e.g. `model.save('my_model.keras')`.
        saving_api.save_model(
      1148/1148 [==============================] - 5452s 5s/step
      - loss: 9.2136 - rmse: 147.5780 - mae: 9.4788 - val_loss: 1.4029 - val_rmse: 4.1392 - val_mae: 1.7139
      Epoch 2/2
      1/1 [==============================] - 0s 132ms/step33
      - loss: 0.4614 - rmse: 2.4790 - mae: 0.6965
      - loss: 0.4565 - rmse: 2.4697 - mae: 0.6792
      - loss: 3.2789 - rmse: 9.4188 - mae: 3.5167
      - loss: 3.2795 - rmse: 9.4202 - mae: 3.5174
      Epoch 2: val_loss did not improve from 1.40289
      1148/1148 [==============================] - 11305s 10s/step
      - loss: 3.2795 - rmse: 9.4202 - mae: 3.5174 - val_loss: 4.2989 - val_rmse: 11.1778 - val_mae: 4.6384
      Training Time: 04:39:54.41
--- OK: Training Output on Terminal 1
      #--- i, count, dataN, count+ranIdx[i] = 45 0 1150 45
      #--- batchIndx = 2340
      #--- i, count, dataN, count+ranIdx[i] = 46 0 1150 46
      #--- batchIndx = 2341
--- OK: Validation Output on Terminal 2
      #--- batchIndx = 2330
      #--- i, count, dataN, count+ranIdx[i] = 36 0 1150 36
      #--- batchIndx = 2331
--- 220419 Training Output of "modelB5":
    1. 4/11: try w/o normalization
       loss: 138.6146 - rmse: 11.6598 - mae: 5.8961 - val_loss: 112.0994 - val_rmse: 10.3778 - val_mae: 5.6667
       Training Time: 00:01:32.09
       $ python simulatorB5.py => NG
    2. 4/11: try w/ normalization
       loss: 0.0160 - rmse: 12.5430 - mae: 6.3640 - val_loss: 0.0127 - val_rmse: 11.0533 - val_mae: 6.0215
       Training Time: 00:01:31.91
       $ python simulatorB5.py => NG
      Conclusion: w/o or w/ normalization makes no difference
      BATCH_SIZE, STEPS, EPOCHS = 1, 1148, 2; w/o normalization
    3. 4/18: try the first 50 few vector
       YTpath = y_true[:, 50]  # try the first 50 few vector
       rnn_1st = 0  # for 1st image in vedio using Xin3 = np.zeros((batch_size, 512)
       for i in range(0, len(ranIdx), batch_size):
       Epoch 1/2
       #--- i, count, dataN, count+ranIdx[i] = 256 0 1150 256   Killed
       256/1148 [=====>........................] - ETA: 48:06 -
       loss: 14.0165 - rmse: 2.8667 - mae: 1.4380
       $ python simulatorB5.py => NG
    4. 4/19: try the whole 2383 vector
       loss = tf.keras.losses.mse(y_true, y_pred)  # for whole 2383 vector
       Epoch 1/2
       #--- i, count, dataN, count+ranIdx[i] = 256 0 1150 256   Killed
       260/1148 [=====>........................] - ETA: 44:56 -
       loss: 13.8437 - rmse: 2.8483 - mae: 1.4290
       $ python simulatorB5.py => NG but better
      Conclusion:
         A: important to move rnn_1st = 0 up
         B: 1, 1148, 2 is needed
         C: 2383 is better than 50
    5. 4/19: try Huber loss with delta=1.0
       #--- i, count, dataN, count+ranIdx[i] = 253 0 1150 253   Killed
       253/1148 [=====>........................] - ETA: 44:39 -
       loss: 0.8905 - rmse: 2.9524 - mae: 1.1404
         delta=2.0
       #--- i, count, dataN, count+ranIdx[i] = 255 0 1150 255   Killed
       255/1148 [=====>........................] - ETA: 49:05 -
       loss: 1.4773 - rmse: 2.8519 - mae: 1.1421
      Conclusion: Huber loss with delta=1.0 is the best.
--- Run "train_plot.py"
--- OK: Output: /saved_model/modelB6_outs.png
--- Run "pose_plot.py"
--- OK: Output: pose_plot_path1.png
--- Run "simulatorB6.py"
--- OK: Output: simulatorB6_fig.png
--- Road Test Errors
    https://drive.google.com/file/d/1z84Mc0vYiwnmqQztx5vnIdeOJwrYUW9b/view
    RT 1: 220208 Road Test Error on modelB5C2C.dlc:
      1. WARNING: This branch is not tested
         <= EventName.startupMaster (events.py) <= def get_startup_event (car_helpers.py)
         <= tested_branch = False (version.py <= manager.py)
         <= get_startup_event (controlsd.py)
    RT 2: 220426 Road Test Error on B5YJ0421.dlc: "No close lead car"
      1. WARNING: This branch is not tested
      2. openpilot Canceled  No close lead car
      3. openpilot Unavailable  No Close Lead Car
         <= EventName.noTarget: ET.NO_ENTRY : NoEntryAlert("No Close Lead Car") (events.py)
         <= self.events.add(EventName.noTarget) (controlsd.py)
      4. openpilot Unavailable  Planner Solution Error

230825: OK PC: Add "PYTHONPATH" to ".bashrc" for running "aJLL/ModelOld/dbcJLL.py" (not /openpilot)
--- From CAN 32 Run
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Coding/py$ python zc.py
--- Error
    ModuleNotFoundError: No module named 'common'
--- "Symbolic Link" cannot solve this Error, so remove it
    (sconsvenv) jinn@Liu:~/openpilot$ find ./ -type l -exec file {} \; |grep broken
      ./aJLL/Coding/py/apy: broken symbolic link to /apy
    (sconsvenv) jinn@Liu:~/openpilot$ rm ./aJLL/Coding/py/apy
--- PYTHONPATH is an environment variable which you can set to add additional
      directories where python will look for modules and packages.
    (sconsvenv) jinn@Liu:~/openpilot$ echo $PYTHONPATH
--- Nothing
    jinn@Liu:~$ gedit ~/.bashrc
--- Add this to the end of .bashrc
    export PYTHONPATH=/home/jinn/openpilot
--- Close the terminal
--- Restart the terminal
    (sconsvenv) jinn@Liu:~/openpilot$ echo $PYTHONPATH
      /home/jinn/openpilot
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Coding/py$ python zc.py
--- OK
--- Run /openpilot/aJLL/ModelOld/dbcJLL.py
    (sconsvenv) jinn@Liu:~/openpilot$ python aJLL/ModelOld/dbcJLL.py
--- OK

230822: OK UI: CBQ 6,7: PC: scons Run "./replayJLL --demo --data_dir" + "uiJLL.py", "uiviewJLL.py", or "/ui/ui"
--- For AM 1 + CBQ 2 + Lcz 1: 230619 Pipe 1f + 230725 Pipe 3a
    Reset "self.path_xyz =, modelV2.position, .orientation, .orientationRate" as in plantLatJLL.py
--- CBQ 6: How to draw "self.path_xyz" on PC ???
    class LateralPlanner:
      self.path_xyz = np.column_stack([md.position.x, md.position.y, md.position.z])
    tools/replay/ui.py
      plot_model(sm['modelV2'], img, calibration, top_down)
      def plot_model(m, img, calibration, top_down):
        draw_path(m.position, color, img, calibration, top_down, RED, 1.22)
--- A: by "ui.py", "uiviewJLL.py", or "/ui/ui"
--- Run "./replay --demo"
    (sconsvenv) jinn@Liu:~/openpilot$ ./selfdrive/ui/ui
    (sconsvenv) jinn@Liu:~/openpilot/tools/replay$ ./replay --demo
--- OK
--- clean up "comma_download_cache"
--- Run "./replay --data_dir"
    (sconsvenv) jinn@Liu:~/openpilot/tools/replay$
    ./replay --data_dir dataC "8bfda98c9c9e4291|2020-05-11--03-00-57--61"
--- Error
    failed to load route "8bfda98c9c9e4291|2020-05-11--03-00-57" from "dataC"
--- Rename to /home/jinn/openpilot/tools/replay/dataC2
--- Move /home/jinn/dataC to /home/jinn/openpilot/tools/replay/dataC
--- Error same
--- Undo dataC, dataC2
--- Run "./replayJLL --data_dir"
    (sconsvenv) jinn@Liu:~/openpilot/tools/replay$
    ./replayJLL --data_dir dataC "8bfda98c9c9e4291|2020-05-11--03-00-57--61"
--- OK but
    (sconsvenv) jinn@Liu:~/openpilot$ ./selfdrive/ui/ui
--- Error: "size > 0": UI screen disappeared
    _ui: cereal/messaging/msgq.cc:386: int msgq_msg_recv(msgq_msg_t *, msgq_queue_t *): Assertion `size > 0' failed.
    Aborted (core dumped)
--- Exchange the order:
    (sconsvenv) jinn@Liu:~/openpilot$ ./selfdrive/ui/ui
    (sconsvenv) jinn@Liu:~/openpilot/tools/replay$
    ./replayJLL --data_dir dataC "8bfda98c9c9e4291|2020-05-11--03-00-57--61"
--- Error same
--- Run "uiviewJLL.py"
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiviewJLL.py
    (sconsvenv) jinn@Liu:~/openpilot/tools/replay$
    ./replayJLL --data_dir dataC "8bfda98c9c9e4291|2020-05-11--03-00-57--61"
--- Error same
    Killing old publisher: deviceState
    Traceback (most recent call last):
      File "uiviewJLL.py", line 30, in <module>
        pm.send(s, msgs[s])  # this in while loop gives UserWarning
      File "/home/jinn/openpilot/cereal/messaging/__init__.py", line 259, in send
        self.sock[s].send(dat)
      File "cereal/messaging/messaging_pyx.pyx", line 149, in cereal.messaging.messaging_pyx.PubSocket.send
        raise MultiplePublishersError
    cereal.messaging.messaging_pyx.MultiplePublishersError
    _ui: cereal/messaging/msgq.cc:386: int msgq_msg_recv(msgq_msg_t *, msgq_queue_t *): Assertion `size > 0' failed.
--- Power Off and Redo
--- Error same
--- Change uiviewJLL.py to
      #for s in msgs:
      #  pm.send(s, msgs[s])  # this in while loop gives UserWarning
--- Error same
--- Run "./replayJLL --demo" + "uiviewJLL.py"
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiviewJLL.py
    (sconsvenv) jinn@Liu:~/openpilot/tools/replay$ ./replayJLL --demo
--- Error same
--- Run "./replayJLL --demo" + "./selfdrive/ui/ui"
    (sconsvenv) jinn@Liu:~/openpilot$ ./selfdrive/ui/ui
    (sconsvenv) jinn@Liu:~/openpilot/tools/replay$ ./replayJLL --demo
--- Error same
--- Q: Need to scons "./replayJLL" again as 230311 ??? A: Yes.
--- Copy /tools/replay/"SConscript", "SConscriptJLL" to SConscriptLP092, "SConscript"
--- Change "SConstruct"
    /openpilot/SConstruct
      # 230309 build /tools/replay/mainJLL.cc
      SConscript(['tools/replay/SConscript'])
--- scons "replayJLL"
    (sconsvenv) jinn@Liu:~/openpilot$ scons
--- Error
    scons: *** Import of non-existent variable ''transformations''
    File "/home/jinn/openpilot/tools/replay/SConscript", line 6, in <module>
--- Change "SConstruct" to
    SConscript(['common/transformations/SConscript'])
--- OK: scons: done building targets.
--- Run "uiviewJLL.py" + "./replayJLL --data_dir"
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiviewJLL.py
    (sconsvenv) jinn@Liu:~/openpilot/tools/replay$
    ./replayJLL --data_dir dataC "8bfda98c9c9e4291|2020-05-11--03-00-57--61"
--- OK
--- Run "/ui/ui" + "./replayJLL --data_dir"
    (sconsvenv) jinn@Liu:~/openpilot$ ./selfdrive/ui/ui
    (sconsvenv) jinn@Liu:~/openpilot/tools/replay$
    ./replayJLL --data_dir dataC "8bfda98c9c9e4291|2020-05-11--03-00-57--61"
--- OK
--- Create Run "uiJLL.py"
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiJLL.py
--- Error
    File "uiJLL.py", line 6, in <module>
      import cv2  # pylint: disable=import-error
    File "/home/jinn/sconsvenv/lib/python3.8/site-packages/cv2/__init__.py", line 8, in <module>
      from .cv2 import *
    ImportError: libnppc.so.11: cannot open shared object file: No such file or directory
--- install pygame
--- Error same. ToDo: see InstallOPOld.docx
--- Change "uiJLL.py" to
    #import cv2  # pylint: disable=import-error
      #cv2.setNumThreads(1)
        #bgr = cv2.cvtColor(imgff, cv2.COLOR_YUV2RGB_NV12)
        #cv2.warpAffine(bgr, zoom_matrix[:2], (img.shape[1], img.shape[0]), dst=img, flags=cv2.WARP_INVERSE_MAP)
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiJLL.py
    (sconsvenv) jinn@Liu:~/openpilot/tools/replay$ ./replayJLL --data_dir dataC "8bfda98c9c9e4291|2020-05-11--03-00-57--61"
--- OK but without vedio replay
--- Install OpenCV
    (sconsvenv) jinn@Liu:~/openpilot$ sudo apt update
    (sconsvenv) jinn@Liu:~/openpilot$ sudo apt-get install python3-pip
    (sconsvenv) jinn@Liu:~/openpilot$ pip3 install opencv-python
    (sconsvenv) jinn@Liu:~/openpilot$ python3 -c "import cv2; print(cv2.__version__)"
      4.8.0
--- Run "ui.py"
    (sconsvenv) jinn@Liu:~/openpilot$ python tools/replay/ui.py
    (sconsvenv) jinn@Liu:~/openpilot/tools/replay$ python ui.py
    (sconsvenv) jinn@Liu:~/openpilot/tools/replay$ ./replayJLL --data_dir dataC "8bfda98c9c9e4291|2020-05-11--03-00-57--61"
--- Error
    ModuleNotFoundError: No module named 'cereal'
--- Run "uiJLL.py" + "./replayJLL --data_dir"
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiJLL.py
    (sconsvenv) jinn@Liu:~/openpilot/tools/replay$ ./replayJLL --data_dir dataC "8bfda98c9c9e4291|2020-05-11--03-00-57--61"
--- OK w/ vedio
--- Output 1: /home/jinn/Screenshots/PC/230822/2023-08-24 17-39-06_UI_TW.png
--- Run "uiJLL.py" + "./replayJLL --demo"
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/UI$ python uiJLL.py
    (sconsvenv) jinn@Liu:~/openpilot/tools/replay$ ./replayJLL --demo
--- Output 2: /home/jinn/Screenshots/PC/230822/2023-08-25 10-31-01_UI_USA.png
--- CBQ 7: Why Output 1 != Output 2 (Error in def draw_path) ???
--- CBQ 8: How to draw CB's "self.path_xyz" on C2 ???
--- Read "draw_path"

230814: OK PC: Run "plantLonJLL.py", "plantLatJLL.py"
--- For Lcz 1: Change "self.path_xyz = ... md.position.x" to "straight" for CBQ 2
    self.path_xyz = np.column_stack([md.position.x, md.position.y, md.position.z])
    from selfdrive.controls.lib.lateral_planner import LateralPlanner
--- Change "lateral_plannerJLL.py" to include "plot" as "Fig1MPC1.png"
--- Run "plannerdJLL.py"
--- Postpone "plannerdJLL.py" + "lateral_plannerJLL.py"
--- I need to create a fake "md.position.x" as in /selfdrive/test/longitudinal_maneuvers/plant.py
    # ******** publish a fake model going straight and fake calibration ********
--- Run Read "plantLonJLL.py"
--- Pipe 1a: 'modelV2' > 'LongitudinalPlanner': self.planner = LongitudinalPlanner() >
    def plant_thread() > plant.step() > def step() > model.modelV2.position = position >
    sm = {'radarState': radar.radarState, 'carState': car_state.carState,
          'controlsState': control.controlsState, 'modelV2': model.modelV2} >
    self.planner.update(sm) > self.speed = self.planner.v_desired_filter.x,
    self.acceleration = self.planner.a_desired
--- Read "longitudinal_planner.py", "long_mpc.py"
    class LongitudinalPlanner:
      def __init__(self, CP, init_v=0.0, init_a=0.0):
        self.mpc = LongitudinalMpc()
      def parse_model(model_msg, model_error):
          x = np.interp(T_IDXS_MPC, T_IDXS, model_msg.position.x) - model_error * T_IDXS_MPC
      def update(self, sm):
        x, v, a, j = self.parse_model(sm['modelV2'], self.v_model_error)
        self.mpc.update(sm['radarState'], v_cruise, x, v, a, j)
        self.v_desired_trajectory_full = np.interp(T_IDXS, T_IDXS_MPC, self.mpc.v_solution)
        self.a_desired_trajectory_full = np.interp(T_IDXS, T_IDXS_MPC, self.mpc.a_solution)
        self.v_desired_trajectory = self.v_desired_trajectory_full[:CONTROL_N]
        self.a_desired_trajectory = self.a_desired_trajectory_full[:CONTROL_N]
        self.j_desired_trajectory = np.interp(T_IDXS[:CONTROL_N], T_IDXS_MPC[:-1], self.mpc.j_solution)
        self.a_desired = float(interp(DT_MDL, T_IDXS[:CONTROL_N], self.a_desired_trajectory))
        self.v_desired_filter.x = self.v_desired_filter.x + DT_MDL * (self.a_desired + a_prev) / 2.0
    class LongitudinalMpc:
      def __init__(self, mode='acc'):
        self.solver = AcadosOcpSolverCython(MODEL_NAME, ACADOS_SOLVER_TYPE, N)
      def update(self, radarstate, v_cruise, x, v, a, j):
        self.yref[:,1] = x
        self.yref[:,2] = v
        self.yref[:,3] = a
        self.yref[:,5] = j
          self.solver.set(i, "yref", self.yref[i])
        self.solver.set(N, "yref", self.yref[N][:COST_E_DIM])
        self.run()
      def run(self):
        self.solution_status = self.solver.solve()
          self.x_sol[i] = self.solver.get(i, 'x')
          self.u_sol[i] = self.solver.get(i, 'u')
        self.v_solution = self.x_sol[:,1]
        self.a_solution = self.x_sol[:,2]
        self.j_solution = self.u_sol[:,0]
--- Pipe 1b: class LongitudinalPlanner > self.mpc = LongitudinalMpc() > def update(self, sm) >
    x,,, = self.parse_model(sm['modelV2'], ) > self.mpc.update(sm['radarState'],,x,,,) >
    self.v_desired_filter.x
--- Write "plantLatJLL.py":
--- Pipe 2: 'modelV2' > 'LateralPlanner': self.planner = LateralPlanner() > def plant_thread() >
    plant.step() > def step() > model.modelV2.position = position >
    sm = {'carState': car_state.carState, 'controlsState': control.controlsState, 'modelV2': model.modelV2} >
    self.planner.update(sm) > self.curvature = self.planner.desired_curvature,
    self.curvature_rate = self.planner.desired_curvature_rate
--- Read 230619: "test_lateral_mpcJLL.py", "lateral_plannerJLL.py", "lat_mpc.py"
    class LateralPlanner:
      def __init__(self, CP):
        self.path_xyz = np.zeros((TRAJECTORY_SIZE, 3))
        self.velocity_xyz = np.zeros((TRAJECTORY_SIZE, 3))
        self.plan_yaw = np.zeros((TRAJECTORY_SIZE,))
        self.y_pts = np.zeros(TRAJECTORY_SIZE)
        self.lat_mpc = LateralMpc()
        self.reset_mpc(np.zeros(4))
      def reset_mpc(self, x0=np.zeros(4)):
        self.x0 = x0
        self.lat_mpc.reset(x0=self.x0)
      def update(self, sm):
        measured_curvature = sm['controlsState'].curvature
        md = sm['modelV2']
          self.path_xyz = np.column_stack([md.position.x, md.position.y, md.position.z])
          self.plan_yaw = np.array(md.orientation.z)
          self.velocity_xyz = np.column_stack([md.velocity.x, md.velocity.y, md.velocity.z])
          car_speed = np.linalg.norm(self.velocity_xyz, axis=1)
          self.v_plan = np.clip(car_speed, MIN_SPEED, np.inf)
          self.v_ego = self.v_plan[0]
        self.lat_mpc.set_weights(PATH_COST, LATERAL_MOTION_COST,
        y_pts = self.path_xyz[:LAT_MPC_N+1, 1]
        heading_pts = self.plan_yaw[:LAT_MPC_N+1]
        self.lat_mpc.run(self.x0, p, y_pts, heading_pts, yaw_rate_pts)
        # with x0[3] = measured_yaw_rate, this would be the actual desired yaw rate.
        # x0 = np.array([x_init, y_init, psi_init, curvature_init])
        self.x0[3] = interp(DT_MDL, self.t_idxs[:LAT_MPC_N + 1], self.lat_mpc.x_sol[:, 3])
      def publish(self, sm, pm):
        plan_send = messaging.new_message('lateralPlan')
        lateralPlan = plan_send.lateralPlan
        lateralPlan.modelMonoTime = sm.logMonoTime['modelV2']
        lateralPlan.dPathPoints = self.y_pts.tolist()
        lateralPlan.psis = self.lat_mpc.x_sol[0:CONTROL_N, 2].tolist()
        lateralPlan.curvatures = (self.lat_mpc.x_sol[0:CONTROL_N, 3]/self.v_ego).tolist()
        lateralPlan.curvatureRates = [float(x/self.v_ego) for x in self.lat_mpc.u_sol[0:CONTROL_N - 1]] + [0.0]
        lateralPlan.useLaneLines = False
    class LateralMpc():
      def __init__(self, x0=np.zeros(X_DIM)):
        self.solver = AcadosOcpSolverCython(MODEL_NAME, ACADOS_SOLVER_TYPE, N)
        self.reset(x0)
      def reset(self, x0=np.zeros(X_DIM)):
        self.x_sol = np.zeros((N+1, X_DIM))
        self.u_sol = np.zeros((N, 1))
        self.yref = np.zeros((N+1, COST_DIM))
        for i in range(N):
          self.solver.cost_set(i, "yref", self.yref[i])
        self.solver.cost_set(N, "yref", self.yref[N][:COST_E_DIM])
      def set_weights(self, path_weight, heading_weight,
      def run(self, x0, p, y_pts, heading_pts, yaw_rate_pts):
        self.solution_status = self.solver.solve()
          self.x_sol[i] = self.solver.get(i, 'x')
          self.u_sol[i] = self.solver.get(i, 'u')
--- 230725 Pipe 3a: pm.send('lateralPlan', ) > def state_control() > lat_plan = self.sm['lateralPlan'] >
    self.desired_curvature,  = get_lag_adjusted_curvature(,,,lat_plan.curvatures,)
--- Run "plantLatJLL.py"
--- OK

230725: OK AM 1: NG CBQ 2: XJ 8: Lcz 1: Pipe 1-6: PC+C2+body: Change "controlsd.py", "carcontroller.py"
--- Do AM 1: AR 2 + Md 1 (Path)
--- ToDo 1: 'modelV2' > 'lateralPlan': Run model_replay.py, /controls/lib/tests/test_latcontrol.py
    selfdrive/test/process_replay/model_replay.py
      sm = messaging.SubMaster(['modelV2', ])
      pm = messaging.PubMaster(['roadCameraState', ..., 'lateralPlan'])
--- Q1a: a) test_latcontrol.py, b) latcontrol.py: Error on "CP" in a) as follows ???
    a) _, _, lac_log = controller.update(True, CS, CP, VM, params, last_actuators, True, 1, 0)
    b) def update(self, active, CS, VM, params, last_actuators, steer_limited, desired_curvature, desired_curvature_rate, llk):
--- Run "test_latcontrolJLL.py"
--- Error
    File "test_latcontrolJLL.py", line 43, in test_saturation
      _, _, lac_log = controller.update(True, CS, CP, VM, params, last_actuators,
    ERROR: test_saturation_0_HONDA_CIVIC_2016 (__main__.TestLatControl)  FAILED (errors=4)
--- Change to
    a) _, _, lac_log = controller.update(True, CS, VM, params, last_actuators, True, 1, 0, 0)
--- OK: A1a: Yes, Error on "CP"
--- Note: "llk" = 'liveLocationKalman' is not used by latcontrol_angle.py but by latcontrol_torque.py
--- Q1b: What does "llk_valid" in /body/carcontroller.py mean ??? A:
--- Pipe 1a: car.capnp > CC.orientationNED > log.capnp > llk.orientationNED
--- Pipe 1b: locationd.cc > pm({"liveLocationKalman"}) > paramsd.py > sm.updated['liveLocationKalman'] >
    ParamsLearner > pm.send('liveParameters',) > controlsd.py > sm['liveParameters'] >
    pm.send('controlsState',) > "measured_curvature" > lateral_planner.py >
    LateralPlanner > 230619 Pipe 1f > lat_mpc.run() > x0[3] "measured_yaw_rate" >
    pm.send('lateralPlan', ) > plannerd.py > controlsd.py > sm['lateralPlan']
--- Pipe 1c: locationd.cc > paramsd.py > controlsd.py > lateral_planner.py > plannerd.py > controlsd.py
--- Pipe 1d: "liveLocationKalman" > ParamsLearner > 'liveParameters' > 'controlsState' >
    'measured_curvature' > LateralPlanner > LateralMpc > x0[3] "measured_yaw_rate"
--- body does not need "state_transition()", Delete it
--- NG: body needs it!
--- XJ 8a: Change "latcontrol.py", "controlsd.py" from /body to /toyota for "self.LaC"
    1) Save to LPXJ/../latcontrolXJ7.py. Delete a2) in 230719 Pipe 1 c) to
--- Pipe 2: /toyota, /body: latcontrol.py > def update() > latcontrol_angle.py > def update()
    2) Change "controlsd.py"
      self.LaC: LatControl  # XJ 8a 2)
      self.LaC = LatControlAngle(self.CP, self.CI)
      actuators.steeringAngleDeg = self.LaC.update(CC.latActive, CS, self.VM, lp,
    3) Save to LPXJ/../latcontrol_angleLP092.py. Change "latcontrol_angle.py" to
      return float(angle_steers_des)
--- OK: but "body" keeps moving forward
    4) Change "controlsd.py" to
      if CC.longActive and CC.latActive:  # XJ 8a 4)
--- OK: but "body" not moving to 3 m
    5) Change "controlsd.py" to B
      A: actuators.steer = actuators.steeringAngleDeg / 45.
      B: actuators.steer =  0.
--- NG: same "body" not moving to 3 m
    6) undo XJ 8a 4) to
      if CC.longActive:
      if CC.latActive:
--- OK: but Q1: Why ??? A: CC.latActive is False at first
--- Q2: "body" turned more left for 5A than 5B. Why not straight ??? > CBQ 2
--- Read LaC.update() for CBQ 2
--- Pipe 3a: 230619 Pipe 1f > pm.send('lateralPlan', ) > def state_control() >
    lat_plan = self.sm['lateralPlan'] >
    self.desired_curvature,  = get_lag_adjusted_curvature(,,,lat_plan.curvatures,) >
    actuators.steeringAngleDeg = self.LaC.update(,,,,,,self.desired_curvature,,)
      class LatControlAngle(LatControl):
        def update(,,,,,,desired_curvature,,,):
          angle_steers_des = math.degrees(VM.get_steer_from_curvature(-desired_curvature,,))
            def get_steer_from_curvature(self, curv: float, u: float, roll: float) -> float:
              return (curv - self.roll_compensation(roll, u)) * self.sR * 1.0 / self.curvature_factor(u)
--- Pipe 3b: "measured_curvature" > LateralPlanner > LateralMpc > x0[3] "measured_yaw_rate" >
    lat_plan.curvatures > get_lag_adjusted_curvature() > self.desired_curvature >
    VM.get_steer_from_curvature() > actuators.steeringAngleDeg
      /log.capnp > struct ControlsState {
      curvature @37 :Float32;  # path curvature from vehicle model
      desiredCurvature @61 :Float32;  # lag adjusted curvatures used by lateral controllers
--- Pipe 3c: ParamsLearner > liveParameters.angleOffsetDeg > curvature > measured_curvature
--- ToDo 2: Write [Liu23b]: get_lag_adjusted_curvature(), get_steer_from_curvature()
--- 230619 Pipe 2b: /paramsd.py > def main() > params = { 'angleOffsetAverageDeg': 0.0, >
    while True:
      |learner = ParamsLearner(CP, CP.steerRatio, 1.0, 0.0)
        class ParamsLearner:
          self.kf = CarKalman(GENERATED_DIR, steer_ratio, stiffness_factor, angle_offset, P_initial)
            class States():  # dim = 9
              ANGLE_OFFSET = _slice(1)  # [rad]
              ...
            /selfdrive/locationd/models/car_kf.py
            class CarKalman(KalmanFilter):
              initial_x = np.array([ 1.0, 15.0, 0.0, 0.0,  10.0, 0.0, 0.0, 0.0, 0.0 ])  # dim = 9
              def __init__(self, generated_dir, steer_ratio=15, stiffness_factor=1, angle_offset=0, P_initial=None):  # pylint: disable=super-init-not-called
                dim_state = self.initial_x.shape[0]
                self.filter = EKF_sym_pyx(generated_dir, self.name, self.Q, self.initial_x, self.P_initial, dim_state, dim_state_err, global_vars=self.global_vars, logger=cloudlog)
                  /rednose_repo/rednose/helpers/ekf_sym_pyx.pyx
                  cdef class EKF_sym_pyx:
                    cdef EKFSym* ekf
                      cdef cppclass EKFSym:
                        VectorXd state()
                          /rednose_repo/rednose/helpers/ekf_sym_pyx.cpp
                          *   def state(self):             # <<<<<<<<<<<<<<
                    def state(self):
                      cdef np.ndarray res = vector_to_numpy(self.ekf.state())
                      return res
            /rednose_repo/rednose/helpers/kalmanfilter.py
            class KalmanFilter:
              initial_x = np.zeros((0, 0))  # = [] see W3.py ===0811
              @property
              def x(self):
                return self.filter.state()
      |x = learner.kf.x  # dim = 9
      angle_offset_average = params['angleOffsetAverageDeg']
      angle_offset_average = clip(math.degrees(x[States.ANGLE_OFFSET]), angle_offset_average - MAX_ANGLE_OFFSET_DELTA, angle_offset_average + MAX_ANGLE_OFFSET_DELTA)
      angle_offset = clip(math.degrees(x[States.ANGLE_OFFSET] + x[States.ANGLE_OFFSET_FAST]), angle_offset - MAX_ANGLE_OFFSET_DELTA, angle_offset + MAX_ANGLE_OFFSET_DELTA)
      liveParameters.angleOffsetAverageDeg = angle_offset_average
      liveParameters.angleOffsetDeg = angle_offset
    /controlsd.py > def publish_logs() >
      lp = self.sm['liveParameters']
      steer_angle_without_offset = math.radians(CS.steeringAngleDeg - lp.angleOffsetDeg)
      curvature = -self.VM.calc_curvature(steer_angle_without_offset, CS.vEgo, lp.roll)
      controlsState.curvature = curvature
    /run_paramsd_on_routeJLL.py >
      'angleOffsetAverageDeg': 0.3493630290031433
--- Pipe 3d: paramsd.py > learner.kf.x > liveParameters.angleOffsetDeg > controlsd.py > controlsState.curvature
--- Do 230619 ToDo 1: Run 'test_lateral_mpcJLL.py'
--- ToDo 3: for CBQ 2: Change class LatControlAngle update()
--- OK: "body" can standstill in hall way but keep going most of the time
--- 230213 Pipe 3 >
--- XJ 8b: Change "/body/carcontroller.py"
    1) speed_desired = CC.actuators.accel
--- NG: "body" rolls very fast. Why ???
    2) speed_diff_desired = 0.
--- NG: turns left
--- Q3: How to define "body" CS.vEgo ???
    self.odometer += abs(CS.vEgo * DT_CTRL)  # XJ 6c
    /toyota/carstate.py
    class CarState(CarStateBase):
      def update(self, cp):
        ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
        ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
        ret.vEgoCluster = ret.vEgo * 1.015  # minimum of all the cars
    /body/carstate.py
        ret.vEgoRaw = ((ret.wheelSpeeds.fl + ret.wheelSpeeds.fr) / 2.) * self.CP.wheelSpeedFactor
        ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
--- Pipe 4a: self.CI, = get_car() > def data_sample() > CS = self.CI.update(self.CC, can_strs) >
    self.odometer += abs(CS.vEgo * DT_CTRL)
--- Pipe 4b: def get_car() > return CarInterface(CP, CarController, CarState), CP
--- Pipe 4c: CarInterfaceBase(ABC) > def update(self, c: car.CarControl,
    can_strings: List[bytes]) -> car.CarState > ret = self._update(c) >
    ret.vEgoCluster = apply_hysteresis(ret.vEgoCluster, ) > CarInterface(CarInterfaceBase)
--- Pipe 4d: CarInterfaceBase(ABC) > def _update(): pass > CarInterface() > def _update(, c) >
    ret = self.CS.update(self.cp) > ret.events[0].enable = True > return ret
--- Pipe 4e: CarState > def update(, cp) > ret.vEgo, ret.aEgo = update_speed_kf(ret.vEgoRaw)
--- Note: ret.vEgoCluster not updated in /body
--- ToDo 4: Run selfdrive/controls/tests/test_cruise_speed.py, /test_following_distance.py
    selfdrive/test/longitudinal_maneuvers/test_longitudinal.py
--- Read "body" CS.vEgoRaw
    ret.vEgoRaw = ((ret.wheelSpeeds.fl + ret.wheelSpeeds.fr) / 2.) * self.CP.wheelSpeedFactor
    speed_measured = SPEED_FROM_RPM * (CS.out.wheelSpeeds.fl + CS.out.wheelSpeeds.fr) / 2.
    SPEED_FROM_RPM = wheelSpeedFactor = 0.008587
    ret.wheelSpeeds.fl = cp.vl['MOTORS_DATA']['SPEED_L']
    ret.wheelSpeeds.fr = cp.vl['MOTORS_DATA']['SPEED_R']
    SG_ SPEED_L : 7|16@0- (1,0) [-1000|1000] "" XXX
    SG_ SPEED_R : 23|16@0- (1,0) [-1000|1000] "" XXX
--- Read CAN DBC File Explained - A Simple Intro
--- ToDo 5: Run /openpilot/aJLL/ModelOld/dbcJLL.py
    dbc_test = dbc(os.path.join(DBC_PATH, 'toyota_prius_2017_pt_generated.dbc'))
    CAR.PRIUS: dbc_dict('toyota_nodsu_pt_generated', 'toyota_adas'),
    CAR.BODY: dbc_dict('comma_body', None),
--- XJ 8b 3): for CBQ 2:
    self.odometer += abs(CS.vEgoRaw * DT_CTRL)  # XJ 8b 3)
--- NG: not straight, not standstill (going forward)
--- 4) for standstill:
      actuators.accel = -0.05
      speed_diff_desired = -0.01
      #speed_diff_desired = -0.0049
--- NG
--- Undo XJ 8b. Go back to XJ 8a 5A) 6). Save to controlsdXJ8.py, carcontrollerXJ8.py
--- OK but still like Ctr 3: (0,0) to (3.24, -0.57)m => ToDo 6 solution ???
--- ToDo 6: PIDController + CC.orientationNED + lat_mpc.py + test_lateral_mpcJLL.py (test_straight())
--- Read "-CC.orientationNED[1]":
    /body/carcontroller.py
      angle_error = np.clip((-CC.orientationNED[1]) - angle_setpoint, -MAX_ANGLE_ERROR, MAX_ANGLE_ERROR)
    /controlsd.py
      def publish_logs(self, CS, start_time, CC, lac_log):  # Send actuators and hud commands to the car, send controlsstate and MPC logging
        orientation_value = list(self.sm['liveLocationKalman'].calibratedOrientationNED.value)
        CC.orientationNED = orientation_value
        angular_rate_value = list(self.sm['liveLocationKalman'].angularVelocityCalibrated.value)
        CC.angularVelocity = angular_rate_value
    /cereal/log.capnp
      liveLocationKalman @72 :LiveLocationKalman;
        struct LiveLocationKalman {
          # https://github.com/commaai/openpilot/tree/master/common/transformations
          positionECEF @0 : Measurement;
          positionGeodetic @1 : Measurement;
          # These angles are all eulers and roll, pitch, yaw
          # orientationECEF transforms to rot matrix: ecef_from_device
          orientationECEF @6 : Measurement;
          calibratedOrientationECEF @20 : Measurement;
          orientationNED @7 : Measurement;  # orientationNED[0]: roll, [1]: pitch, [2]: yaw
          angularVelocityDevice @8 : Measurement;
          # orientationNEDCalibrated transforms to rot matrix: NED_from_calibrated
          calibratedOrientationNED @9 : Measurement;
          # Calibrated frame is simply device frame aligned with the vehicle
          velocityCalibrated @10 : Measurement;
          accelerationCalibrated @11 : Measurement;
          angularVelocityCalibrated @12 : Measurement;
          ...
          struct Measurement { value @0 : List(Float64); std @1 : List(Float64); valid @2 : Bool; }  }
    /common/transformations/coordinates.hpp
      struct ECEF { double x, y, z; Eigen::Vector3d to_vector(){ return Eigen::Vector3d(x, y, z); } };
      struct NED { double n, e, d; Eigen::Vector3d to_vector(){ return Eigen::Vector3d(n, e, d); } };
      struct Geodetic { double lat, lon, alt; bool radians=false; };
      class LocalCoord { NED ecef2ned(ECEF e); ECEF ned2ecef(NED n);
    /selfdrive/locationd/models/live_kf.h
      class LiveKalman { Eigen::VectorXd get_x();
    /selfdrive/locationd/locationd.h
      class Localizer { std::unique_ptr<LiveKalman> kf;
    /selfdrive/locationd/locationd.cc
      static void init_measurement(cereal::LiveLocationKalman::Measurement::Builder meas, const VectorXd& val, const VectorXd& std, bool valid) {
        meas.setValue(kj::arrayPtr(val.data(), val.size()));
        meas.setStd(kj::arrayPtr(std.data(), std.size()));
        meas.setValid(valid); }
      void Localizer::build_live_location(cereal::LiveLocationKalman::Builder& fix) {
        VectorXd predicted_state = this->kf->get_x();
        VectorXd fix_ecef = predicted_state.segment<STATE_ECEF_POS_LEN>(STATE_ECEF_POS_START);
        ECEF fix_ecef_ecef = { .x = fix_ecef(0), .y = fix_ecef(1), .z = fix_ecef(2) };
        VectorXd orientation_ecef = quat2euler(vector2quat(predicted_state.segment<STATE_ECEF_ORIENTATION_LEN>(STATE_ECEF_ORIENTATION_START)));
        MatrixXdr device_from_ecef = euler2rot(orientation_ecef).transpose();
        VectorXd calibrated_orientation_ecef = rot2euler((this->calib_from_device * device_from_ecef).transpose());
        VectorXd device_from_ecef_eul = quat2euler(vector2quat(predicted_state.segment<STATE_ECEF_ORIENTATION_LEN>(STATE_ECEF_ORIENTATION_START))).transpose();
        VectorXd orientation_ned = ned_euler_from_ecef(fix_ecef_ecef, orientation_ecef);
        VectorXd calibrated_orientation_ned = ned_euler_from_ecef(fix_ecef_ecef, calibrated_orientation_ecef);
        init_measurement(fix.initPositionECEF(), fix_ecef, fix_ecef_std, this->gps_mode);
        init_measurement(fix.initOrientationNED(), orientation_ned, orientation_ned_std, this->gps_mode);
        init_measurement(fix.initCalibratedOrientationNED(), calibrated_orientation_ned, nans, this->calibrated && this->gps_mode);
        init_measurement(fix.initAngularVelocityCalibrated(), ang_vel_calib, ang_vel_calib_std, this->calibrated);
      VectorXd Localizer::get_position_geodetic() {
        VectorXd fix_ecef = this->kf->get_x().segment<STATE_ECEF_POS_LEN>(STATE_ECEF_POS_START);
        ECEF fix_ecef_ecef = { .x = fix_ecef(0), .y = fix_ecef(1), .z = fix_ecef(2) };
        Geodetic fix_pos_geo = ecef2geodetic(fix_ecef_ecef);
        return Vector3d(fix_pos_geo.lat, fix_pos_geo.lon, fix_pos_geo.alt);
      kj::ArrayPtr<capnp::byte> Localizer::get_message_bytes(MessageBuilder& msg_builder, bool inputsOK,
                                                             bool sensorsOK, bool gpsOK, bool msgValid) {
        cereal::LiveLocationKalman::Builder liveLoc = evt.initLiveLocationKalman();
        this->build_live_location(liveLoc);
        liveLoc.setSensorsOK(sensorsOK);
        liveLoc.setGpsOK(gpsOK);
        liveLoc.setInputsOK(inputsOK);
        return msg_builder.toBytes();
      int Localizer::locationd_thread() {
        const std::initializer_list<const char *> service_list = {gps_location_socket, "cameraOdometry", "liveCalibration",
                                                                "carState", "carParams", /*"accelerometer", "gyroscope"*/ "sensorEvents"};
        SubMaster sm(service_list, {}, nullptr, {gps_location_socket, "carParams"});
        PubMaster pm({"liveLocationKalman"});
        while (!do_exit) {
          sm.update();
            for (const char* service : service_list) {
              if (sm.updated(service) && sm.valid(service)){
                const cereal::Event::Reader log = sm[service];
                this->handle_msg(log);
          const char* trigger_msg = sm["carParams"].getCarParams().getNotCar() ? "sensorEvents" : "cameraOdometry";
          if (sm.updated(trigger_msg)) {
            MessageBuilder msg_builder;
            kj::ArrayPtr<capnp::byte> bytes = this->get_message_bytes(msg_builder, inputsOK, sensorsOK, gpsOK, filterInitialized);
            pm.send("liveLocationKalman", bytes.begin(), bytes.size());
            if (cnt % 1200 == 0 && gpsOK) {  // once a minute
              VectorXd posGeo = this->get_position_geodetic();
              std::string lastGPSPosJSON = util::string_format(
                "{\"latitude\": %.15f, \"longitude\": %.15f, \"altitude\": %.15f}", posGeo(0), posGeo(1), posGeo(2));
                Params().put("LastGPSPosition", gpsjson);
--- Pipe 5a: log = sm[service] > handle_msg(log) > get_message_bytes() > pm.send("liveLocationKalman",,) >
    self.sm['liveLocationKalman'].calibratedOrientationNED.. > CC.orientationNED = orientation_value
--- Pipe 5b: initLiveLocationKalman() > build_live_location(liveLoc) >
    calibrated_orientation_ned = ned_euler_from_ecef() > get_message_bytes()
--- Pipe 5c: gpsOK > get_position_geodetic() > kf->get_x() > fix_ecef > fix_ecef_ecef >
    ecef2geodetic() > posGeo = Vector3d(fix_pos_geo.lat, fix_pos_geo.lon, fix_pos_geo.alt) >
    Params().put("LastGPSPosition", )
--- Pipe 5d: log.capnp > coordinates.hpp > live_kf.h > locationd.cc > controlsd.py > carcontroller.py > controlsd.py
--- ToDo 7: Run /locationd/test/test_locationd.py, /_test_locationd_lib.py
    /rednose_repo/examples/Compute_station_pos.ipynb, live_kf.py, /tools/gpstest/test_gps.py
--- 230619 Pipe 1f: LateralPlanner > self.lat_mpc.reset(x0=self.x0) >
    def update() > measured_curvature = sm['controlsState'].curvature > md = sm['modelV2'] >
    self.path_xyz = ... md.position.x > self.plan_yaw = np.array(md.orientation.z) > ...
--- Read "position.x"
    /selfdrive/modeld/modeld.cc
    int main(int argc, char **argv) {
      if (!do_exit) {
        run_model(model, vipc_client_main, vipc_client_extra, main_wide_camera, use_extra_client);
    void run_model(ModelState &model, VisionIpcClient &vipc_client_main, VisionIpcClient &vipc_client_extra, bool main_wide_camera, bool use_extra_client) {
      while (!do_exit) {
        if (model_output != nullptr) {
          model_publish(pm, meta_main.frame_id, meta_extra.frame_id, frame_id, frame_drop_ratio, *model_output, meta_main.timestamp_eof, model_execution_time,
    /selfdrive/modeld/models/driving.cc
    void model_publish(PubMaster &pm,,,,,const ModelOutput &net_outputs,,,kj::ArrayPtr<const float> raw_pred,) {
        framed.setRawPredictions(raw_pred.asBytes());
      fill_model(framed, net_outputs);
      pm.send("modelV2", msg);
    void fill_model(cereal::ModelDataV2::Builder &framed, const ModelOutput &net_outputs) {
      const auto &best_plan = net_outputs.plans.get_best_prediction();
      fill_plan(framed, best_plan);
    void fill_plan(cereal::ModelDataV2::Builder &framed, const ModelOutputPlanPrediction &plan) {
      for(int i=0; i<TRAJECTORY_SIZE; i++) {
        pos_x[i] = plan.mean[i].position.x;
      fill_xyzt(framed.initPosition(), T_IDXS_FLOAT, pos_x, pos_y, pos_z, pos_x_std, pos_y_std, pos_z_std);
    void fill_xyzt(cereal::XYZTData::Builder xyzt,,const std::array<float, size> &x,,) {
      xyzt.setT(to_kj_array_ptr(t));
      xyzt.setX(to_kj_array_ptr(x));
    void fill_xyzt(cereal::XYZTData::Builder xyzt,,const std::array<float, size> &x,,,,,,) {
      fill_xyzt(xyzt, t, x, y, z);
      xyzt.setXStd(to_kj_array_ptr(x_std));
    /selfdrive/modeld/models/driving.h
      struct ModelOutputXYZ { float x; float y; float z; };
      struct ModelOutputPlanElement { ModelOutputXYZ position;
      struct ModelOutputPlans {
        std::array<ModelOutputPlanPrediction, PLAN_MHP_N> prediction;
        constexpr const ModelOutputPlanPrediction &get_best_prediction() const {
          return prediction[max_idx];
      struct ModelOutput { const ModelOutputPlans plans;
    /cereal/gen/cpp/log.capnp.h
      inline void ModelDataV2::Builder::setRawPredictions( ::capnp::Data::Reader value) {
      inline void XYZTData::Builder::setX(::kj::ArrayPtr<const float> value) {
    /cereal/log.capnp
      struct XYZTData @0xc3cbae1fd505ae80 { x @0 :List(Float32); y @1 :List(Float32); z @2 :List(Float32);
      struct ModelDataV2 { rawPredictions @16 :Data; position @4 :XYZTData; orientation @5 :XYZTData;
--- Pipe 6a: modeld.cc > main() > run_model() > model_publish() > fill_model() > pm.send("modelV2",)
--- Pipe 6b: fill_model() > best_plan > fill_plan(framed, best_plan) > pos_x[i] = plan.mean[i].position.x >
    fill_xyzt(,,pos_x,,,,,,) > xyzt.setX(to_kj_array_ptr(x))
--- Lcz 1: Change "self.path_xyz = ... md.position.x" to "straight w/o md" for CBQ 2
    as in "test_lateral_mpcJLL.py/test_straight"

230719: OK AR 2: Md 1: XJ 7: Pipe 1: PC+C2+body: Change "controlsd.py", Rewrite "longcontrol.py", "latcontrol.py"
--- Md 1: 'modelV2' + Path: 'longitudinalPlan' + 'lateralPlan'
--- Do AR 2: AM 1 w/o Md 1 (Path)
--- 230213 Pipe 1-2, 230521 Pipe 3: a)-d), 230525 Delete 230521 Pipe 3b, 3c
--- Change 230521 Pipe 3 a) b) c) d) to
--- Pipe 1: def state_control(CS) > CC = car.CarControl > actuators = CC.actuators >
    a1) actuators.accel = 1.5*clip(), a2) steer = clip(), actuators.steer = steer
    b) self.LoC > actuators.accel = self.LoC.update() >
         longcontrol.py > def update() > a1) > self.pid.update() > pid.py > def update()
    c) self.LaC > actuators.steer = self.LaC.update() >
         latcontrol.py > def update() > a2) > latcontrol_angle.py > def update()
    d) unchanged
--- Note 1: "actuators.steer" is for body not cars
    actuators.steer, actuators.steeringAngleDeg = steer, steer * 45.
    actuators.steer, actuators.steeringAngleDeg, lac_log = self.LaC.update(CC.latActive, CS, self.VM, lp,
      latcontrol_angle.py > def update() > return 0, float(angle_steers_des), angle_log >
      actuators.steer = 0 for cars
--- Note 2: "lac_log.output", "LateralAngleState", "LateralDebugState"
    if not self.joystick_mode:
      lac_log = self.LaC.update() > latcontrol_angle.py > def update() >
        angle_log = log.ControlsState.LateralAngleState > struct LateralAngleState >
        angle_log.output (not set) > lac_log.output (not set)
    else:
      lac_log = log.ControlsState.LateralDebugState > struct LateralDebugState >
        lac_log.output = actuators.steer (set)
--- Note 3: "steerControlType" > "LatControlAngle"
    /toyota/interface.py > steerControlType = car.CarParams.SteerControlType.angle
    /body/interface.py >   steerControlType = car.CarParams.SteerControlType.angle >
      self.CP.steerControlType > steerControlType @34 :SteerControlType; >
      enum SteerControlType {torque @0; angle @1; curvature @2;}
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      self.LaC = LatControlAngle(self.CP, self.CI)
--- Note 4: Key: ".steer =", ".accel ="
    body >   new_actuators.steer = torque_r
             new_actuators.accel = torque_l
    toyota > new_actuators.steer = apply_steer / CarControllerParams.STEER_MAX
             new_actuators.accel = self.accel
--- XJ 7a: Do Pipe 1 b) > a1): self.LoC: Change "controlsd.py", Rewrite "longcontrol.py"
    1) Save C2/../longcontrol.py to LPXJ/../longcontrolLP092.py
--- Q: What are dashcamOnly, self.read_only, passive, controller_available, "OpenpilotEnabledToggle" for ???
    A: dashcamOnly > self.read_only > dashcam mode; not openpilot_enabled_toggle > passive >
         not controller_available; "openpilot_enabled_toggle > active" > OP can be active or passive >
         if OP passive (not engaged) > for "read_only"
       /tesla, toyota/interface.py > dashcamOnly = True;
       but not /body/interface.py > for /body, OP always active (we cannot toglle OP) >
    2) Change "controlsd.py": Delete
      "self.read_only" (not needed) > Delete "passive, openpilot_enabled_toggle, dashcamOnly"
    3) Change "controlsd.py": add back:
      from selfdrive.controls.lib.longcontrol import LongControl
      self.LoC = LongControl()
--- 4) Change "controlsd.py": Delete
      if self.sm.rcv_frame['testJoystick'] > 0:  # XJ 6c: for running web.py (remote control)
      elif 3.0 <= self.odometer and self.odometer < 3.1:  # XJ 6c: standstill
      elif 3.1 <= self.odometer and self.odometer < 6.5:  # XJ 6c: backward
--- 5) Change "controlsd.py", Rewrite "longcontrol.py" to
      #actuators.accel = 1.5*clip(0.5, -1, 1)
      actuators.accel = self.LoC.update()
        /controls/lib/longcontrol.py
        LongCtrlState = car.CarControl.Actuators.LongControlState  # needed by plannerd.py
        class LongControl:
          def __init__(self):
            self.last_output_accel = 0.0
          def update():
--- XJ 7b: Do Pipe 1 c) > a2): self.LaC: Change "controlsd.py", Rewrite "latcontrol.py"
--- 1) Save C2/../latcontrol.py to LPXJ/../latcontrolLP092.py; Rewrite "latcontrol.py" to
      /controls/lib/latcontrol.py
      class LatControl:
        def __init__(self):
          self.last_output_accel = 0.0
        def update():
--- 2) Change "controlsd.py" to
      from selfdrive.controls.lib.latcontrol import LatControl
      self.LaC = LatControl()
--- OK: Run "body"
