CB 7  Jinn-Liang Liu  2023.8.29 - 2024.2.23  All Rights Reserved. © Copyright 2023 - 2024

========== OP Coding Notes: Run, Change, Read ==========

========== Summary of Coding Notes ==========

240207: NG MC3 MC6 MC7: PC: Run Change "train_modelB6.py"
--- Do BATCH_SIZE > 1 (for GPUs)
--- Do 1: decay_steps = STEPS*EPOCHS and PrintLearningRate(Callback)
--- MC6: loss = KB.max(KB.abs(y_pred - y_true), axis=-1)
--- NG MC6: maxae: 21.76048 <! MC5d: best maxae: 1.7771
--- S 4: a) OK: decay_steps = STEPS*EPOCHS, PrintLearningRate(Callback)
         b) "cold start": use new random weights, "warm restart": by load_weights()
         c) NG MC6: loss = KB.max(KB.abs(y_pred - y_true),) > maxae: 21.76048 <! MC5d: 1.7771
--- Do 2: CosineDecayWarmup:
--- MC7: (MC5: cosine_similarity + mse) + CosineDecayWarmup
--- NG MC7: maxae: 4.2998 <! MC5d: best maxae: 1.7771
--- ToDo: Run MC7 on Server
--- ToDo: PC GPU
--- ToDo: Cycle Learning Rate
--- ToDo: Keras Learning Rate Finder
240124: NG MC3 MC4 MC5: NG RT 4: Pipe A B: S 1 S 2 S 3: PC: Run Change "train_modelB6.py"
--- Q1: PC data too small > RT 3 ??? Yes, see S 2 below.
--- Q2: Error in "datagenB6.py" (yuv.h5, Ximgs etc.) > RT 3 ??? No, see B6SimMC5f1.png
--- Q3: Need calibration > RT 3 ??? No, see B6SimMC5b*.png
--- Do 1: Put "hevc2yuvh5.py" in "datagenB6.py" w/o "yuv.h5"
--- Change datagen_test() in "datagenB6.py"
--- NG Do 1: cv2.VideoCapture got wrong total_frames in def datagen_test() for Testing
--- Quit Do 1: I can't get specific frames by cv2 in def datagen() for Training
--- Change "datagenB6.py" = "B6b" (datagen()) + "B6c" (datagen_test() good for vedio show)
--- MC3 = MC2: LogCosh + AdamW
--- Pipe A: 1. train_modelB6.py (custom_loss*) > B6BW.hdf5, B6.keras > B6Loss*.png >
    2. simulatorB6.py > B6Sim*.png > 3. train_modelB6.py > B6Y*.png > GoTo 1 if needed >
--- Pipe B: 4. B6*.keras > h5topb.py > B6*.pb > 5. snpe > B6*.dlc >
    6. FTP > sftp://root@10.0.0.18:8022/data/openpilot/models/B6*.dlc > 7. Road Test
--- MC4: T9 > cosine_similarity + LogCosh
--- MC5: cosine_similarity + mse
--- MC5b: B*** > RNN state important > Xin3_temp
--- OK MC5b: C***: Yes, RNN state is important.
--- Q6: MC1-MC5b use --33/yuv.h5 only ??? Yes.
--- Q7: --33/yuv.h5 for all MC1-MC5b => RT 3 ??? No, it is due to S 1 b).
--- OP32:  supercombo079.keras > --32/ (frame_no = 2) > B6SimOP32.png
--- S 1: Summary 1:
    a) SimMC5c1 (20, 60) is better than (>!) 5b1 (1148)
    b) Trapped in Local Minimum: EarlyStopping: B6LossMC5c1 >! c2 >! c3
    c) b) > SimMC5c1 (shorter training) >! c3 (longer)
    d) RNN state important > Xin3_temp, batch: sequential > OK: parallel GPUs
--- Q8: How to Overcome Local Minimum?
--- OK MC5d2: Train and Valid only --32 (ignor val_loss) > Best maxae: 1.7771
--- Q9: SimMC5f1 (00:00:39) better than MC5f (03:09:29) ??? Yes for Sim but No for RT 4
--- NG RT 4: 240204 Road Test Error on B6CMf51.dlc: Same as RT 2
--- OK: SnG on C2A
--- Q10: Prius SnG needs smartDSU ???
--- S 2: Summary 2: Need to
    a) beat MC5d: Train and Valid only --32 (or --33) (ignor val_loss) + (20, 60) > Best maxae: 1.7771
    b) have "both" good B6Sim*.png and B6Y*.png
    c) solve Q8: How to Overcome Local Minimum?
       > SimMC5c1 (20, 60, better escaping L Min) >! 5b1 (1148)
       > SimMC5e (1148, 5, longer) <! (worse) MC5d3 (1148, 2, shorter)
       > use MC5c1 (20, 60) > patience=25 or less
    d) solve Q9: SimMC5f1 (00:00:39) >! MC5f (03:09:29) ??? Yes for Sim but No for RT 4
    e) solve Q11: Why all B6Sim*.png show "Left" curves ??? Due to the data --32 and --33 ???
    f) do Q12: Which of MC3: LogCosh and MC5: cosine_similarity + mse is better ???
    g) use Server for much longer training on bigger data
--- S 2e: For Q11: Add --37 to dataB6
--- Change yuvX.shape from 1150 (yuv1150.h5) to 1200:
--- S 2a: train and validate same --37/yuv.h5
--- S 3: a) SimCM: 3a1 >! 5g1 but maxae: 5g1 >! 3a1; both NG Q11: Why all B6Sim*.png show "Left" curves
--- S 3: b) SimCM: 3a3 >! 3a1 >! 3a: longer training better
         c) KSRT: keep serverB6, restart train_modelB6 to escape L Min
         d) Use MC3 (better Sim) or MC5 (better maxae)
240123: NG. Md 2: PC: "modelB7" w/o .py
--- ToDo Md 2: "supercombo079.summary" > write "modelB7.py"
--- Do 1: "supercombo079.keras" > "modelB7" w/o .py
--- NG Do 1:
240121: OK RT 2: RT 3: UI 2d: C2A+car: Road Test: "B6MC2.dlc"
--- RT 3: 240121 Road Test Error: "Path Prediction Error":
--- UI 2d: Add "啟用行車記錄功能" to OP079C2 for seeing "Path" by "B6MC2.dlc"
240113: PC: Run OP095/"get_model_metadata.py"
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/OP095$ python get_model_metadata.py
240109: Poster Presentation: 2309 AI Projects.docx
240103: Server A: "WiFi" or "Bluetooth" Wireless Connection
231229: Pipe 1: PC: Run Change "train_modelB6.py", "custom_loss"
--- Pipe 1: train_modelB6.py > B6MC2.keras > h5topb.py > B6MC2.pb > B6MC2.dlc > Road Test
231217: OK UI: PC: Run Read "PlotJuggler"
    (sconsvenv) jinn@Liu:~/openpilot$ ./tools/plotjuggler/juggle.py "/home/jinn/dataC/8bfda98c9c9e4291|2020-05-11--03-00-57--61"
231211: OK Md 4b: NG Md 4c: Md 4d: PC: "onnx2py.py" > "supercombo092.py"
--- OK Md 4b: See /home/jinn/sconsvenv/lib/python3.8/site-packages/supercombo092.py
--- move supercombo092.py (not useful) .onnx back to /home/jinn/openpilot/selfdrive/modeld/models/
--- Md 4c: onnx2torch => pytorch2keras
231121: PID 2: Pipe 1-3: PC: Read "LatControlTorque" + Run Read "test_latcontrolJLL.py"
--- PID 2: [Liu23a] + "PIDController" + "LatControlPID" + "LatControlTorque"
--- Pipe 1: controlsd.py > Controls() > self.CP = get_car() > CP = CarInterface.get_params(candidate,,,,) >
    cls._get_params(,candidate,,,,) > CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning) >
    tune.init('torque')
--- Pipe 2: values.py > class CAR > PRIUS = "TOYOTA PRIUS 2017" > CAR.PRIUS > interface.py > candidate == CAR.PRIUS
--- Pipe 3: fw_versions.py > match_fw_to_car() > car_helpers.py > get_car() > candidate = fingerprint()
    controlsd.py > self.CP = get_car()
--- "lateralTuning" > (a) lateralTuning.pid, init('pid') (b) init('torque'). No cars use'indi' anymore (OP092)
--- TOYOTA.PRIUS > "LatControlTorque" > "PIDController"
231113: OK Md 4a: NG UI 2a: PC: "h5toonnx.py" > "modelB6.onnx"
--- Do Md 4a: downgrade onnx
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ python h5toonnx.py
--- Run modelB6.onnx
    (sconsvenv) jinn@Liu:~/openpilot$ ./selfdrive/ui/ui
    (sconsvenv) jinn@Liu:~/openpilot/tools/replay$ ./replayJLL --data_dir dataC "8bfda98c9c9e4291|2020-05-11--03-00-57--61"
--- NG: replayJLL only replays the video (not by modelB6.onnx (supercombo.onnx))
--- UI 2a: How to display PC/C2 UI run by modelB6.onnx ???
231107: NG RT 2: C2A+car: Road Test: "supercombo079.dlc", "B5YJ0421.dlc"
231030: NG Md 4a: PC: "h5toonnx.py" > "modelB6.onnx"
--- Md 4a: "h5toonnx.py" > "modelB6.onnx"
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model/saved_model$ python h5toonnx1.py
230922: OK "VMware" PC: openpilot$ "scons -i" then "scons -u -j$(nproc)"
--- scons openpilot on "VMware" in 2 Steps: 1. scons -i, 2. scons -u -j$(nproc)
230919: How To Connect a Projector to Ubuntu Laptop?
230913: OK. Md 3: PC: Change Run "tiny1.py" "tiny2.py"
--- Md 3: aJLL/tinygrad/test/test_nn.py
230912: OK CBQ 9: AR 2: XJ 9: PC+C2+body: Change "controlsd.py" "longcontrol.py" "carcontroller.py"
--- XJ 9a: Change "controlsd.py"
--- OK: -2.5*clip
--- CBQ 9: CB keeps going forward when self.odometer > 6.5 Why ???
--- XJ 9b: Change "controlsd.py"
--- OK: 4.5*clip(-0.5 backward normal speed, but
--- XJ 9c: Change "longcontrol.py"
--- XJ 9c: Change "controlsd.py"
--- OK CBQ 9 solved. Due to CB PowerOff (or "self.speed_pid.update") not only C2 PowerOff.
--- XJ 9d: Change "longcontrol.py" "controlsd.py" for CB speed

===== Details of Coding Notes =====

240207: NG MC3 MC6 MC7: PC: Run Change "train_modelB6.py"
--- Redo MC3: LogCosh + AdamW
--- MC3a3: MC3a2 + "warm restart" (use previous weights from B6BW.hdf5 by load_weights())
    BATCH_SIZE, STEPS, EPOCHS = 1, 40, 60 (about 1 hr on --37 for both Train and Valid)
    AdamW(learning_rate=lr_schedule, weight_decay=0.004, clipvalue=1.0)
    EarlyStopping(monitor='val_loss', min_delta=0, patience=10, verbose=1, mode='min', restore_best_weights=True)
--- Do BATCH_SIZE > 1 (for GPUs)
    ExponentialDecay(initial_learning_rate=0.0001, decay_steps=10000, decay_rate=0.9) returns a
      decayed learning rate using the function:
      def decayed_learning_rate(step):
        return initial_learning_rate * decay_rate ^ (step / decay_steps)
      decay_rate = final_learning_rate / initial_learning_rate
    datagen_debug(100, camera_file), BATCH_SIZE=100 >
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ python datagenB6.py
      #--- yuvX.shape = (1199, 6, 128, 256) > yuvX_len = len(yuvX) = 1199 Frames (F)
      #--- lastIdx = 1097
      #--- Epoch: 1  Step: 1  CFile: 1  i: 0
      #--- Epoch: 1  Step: 2  CFile: 1  i: 100 ...
      #--- Epoch: 1  Step: 11  CFile: 1  i: 1000
    > Steps = yuvX_len/BATCH_SIZE = 1200/BATCH_SIZE >
--- MC3b: MC3 + "cold start" (use new random weights)
    BATCH_SIZE, STEPS, EPOCHS = 4, 10, 6  # for quick training  (Time: 00:03:36) decay_steps=120
    #EarlyStopping()
--- MC3b1: MC3b + "warm restart" +
    BATCH_SIZE, STEPS, EPOCHS = 4, 20, 60 (on --33, --37 (Train) and --32 (Valid))
    ExponentialDecay(initial_learning_rate=0.01, decay_steps=1200, decay_rate=0.1)
--- Do 1: decay_steps = STEPS*EPOCHS and PrintLearningRate(Callback)
    final_learning_rate = initial_learning_rate * decay_rate ^ (step / decay_steps)
    decay_rate^t = final_learning_rate/initial_learning_rate = 1e-4/1e-2 > t = 1
    > decay_steps = STEPS*EPOCHS = 1200 (BATCH_SIZE*STEPS*EPOCHS = 4800 F = 2 Epochs of 2 vedios)
     Epoch: 1 , LR: 0.00964
    20/20 [===] - 408s 20s/step - loss: 487403.0000 - maxae: 177456864.0000 - val_loss: 1983017.0000 - val_maxae: 647565440.0000 - lr: 0.0096
     Epoch: 60 , LR: 0.001
    20/20 [===] - 238s 13s/step - loss: 0.6257 - maxae: 59.0871 - val_loss: 1.1577 - val_maxae: 58.4937 - lr: 0.0010
    #--- Training Time: 04:02:18.18
     Epoch: 60 , LR: 0.001
    20/20 [===] - 249s 13s/step - loss: 0.6432 - maxae: 73.2415 - val_loss: 0.9098 - val_maxae: 73.1428 - lr: 0.0010
    #--- Training Time: 04:21:45.49
--- NG: maxae: 73.2415
--- MC3b2: MC3b1 + "warm restart" +
    BATCH_SIZE, STEPS, EPOCHS = 2, 10, 60 (on --33, --37 (Train) and --32 (Valid))
    ExponentialDecay(initial_learning_rate=0.001, decay_steps=300, decay_rate=0.1) > t = 2
    Epoch 21: val_loss improved from 0.50212 to 0.47908, saving model to ./saved_model/B6BW.hdf5
     Epoch: 21 , LR: 0.000201
    10/10 [===] - 69s 8s/step - loss: 0.2590 - maxae: 90.7798 - val_loss: 0.4791 - val_maxae: 90.4242 - lr: 2.0106e-04
--- NG MC3a MC3b: maxae: 90.7798
--- MC6: loss = KB.max(KB.abs(y_pred - y_true), axis=-1)
    BATCH_SIZE, STEPS, EPOCHS = 2, 10, 60 (on --33, --37 (Train) and --32 (Valid))
    ExponentialDecay(initial_learning_rate=0.001, decay_steps=300, decay_rate=0.1) > t = 2
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ python serverB6.py --port 5557
    #--- datagenB6  OK: cfile = /home/jinn/dataB6/UHD--2018-08-02--08-34-47--37/yuv.h5
    #--- datagenB6  OK: cfile = /home/jinn/dataB6/UHD--2018-08-02--08-34-47--33/yuv.h5
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ python serverB6.py --port 5558 --validation
    #--- datagenB6  OK: cfile = /home/jinn/dataB6/UHD--2018-08-02--08-34-47--32/yuv.h5
     Epoch: 1 , LR: 0.000933
    10/10 [===] - 123s 11s/step - loss: 87.1914 - maxae: 87.1914 - val_loss: 84.4799 - val_maxae: 84.4799 - lr: 9.3325e-04
    Epoch 21: val_loss improved from 48.24783 to 47.88553, saving model to ./saved_model/B6BW.hdf5
     Epoch: 21 , LR: 0.000201
    10/10 [===] - 64s 7s/step - loss: 30.6499 - maxae: 30.6499 - val_loss: 47.8855 - val_maxae: 47.8855 - lr: 2.0106e-04
    Epoch 60: val_loss did not improve from 22.99025
     Epoch: 60 , LR: 1.01e-05
    10/10 [===] - 69s 8s/step - loss: 23.8572 - maxae: 23.8572 - val_loss: 24.4855 - val_maxae: 24.4855 - lr: 1.0077e-05
    #--- Training Time: 01:06:08.37
--- LR: 1.01e-05; val_loss = val_maxae: 24.4855
--- MC6a: MC6 + "warm restart" +
    ExponentialDecay(initial_learning_rate=0.01, decay_steps=200, decay_rate=0.1) > t = 3
    Epoch 1: val_loss improved from inf to 76.76894, saving model to ./saved_model/B6BW.hdf5
     Epoch: 1 , LR: 0.00902
    Epoch 11: val_loss improved from 26.15228 to 24.12637, saving model to ./saved_model/B6BW.hdf5
     Epoch: 11 , LR: 0.00285
    Epoch 26: val_loss improved from 22.60321 to 20.10715, saving model to ./saved_model/B6BW.hdf5
     Epoch: 26 , LR: 0.000507
    Epoch 60: val_loss did not improve from 20.10715
     Epoch: 60 , LR: 1.01e-05
    10/10 [===] - 63s 7s/step - loss: 23.4769 - maxae: 23.4769 - val_loss: 41.0301 - val_maxae: 41.0301 - lr: 1.0116e-05
--- LR: 0.00285; val_loss: 20.10715
--- MC6a1: MC6a + "warm restart" +
    ExponentialDecay(initial_learning_rate=0.001, decay_steps=300, decay_rate=0.1) > t = 2
    Epoch 36: val_loss improved from 22.30979 to 21.76048, saving model to ./saved_model/B6BW.hdf5
     Epoch: 36 , LR: 0.00016
    Epoch 60: val_loss did not improve from 21.76048
     Epoch: 60 , LR: 1.01e-05
    10/10 [===] - 86s 9s/step - loss: 22.2847 - maxae: 22.2847 - val_loss: 30.1188 - val_maxae: 30.1188 - lr: 1.0116e-05
    #--- Training Time: 01:12:52.44
--- NG MC6: maxae: 21.76048 <! MC5d: best maxae: 1.7771
--- S 4: a) OK: decay_steps = STEPS*EPOCHS, PrintLearningRate(Callback)
         b) "cold start": use new random weights, "warm restart": by load_weights()
         c) NG MC6: loss = KB.max(KB.abs(y_pred - y_true),) > maxae: 21.76048 <! MC5d: 1.7771
--- Do 2: CosineDecayWarmup:
          Learning Rate Warmup with Cosine Decay in Keras/TensorFlow
          Cosine Annealing Explained; 模型壓縮及優化 — Learning rate; 深度學習Warm up策略在幹什麼?
--- MC7: (MC5: cosine_similarity + mse) + CosineDecayWarmup
    BATCH_SIZE, STEPS, EPOCHS = 4, 20, 60
    total_epoches, warmup_epoches, hold = EPOCHS, 6, 4  # must: total_epoches > warmup_epoches + hold
    Epoch 1: val_loss improved from inf to 397.40265, saving model to ./saved_model/B6BW.hdf5
    20/20 [===] - 420s 21s/step - loss: 471.7911 - maxae: 245.0437 - val_loss: 397.4026 - val_maxae: 225.4516 - lr: 0.0000e+00
    Epoch 4: val_loss improved from 125.56313 to 8.71620, saving model to ./saved_model/B6BW.hdf5
    20/20 [===] - 241s 13s/step - loss: 47.5093 - maxae: 107.0085 - val_loss: 8.7162 - val_maxae: 50.3034 - lr: 5.0000e-04
     epoch: 4 , LR: 0.0005
    20/20 [===] - 242s 13s/step - loss: 5.1451 - maxae: 26.6448 - val_loss: 3.5196 - val_maxae: 15.7614 - lr: 6.6667e-04
     epoch: 5 , LR: 0.000667
    Epoch 19: val_loss improved from 2.32436 to 0.66384, saving model to ./saved_model/B6BW.hdf5
    20/20 [===] - 244s 13s/step - loss: 0.2263 - maxae: 14.3577 - val_loss: 0.6638 - val_maxae: 11.3398 - lr: 7.9389e-04
     epoch: 19 , LR: 0.000794
    1/1 [===] - 0s 213ms/steposs: -0.0346 - maxae: 4.6845
    Epoch 20: val_loss improved from 0.66384 to 0.54225, saving model to ./saved_model/B6BW.hdf5
    20/20 [===] - 243s 13s/step - loss: 0.0775 - maxae: 13.9315 - val_loss: 0.5422 - val_maxae: 17.4632 - lr: 7.7232e-04
     epoch: 20 , LR: 0.000772
    Epoch 21/60
    1/1 [===] - 0s 239ms/steposs: 0.0068 - maxae: 17.9928
    Epoch 21: val_loss improved from 0.54225 to 0.29605, saving model to ./saved_model/B6BW.hdf5
    Epoch 27: val_loss improved from 0.29605 to 0.29340, saving model to ./saved_model/B6BW.hdf5
     epoch: 28 , LR: 0.000578
    Epoch 29/60
    ***1/1 [===] - 0s 200ms/steposs: -0.2300 - maxae: 4.2998
    Epoch 51: val_loss improved from 0.29340 to 0.10138, saving model to ./saved_model/B6BW.hdf5
    Epoch 53: val_loss improved from 0.10138 to -0.02815, saving model to ./saved_model/B6BW.hdf5
    Epoch 60: val_loss did not improve from -0.02815
    20/20 [===] - 243s 13s/step - loss: 1.8278 - maxae: 23.7030 - val_loss: 4.1880 - val_maxae: 18.8096 - lr: 6.8523e-07
    #--- Training Time: 04:05:45
--- NG MC7: maxae: 4.2998 <! MC5d: best maxae: 1.7771
--- ToDo: Run MC7 on Server
--- ToDo: PC GPU
--- ToDo: Cycle Learning Rate
--- ToDo: Keras Learning Rate Finder

240124: NG MC3 MC4 MC5: NG RT 4: Pipe A B: S 1 S 2: PC: Run Change "train_modelB6.py"
--- Do RT 3:
--- Q1: PC data too small > RT 3 ??? Yes, see S 2 below.
--- Q2: Error in "datagenB6.py" (yuv.h5, Ximgs etc.) > RT 3 ??? No, see B6SimMC5f1.png
--- Q3: Need calibration > RT 3 ??? No, see B6SimMC5b*.png
--- Change "train_modelB6.py"
    optimizer = tf.keras.optimizers.AdamW(learning_rate=lr_schedule, weight_decay=0.004, clipvalue=1.0)
--- clipvalue=1.0 solves the "zig-zag" problem by T9 in 2309 AI Projects.docx
--- Google: Why AdamW matters
--- Do 1: Put "hevc2yuvh5.py" in "datagenB6.py" w/o "yuv.h5"
--- Save "train_modelB6", "datagenB6" to aJLLold/"*B6b" (keep old versions as B6b)
--- Change datagen_test() in "datagenB6.py"
--- Q4: Are cv2.COLOR_BGR2YUV_I420 == rgb2yuv = std::make_unique<Rgb2Yuv> ???
--- Q5: RGB != rgb ???
    ret, RGB = cap.read()  #--- ret =  True
    rgb = cv2.cvtColor(sYUV, cv2.COLOR_YUV2RGB_I420)
--- NG Do 1: cv2.VideoCapture got wrong total_frames in def datagen_test() for Testing
    print('#--- frame_count =', frame_count)
      #--- frame_count = 1199
    cap = cv2.VideoCapture(cfile)
    total_frames = cap.get(cv2.CAP_PROP_FRAME_COUNT)
    print('#--- total_frames =', total_frames)
      #--- total_frames = -192153584101141.0
--- stackoverflow:
    There are two methods to determine the number of frames in a video file
    Method #1: Utilize built-in OpenCV properties to access video file meta information
      which is fast and efficient but inaccurate
    Method #2: Manually loop over each frame in the video file with a counter
      which is slow and inefficient but accurate
--- Quit Do 1: I can't get specific frames by cv2 in def datagen() for Training
--- Save to "datagenB6c"
--- datagen_test() Testing Time:
    "B6b": 00:00:04.36
    "B6c": 00:00:04.57 (bit slower)
--- Change "datagenB6.py" = "B6b" (datagen()) + "B6c" (datagen_test() good for vedio show)
    if i != 0:
      Xin3[bcount] = Xin3_temp  #--- np.shape(Xin3_temp) = (512,)
    outs = supercombo_y(Ximgs[bcount], Xin1[bcount], Xin2[bcount], Xin3[bcount])
--- Xin3: RNN state > outs (prediction, Xin3_temp, batch): sequential >
    we can have BATCH_SIZE > 1 for parallel GPUs
--- EarlyStopping
    BATCH_SIZE, STEPS, EPOCHS = 1, 20, 58     # for long training on PC (Training Time: 00:47:04)
    early_stopping = EarlyStopping(monitor='val_loss', min_delta=0,
      patience=25, verbose=1, mode='min', restore_best_weights=True)
    Epoch 44: val_loss did not improve from 0.18060
    Restoring model weights from the end of the best epoch: 19.
    Epoch 44: early stopping
    #--- Training Time: 00:47:04.95
--- Redo MC2
--- MC3 = MC2: LogCosh + AdamW
    BATCH_SIZE, STEPS, EPOCHS = 1, 20, 60    # for long training on PC (Training Time: 01:09:34)
    Epochs (JLL) != EPOCHS (Keras); Steps (JLL) != STEPS (Keras); BATCH_SIZE = 1 due to RNN
    1 vedio = 1 CFile = 1148 Images = Steps = 1150 - 2, STEPS = 1148/20 = 58 > train 1 CFile
    ExponentialDecay(initial_learning_rate=0.0001, decay_steps=10000, decay_rate=0.9)
    AdamW(learning_rate=lr_schedule, weight_decay=0.004, clipvalue=1.0)
--- B6LossMC3.png: big loss on the "first" few-100 imges of CFile 1 in EPOCHS 0-10 (Epoch 1)
    and 50-60 (Epoch 2)
--- NG MC3: B6SimMC3.png
--- Pipe A: 1. train_modelB6.py (custom_loss*) > B6BW.hdf5, B6.keras > B6Loss*.png >
    2. simulatorB6.py > B6Sim*.png > 3. train_modelB6.py > B6Y*.png > GoTo 1 if needed >
--- Pipe B: 4. B6*.keras > h5topb.py > B6*.pb > 5. snpe > B6*.dlc >
    6. FTP > sftp://root@10.0.0.18:8022/data/openpilot/models/B6*.dlc > 7. Road Test
--- MC4: T9 > cosine_similarity + LogCosh
    loss_CS = tf.keras.losses.cosine_similarity(y_true, y_pred, axis=-1)  # MC4
    loss_LC = logcosh(y_true, y_pred)
--- NG MC4: B6SimMC4.png
--- MC5: cosine_similarity + mse
    loss_CS = tf.keras.losses.cosine_similarity(y_true, y_pred, axis=-1)  # MC4
    loss_MSE = tf.keras.losses.mse(y_true, y_pred)  # MC5
    BATCH_SIZE, STEPS, EPOCHS = 1, 20, 60
    Epoch 52: val_loss did not improve from 0.07099
    20/20 [===] - 64s 3s/step - loss: -0.0950 - rmse: 0.8956 - maxae: 4.6890
     - val_loss: 0.6550 - val_rmse: 1.5062 - val_maxae: 10.9242
    Epoch 53: val_loss did not improve from 0.07099
    20/20 [===] - 64s 3s/step - loss: -0.1705 - rmse: 0.8105 - maxae: 5.6364
     - val_loss: 0.5802 - val_rmse: 1.4628 - val_maxae: 8.9136
    Epoch 54: val_loss did not improve from 0.07099
    20/20 [===] - 65s 3s/step - loss: -0.1841 - rmse: 0.7942 - maxae: 5.7810
     - val_loss: 0.6952 - val_rmse: 1.5413 - val_maxae: 10.8250
    A***Epoch 55: val_loss did not improve from 0.07099
    20/20 [===] - 66s 3s/step - loss: -0.1595 - rmse: 0.8235 - maxae: 4.4668
     - val_loss: 130.6116 - val_rmse: 10.5735 - val_maxae: 80.8042
    Epoch 56: val_loss did not improve from 0.07099
    20/20 [===] - 65s 3s/step - loss: -0.0835 - rmse: 0.9093 - maxae: 9.1584
     - val_loss: 233.8933 - val_rmse: 21.5431 - val_maxae: 163.2652
    Epoch 58/60
    1/1 [===] - 0s 94ms/steploss: -0.1302 - rmse: 0.8577 - maxae: 4.5120
    1/1 [===] - 0s 144ms/steposs: -0.0641 - rmse: 0.9211 - maxae: 4.8169
    1/1 [===] - 0s 110ms/steposs: -0.0312 - rmse: 0.9543 - maxae: 4.8555
    B***1/1 [===] - 0s 86ms/steploss: 90.0223 - rmse: 6.9947 - maxae: 49.5760
    1/1 [===] - 0s 82ms/steploss: 98.8674 - rmse: 8.1871 - maxae: 58.8594
-- MC5a: BATCH_SIZE, STEPS, EPOCHS = 1, 1148, 2
    def maxae(y_true, y_pred):
    model.load_weights('./saved_model/B6BW.hdf5')
    model.save('./saved_model/B6.keras')
    2024-02-01 17:26:47.690893: W tensorflow/compiler/tf2tensorrt/utils/py_utils.cc:38] TF-TRT Warning: Could not find TensorRT
    Epoch 1/2
       B***1/1148 [...] - ETA: 5:53:06 - loss: 491.4980 - rmse: 31.3604 - maxae: 2461/1 [===] - 0s 77ms/step
       2/1148 [...] - ETA: 3:38 - loss: 382.8010 - rmse: 27.3971 - maxae: 213.371/1 [===] - 0s 82ms/step
       3/1148 [...] - ETA: 3:42 - loss: 336.2130 - rmse: 25.6211 - maxae: 198.681/1 [===] - 0s 73ms/step
    1/1 [===] - 0s 35ms/steps - loss: 1.6138 - rmse: 1.1517 - maxae: 11.3885
    1/1 [===] - 0s 36ms/steps - loss: 1.6124 - rmse: 1.1515 - maxae: 11.3827
    1/1 [===] - 0s 41ms/steps - loss: 1.9745 - rmse: 1.1757 - maxae: 11.5983
    1/1 [===] - 0s 50ms/step
    Epoch 1: val_loss improved from inf to 15.11439, saving model to ./saved_model/B6BW.hdf5
    /home/jinn/sconsvenv/lib/python3.8/site-packages/keras/src/engine/training.py:3000: UserWarning: You are saving your model as an HDF5 file via `model.save()`. This file format is considered legacy. We recommend using instead the native Keras format, e.g. `model.save('my_model.keras')`.
      saving_api.save_model(
    1148/1148 [===] - 5514s 5s/step - loss: 1.9745 - rmse: 1.1757 - maxae: 11.5983 - val_loss: 15.1144 - val_rmse: 4.1140 - val_maxae: 29.9322
    Epoch 2/2
    B***1/1 [===] - 0s 133ms/step03 - loss: 173.6710 - rmse: 18.6623 - maxae: 139.5968
    1/1 [===] - 0s 137ms/step07 - loss: 107.0631 - rmse: 14.0270 - maxae: 111.8339
    1/1 [===] - 0s 119ms/step10 - loss: 66.3612 - rmse: 9.7789 - maxae: 88.4097
--- MC5b: B*** > RNN state important > Xin3_temp
    Xin3_temp =  Xin3[bcount] #--- np.shape(Xin3_temp) = (512,)
    model.load_weights('./saved_model/B6BW.hdf5')
    model.save('./saved_model/B6.keras')
    BATCH_SIZE, STEPS, EPOCHS = 1, 1148, 2
    2024-02-01 21:16:51.864183: W tensorflow/compiler/tf2tensorrt/utils/py_utils.cc:38] TF-TRT Warning: Could not find TensorRT
    Epoch 1/2
    ***1/1 [===] - 0s 76ms/step:38:09 - loss: 256.4872 - rmse: 22.6595 - maxae: 334.3207
    1/1 [===] - 0s 75ms/step:47 - loss: 138.8968 - rmse: 14.6295 - maxae: 215.2983
    1/1 [===] - 0s 68ms/step:43 - loss: 99.5381 - rmse: 11.9288 - maxae: 167.7281
    1/1 [===] - 0s 72ms/step:43 - loss: 76.1232 - rmse: 9.8393 - maxae: 142.5742
    1/1 [===] - 0s 39ms/steps - loss: 0.1315 - rmse: 0.8248 - maxae: 9.7658
    1/1 [===] - 0s 38ms/steps - loss: 0.1314 - rmse: 0.8249 - maxae: 9.7617
    1/1 [===] - 0s 44ms/steps - loss: 0.1315 - rmse: 0.8252 - maxae: 9.7588
    1/1 [===] - 0s 44ms/step
    1/1 [===] - 0s 44ms/step
    Epoch 1: val_loss improved from inf to 25.78878, saving model to ./saved_model/B6BW.hdf5
    /home/jinn/sconsvenv/lib/python3.8/site-packages/keras/src/engine/training.py:3000: UserWarning: You are saving your model as an HDF5 file via `model.save()`. This file format is considered legacy. We recommend using instead the native Keras format, e.g. `model.save('my_model.keras')`.
      saving_api.save_model(
    1148/1148 [===] - 5965s 5s/step - loss: 0.1315 - rmse: 0.8252 - maxae: 9.7588 - val_loss: 25.7888 - val_rmse: 6.1846 - val_maxae: 46.6773
    Epoch 2/2
       C***1/1148 [...] - ETA: 3:14 - loss: 0.2945 - rmse: 1.2604 - maxae: 1/1 [===] - 0s 152ms/step
       2/1148 [...] - ETA: 3:41 - loss: 0.0591 - rmse: 1.0325 - maxae:    3/1148 [...] - ETA: 3:39 - loss: -0.0527 - rmse: 0.9113 - maxae:1/1 [===] - 0s 126ms/step
    1/1 [===] - 0s 36ms/steps - loss: -0.2250 - rmse: 0.7016 - maxae: 7.2352
    1/1 [===] - 0s 36ms/steps - loss: -0.2247 - rmse: 0.7019 - maxae: 7.2345
    ***1/1 [===] - 0s 40ms/steps - loss: -0.2244 - rmse: 0.7023 - maxae: 7.2400
    1/1 [===] - 0s 38ms/step
    1/1 [===] - 0s 40ms/step
--- OK MC5b: C***: Yes, RNN state is important.
--- Q6: MC1-MC5b use --33/yuv.h5 only ??? Yes.
    $ python serverB6.py --port 5557
    ['/home/jinn/dataB6/UHD--2018-08-02--08-34-47--33/yuv.h5']
--- Q7: --33/yuv.h5 for all MC1-MC5b => RT 3 ??? No, it is due to S 1 b).
    Why I can't get T9 ??? OK: see B6SimMC5f1.png.
    simulatorB6.py uses --33/yuv.h5
--- MC5b32: B6.keras > --32/video.hevc > B6SimMC5b32.png
--- MC5b33: B6.keras > --33/           > B6SimMC5b33.png
--- OP32:  supercombo079.keras > --32/ (frame_no = 2) > B6SimOP32.png
--- OP32i: supercombo079.keras > --32/ (frame_no = 3) > B6SimOP32i.png
--- OP33:  supercombo079.keras > --33/ (frame_no = 2) > B6SimOP33.png
--- OP33i: supercombo079.keras > --33/ (frame_no = 3) > B6SimOP33i.png
--- ToDo1: 1148 too big > save B6BW.hdf5 only once > 20, 60
--- MC5c: Do ToDo1 (1148 > 20, 60), Q7: Swap training and valid vedios:
    BATCH_SIZE, STEPS, EPOCHS = 1, 20, 60
    train_files = all_yuvs[train_len: train_len + valid_len]
    valid_files = all_yuvs[: train_len]
--- LossMC5c1, 5c2, 5c3: train_modelB6.py > 1st, 2nd, 3rd
--- SimMC5c1.png, c1B, c3: B6.keras > --32, --33
--- OK ToDo1: SimMC5c1 (20, 60) is better than (>!) 5b1 (1148)
--- S 1: Summary 1:
    a) SimMC5c1 (20, 60) is better than (>!) 5b1 (1148)
    b) Trapped in Local Minimum: EarlyStopping: B6LossMC5c1 >! c2 >! c3
    c) b) > SimMC5c1 (shorter training) >! c3 (longer)
    d) RNN state important > Xin3_temp, batch: sequential > OK: parallel GPUs
--- Q8: How to Overcome Local Minimum?
    Tuning Your Keras SGD Neural
    strategies to help mitigate the local minimum problem
    Minimizing Worst-Case Violations of Neural Networks
--- MC5d1: Train and Valid only --32/yuv.h5 (1st)
    BATCH_SIZE, STEPS, EPOCHS = 1, 20, 60
    ***Epoch 16: val_loss improved from -0.34826 to -0.40022, saving model to ./saved_model/B6BW.hdf5
    Epoch 18/60
    ***1/1 [===] - 0s 99ms/steploss: -0.3689 - rmse: 0.5119 - maxae: 2.2601
    Epoch 41/60
    1/1 [===] - 0s 84ms/steploss: -0.4148 - rmse: 0.4128 - maxae: 5.6290
--- MC5d2: continue training (2nd)
    Epoch 1/60
    1/1 [===] - 0s 74ms/step- loss: 9.9229 - rmse: 4.5656 - maxae: 36.8336
    ***Epoch 3: val_loss improved from -0.26341 to -0.38476, saving model to ./saved_model/B6BW.hdf5
    Epoch 7/60
    ***1/1 [===] - 0s 120ms/steposs: -0.4250 - rmse: 0.3873 - maxae: 1.7771
    Restoring model weights from the end of the best epoch: 3.
    Epoch 28: early stopping
    #--- Training Time: 00:31:28.35
--- OK MC5d2: Train and Valid only --32 (ignor val_loss) > Best maxae: 1.7771
--- MC5d3: 1, 1148, 2
    BATCH_SIZE, STEPS, EPOCHS = 1, 1148, 2
    2024-02-02 16:58:02.953722: W tensorflow/compiler/tf2tensorrt/utils/py_utils.cc:38] TF-TRT Warning: Could not find TensorRT
    Epoch 1/2
    1/1 [===] - 0s 37ms/step1:48 - loss: 10.9293 - rmse: 3.9771 - maxae: 52.0365
    1/1 [===] - 0s 34ms/step1:50 - loss: 10.7152 - rmse: 3.9322 - maxae: 52.0928
    1/1 [===] - 0s 34ms/step1:57 - loss: 10.5019 - rmse: 3.8848 - maxae: 52.1109
    1/1 [===] - 0s 35ms/steps - loss: 0.1441 - rmse: 0.7511 - maxae: 9.9128
    1/1 [===] - 0s 36ms/steps - loss: 0.1437 - rmse: 0.7510 - maxae: 9.9072
    1/1 [===] - 0s 40ms/steps - loss: 0.1433 - rmse: 0.7507 - maxae: 9.9042
    1/1 [===] - 0s 52ms/step
    Epoch 1: val_loss improved from inf to 11.99431, saving model to ./saved_model/B6BW.hdf5
    Epoch 2/2
    1/1 [===] - 0s 39ms/step:28 - loss: -0.3593 - rmse: 0.5156 - maxae: 5.6263
    1/1 [===] - 0s 39ms/step:39 - loss: -0.3595 - rmse: 0.5155 - maxae: 5.5803
    1/1 [===] - 0s 38ms/step0:36 - loss: -0.3608 - rmse: 0.5131 - maxae: 5.5612
    1/1 [===] - 0s 35ms/step1:41 - loss: -0.3724 - rmse: 0.4928 - maxae: 4.8621
    1/1 [===] - 0s 36ms/step3:11 - loss: -0.3025 - rmse: 0.5933 - maxae: 8.2601
    1/1 [===] - 0s 40ms/steps - loss: -0.3107 - rmse: 0.5842 - maxae: 6.8703
    1/1 [===] - 0s 36ms/steps - loss: -0.3108 - rmse: 0.5841 - maxae: 6.8688
    1/1 [===] - 0s 44ms/steps - loss: -0.3105 - rmse: 0.5845 - maxae: 6.8690
    1/1 [===] - 0s 44ms/step
    Epoch 2: val_loss improved from 11.99431 to 3.78566, saving model to ./saved_model/B6BW.hdf5
    1148/1148 [===] - 5547s 5s/step - loss: -0.3105 - rmse: 0.5845 - maxae: 6.8690 - val_loss: 3.7857 - val_rmse: 2.7431 - val_maxae: 21.0124
    #--- Training Time: 03:09:29.83
--- NG MC5d3: maxae: 3.78566 !< 1.7771
--- MC5e: MC5d + 1, 1148, 5
--- NG MC5e: SimMC5e (1148, 5) <! MC5d3 (1148, 2)
--- MC5: cosine_similarity + mse
--- MC5f: MC5 + B6BW32: both trained and validated on --32/yuv.h5 w/o --33
    All previous B6BW.hdf5 are trained on --33 (B6BW33) in begining
    BATCH_SIZE, STEPS, EPOCHS = 1, 1148, 2
    1/1 [===] - 0s 36ms/step3:10 - loss: 221.5458 - rmse: 20.8595 - maxae: 176.0556
    1/1 [===] - 0s 35ms/step5:08 - loss: 227.5007 - rmse: 21.1314 - maxae: 177.5765
    Epoch 2: val_loss improved from 23.98432 to 4.88490, saving model to ./saved_model/B6BW.hdf5
    1148/1148 [===] - 5390s 5s/step - loss: -0.2593 - rmse: 0.6821 - maxae: 6.2213 - val_loss: 4.8849 - val_rmse: 2.5069 - val_maxae: 20.6629
    #--- Training Time: 03:04:13.84
--- NG MC5f: maxae: 6.2213 !< 1.7771
--- MC5f1: Do 2401賴佩昕Report 3
    a) BATCH_SIZE, STEPS, EPOCHS = 1, 2, 2
    2) initial_learning_rate=0.0001
    Epoch 2: val_loss improved from 236.75482 to 199.71797, saving model to ./saved_model/B6BW.hdf5
    2/2 [===] - 10s 10s/step - loss: 153.1056 - rmse: 17.4827 - maxae: 137.9397 - val_loss: 199.7180 - val_rmse: 19.9826 - val_maxae: 158.0679
    #--- Training Time: 00:00:39.03
--- OK: B6SimMC5f1.png
--- Q9: SimMC5f1 (00:00:39) better than MC5f (03:09:29) ??? Yes for Sim but No for RT 4
--- Do Road Test: 4. B6.keras > h5topb.py > B6*.pb > 5. snpe > SimMC5f1.dlc > 6. FTP > 7. Road Test
--- Do SnG on C2A: Change "carcontroller.py" to
    /jinn/CarXJ/openpilot/selfdrive/car/toyota/carcontroller079X.py
    Lines 92, 93 New:
      #if CS.out.standstill and not self.last_standstill and CS.CP.carFingerprint not in NO_STOP_TIMER_CAR:
      #  self.standstill_req = True
--- NG RT 4: 240204 Road Test Error on B6CMf51.dlc: Same as RT 2
--- OK: SnG on C2A
--- Q10: Prius SnG needs smartDSU ???
--- S 2: Summary 2: Need to
    a) beat MC5d: Train and Valid only --32 (or --33) (ignor val_loss) + (20, 60) > Best maxae: 1.7771
    b) have "both" good B6Sim*.png and B6Y*.png
    c) solve Q8: How to Overcome Local Minimum?
       > SimMC5c1 (20, 60, better escaping L Min) >! 5b1 (1148)
       > SimMC5e (1148, 5, longer) <! (worse) MC5d3 (1148, 2, shorter)
       > use MC5c1 (20, 60) > patience=25 or less
    d) solve Q9: SimMC5f1 (00:00:39) >! MC5f (03:09:29) ??? Yes for Sim but No for RT 4
    e) solve Q11: Why all B6Sim*.png show "Left" curves ??? Due to the data --32 and --33 ???
    f) do Q12: Which of MC3: LogCosh and MC5: cosine_similarity + mse is better ???
    g) use Server for much longer training on bigger data
--- S 2e: For Q11: Add --37 to dataB6
--- Change yuvX.shape from 1150 (yuv1150.h5) to 1200:
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ python hevc2yuvh5.py
--- S 2a: train and validate same --37/yuv.h5
    train_files = all_yuvs[0:1]
    valid_files = all_yuvs[0:1]
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ python serverB6.py --port 5557
    #--- datagenB6  OK: cfile = /home/jinn/dataB6/UHD--2018-08-02--08-34-47--37/yuv.h5
    #--- yuvX.shape = (1200, 6, 128, 256)
--- MC3: LogCosh + AdamW
--- MC3a: MC3 + S 2a (on --37 > B6BW37) + S 2c (20, 6)
--- MC3a1: MC3a + S 2c (40, 60)
    Epoch 1/60
     1/40 [...] - ETA: 11:48 - loss: 10.4436 - rmse: 24.2432 - maxae:
     2/40 [>..] - ETA: 7s - loss: 10.8606 - rmse: 24.3862 - maxae: 326.1
    Epoch 60: val_loss did not improve from 0.08604
    40/40 [===] - 147s 4s/step - loss: 0.3097 - rmse: 1.1840 - maxae: 22.6731 - val_loss: 1.7021 - val_rmse: 4.2809 - val_maxae: 42.9407
    #--- Training Time: 02:11:36.81
--- MC5: cosine_similarity + mse + AdamW
--- MC5g: MC5 + S 2a (--37) + S 2c (20, 6)
--- MC5g1: MC5g + S 2c (40, 60)
    Epoch 60: val_loss did not improve from -0.15270
    40/40 [===] - 124s 3s/step - loss: 0.2242 - rmse: 1.0879 - maxae: 12.9703 - val_loss: 5.8477 - val_rmse: 3.5351 - val_maxae: 53.5602
    #--- Training Time: 02:11:35.75
--- S 3: a) SimCM: 3a1 >! 5g1 but maxae: 5g1 >! 3a1; both NG Q11: Why all B6Sim*.png show "Left" curves
--- MC3a2: MC3a1.keras + one more (40, 60) (RSRT: restart $ python serverB6.py, restart $ python train_modelB6.py)
    EarlyStopping(,,patience=10,,,)
    model.load_weights('./saved_model/B6BW.hdf5')
    Epoch 11/60
    1/1 [===] - 0s 154ms/steposs: 0.0894 - rmse: 0.5388 - maxae: 8.9505
    Epoch 11: val_loss improved from 0.13130 to 0.09487, saving model to ./saved_model/B6BW.hdf5
    40/40 [===] - 135s 3s/step - loss: 0.0969 - rmse: 0.5706 - maxae: 10.3929 - val_loss: 0.0949 - val_rmse: 0.5391 - val_maxae: 8.9630
    Epoch 12/60
    1/1 [===] - 0s 92ms/steploss: 0.0707 - rmse: 0.4723 - maxae: 8.9883
    1/1 [===] - 0s 119ms/steposs: 0.0719 - rmse: 0.4746 - maxae: 8.9502
    1/1 [===] - 0s 144ms/steposs: 0.0815 - rmse: 0.5079 - maxae: 8.9432
    Epoch 21: val_loss did not improve from 0.09487
    Restoring model weights from the end of the best epoch: 11.
    40/40 [===] - 123s 3s/step - loss: 0.1256 - rmse: 0.6272 - maxae: 10.0307 - val_loss: 0.2376 - val_rmse: 0.9242 - val_maxae: 12.0026
    Epoch 21: early stopping
    #--- Training Time: 00:47:43.69
--- MC3a3: MC3a2 + continue training (KSRT: keep $ python serverB6.py, restart $ python train_modelB6.py)
    Epoch 1: val_loss improved from inf to 0.18999, saving model to ./saved_model/B6BW.hdf5
    /home/jinn/sconsvenv/lib/python3.8/site-packages/keras/src/engine/training.py:3000: UserWarning: You are saving your model as an HDF5 file via `model.save()`. This file format is considered legacy. We recommend using instead the native Keras format, e.g. `model.save('my_model.keras')`.
    Epoch 15: val_loss improved from 0.11783 to 0.11049, saving model to ./saved_model/B6BW.hdf5
    Epoch 18: val_loss improved from 0.11049 to 0.07173, saving model to ./saved_model/B6BW.hdf5
    Epoch 28: val_loss did not improve from 0.07173
    Restoring model weights from the end of the best epoch: 18.
    Epoch 28: early stopping
    #--- Training Time: 01:01:38.19
--- S 3: b) SimCM: 3a3 >! 3a1 >! 3a: longer training better
         c) KSRT: keep serverB6, restart train_modelB6 to escape L Min
         d) Use MC3 (better Sim) or MC5 (better maxae)

240123: NG. Md 2: PC: "modelB7" w/o .py
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ python h5totf.py
--- OK: modelB6_Huber.h5 > modelB6_Huber.pb
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ python h5totf.py
      WARNING:tensorflow:No training configuration found in the save file,
      so the model was *not* compiled. Compile it manually.
--- OK: supercombo079.keras > supercombo079.pb
--- OK: modelB6.summary, supercombo079.summary
--- ToDo Md 2: "supercombo079.summary" > write "modelB7.py"
--- Tensorflow Keras Copy Weights From One Model to Another
    def buildModel():
      model = tf.keras.models.Sequential()
      model.compile(loss="mean_squared_error", optimizer=opt)
      return model
    def updateTargetModel(model, targetModel):
      modelWeights       = model.trainable_weights
      targetModelWeights = targetModel.trainable_weights
      for i in range(len(targetModelWeights)):
        targetModelWeights[i].assign(modelWeights[i])
    target_model.set_weights(model.get_weights())
--- Copy "supercombo079.keras" to "supercomboB7.keras" > train modelB7 w/o .py
    model = keras.models.load_model('models/supercombo079.keras', inputs=in0_in3, outputs=outs, name='modelB7')
    supercombo = load_model('models/supercombo079.keras', compile=False)
--- Do 1: "supercombo079.keras" > "modelB7" w/o .py
--- Q1: compile=False in keras.models.load_model(,,) ???
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ python train_modelB7.py
      TypeError: load_model() got an unexpected keyword argument 'inputs'
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ python train_modelB7.py --port 5557 --port_val 5558
      File "train_modelB7.py", line 138, in <module>
        history = model.fit(
      File "train_modelB7.py", line 68, in get_data
        loss = custom_loss(Ytrue, p)
      ValueError: Can't convert non-rectangular Python sequence to Tensor.
--- NG Do 1:

240121: OK RT 2: RT 3: UI 2d: C2A+car: Road Test: "B6MC2.dlc"
--- Road Test: 231229 Pipe 1: "supercombo079.dlc" = "B6MC2.dlc"
--- OK RT 2: "No close lead car" is solved.
--- RT 3: 240121 Road Test Error: "Path Prediction Error":
    https://drive.google.com/file/d/1FksGyMMuE5_zVYFufA_6JiBPxfdebk5U/view?usp=sharing
--- How do I get the weights of a layer in Keras?
--- Change "train_modelB6.py"
    model.load_weights('models/supercombo079.keras')
    print("#--- len(model.layers) =", len(model.layers))
    #model.load_weights('./saved_model/modelB6_LogCosh.keras')
      #--- len(model.layers) = 210
--- Error:
    File "train_modelB6.py", line 255, in <module>
      model.load_weights('models/supercombo079.keras')
    ValueError: Layer count mismatch when loading weights from file. Model expected 124 layers, found 57 saved layers.
--- convert .keras (.h5) to .py, h5pyViewer: view .keras files
--- konverter supercombo079.keras supercombo079.py
    (sconsvenv) jinn@Liu:~/openpilot$ pip install keras-konverter
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model/models$ konverter supercombo079.keras supercombo079.py
      RuntimeError: module compiled against API version 0xf but this version of numpy is 0xd
      ImportError: numpy.core.multiarray failed to import
    (sconsvenv) jinn@Liu:~/openpilot$ pip install numpy --upgrade
      Exception: Input model must be a Sequential tf.keras model, not <class 'keras.src.engine.functional.Functional'>
--- NG: konverter
--- UI 2d: Add "啟用行車記錄功能" to OP079C2 for seeing "Path" by "B6MC2.dlc"

240113: PC: Run OP095/"get_model_metadata.py"
--- for 2401_Auto.pptx: OP092/modeld.cc ModelOutput, OP095/modeld.py, get_model_metadata.py
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/OP095$ python get_model_metadata.py
      #--- [x.key for x in model.metadata_props] = ['output_slices', 'vision_model', 'policy_model', 'nav_model']
    metadata['output_shapes'] = dict([get_name_and_shape(x) for x in model.graph.output])
    OP095: /selfdrive/modeld/parse_model_outputs.py
        self.parse_mdn('plan', outs, in_N=ModelConstants.PLAN_MHP_N, out_N=ModelConstants.PLAN_MHP_SELECTION,
                       out_shape=(ModelConstants.IDX_N,ModelConstants.PLAN_WIDTH))
      PLAN_MHP_SELECTION = 1
      def parse_mdn(self, name, outs, in_N=0, out_N=1, out_shape=None):
        raw = outs[name]
        raw = raw.reshape((raw.shape[0], max(in_N, 1), -1))
          full_shape = tuple([raw.shape[0], in_N] + list(out_shape))
    IDX_N = 33
    PLAN_WIDTH = 15
    out_shape=(IDX_N, PLAN_WIDTH)
    PLAN_MHP_N = 5
    full_shape = tuple([1, PLAN_MHP_N] + list(out_shape))
    #print(list(out_shape))  # [33, 15]
    print(full_shape)  # (1, 5, 33, 15)

240103: Server A: "WiFi" or "Bluetooth" Wireless Connection
--- Server A: https://docs.google.com/document/d/1kNwWhDpnPnuvkF7LHwQRjBp4p7kQii7X/edit
--- How to enable Bluetooth at startup on Ubuntu 20.04 LTS?
    $ sudo systemctl enable bluetooth
    Connection to Bluez failed [closed]
    $ sudo /etc/init.d/bluetooth start
    [ ok ] Starting bluetooth (via systemctl): bluetooth.service.
--- click on "Bluetooth Manager" in Server
--- NG: Unable to connect to BlueZ
    $ sudo service bluetooth start
--- NG: Unable to connect to BlueZ
--- https://bluez-cheat-sheet.readthedocs.io/en/latest/
    $ sudo bluetoothctl
    Wating to connect to bluetoothd ...
--- Ctl c
--- click on "Bluetooth Manager" in Server
--- OK: Click "Enable Bluetooth"
    $ sudo bluetoothctl
    Agent registered
    [bluetooth]# list
--- NG: nothing
    [bluetooth]# scan on
    No default controller available
    bluetoothctl No default controller available [closed]

231229: Pipe 1: MC1: MC2: PC: Run Change "train_modelB6.py", "custom_loss"
--- Do RT 2: Change "train_modelB6.py" > Test model by only 2 images
--- 2309 AI Projects.docx > Q5 Q&A 231127 > 240109: Poster Presentation >
--- Change "custom_loss" and Do the "Training Loss" figures of T5動機碩 and the "Prediction" figures of T9動機系
    logcosh = tf.keras.losses.LogCosh()
--- MC1 = Md Case 1: LogCosh + AdamW, BATCH_SIZE, STEPS, EPOCHS = 1, 20, 48
--- MC2 = Md Case 2: LogCosh + AdamW, BATCH_SIZE, STEPS, EPOCHS = 1, 1148, 4
--- Pipe 1: train_modelB6.py > B6MC2.keras > h5topb.py > B6MC2.pb > B6MC2.dlc > Road Test

231217: OK UI: PC: Run Read "PlotJuggler"
    (sconsvenv) jinn@Liu:~/openpilot$ ./tools/plotjuggler/juggle.py --demo
      PlotJuggler is missing. Downloading...
      Loading compatible plugins from directory:  "/home/jinn/openpilot/tools/plotjuggler/bin"
      "libDataLoadRlog.so is a DataLoader plugin"
      Loaded DBC: toyota_new_mc_pt_generated
      Loading schema: "/home/jinn/openpilot/cereal/log.capnp"
      "libDataStreamCereal.so is a DataStreamer plugin"
      Number of plugins loaded:  2 ...
      Done reading Rlog data
--- Read PlotJugger Help
    (sconsvenv) jinn@Liu:~/openpilot$ ./tools/plotjuggler/juggle.py --stream
--- OK
--- Load local data from a file
    (sconsvenv) jinn@Liu:~/openpilot$ ./tools/plotjuggler/juggle.py "/home/jinn/dataC/8bfda98c9c9e4291|2020-05-11--03-00-57--61"
--- OK

231211: OK Md 4b: NG Md 4c: Md 4d: PC: "onnx2py.py" > "supercombo092.py"
--- move /home/jinn/openpilot/selfdrive/modeld/models/supercombo092.onnx
--- to   /home/jinn/sconsvenv/lib/python3.8/site-packages/supercombo092.onnx
--- Md 4b: "onnx2py.py" > "supercombo092.py"
    jinn@Liu:~/sconsvenv/lib/python3.8/site-packages$ python -m onnxconverter_common.onnx2py supercombo092.onnx supercombo092.py
--- Error
      File "/home/jinn/sconsvenv/lib/python3.8/site-packages/onnxconverter_common/onnx2py.py", line 228, in convert_model
        return helper_traced.make_model(opset_imports=opset_imports, **fields, graph=graph)
      File "/home/jinn/sconsvenv/lib/python3.8/site-packages/onnxconverter_common/pytracing.py", line 86, in __call__
        py_obj = TracingObject.get_py_obj(self)(*arg_o, **kwarg_o)
      File "/home/jinn/sconsvenv/lib/python3.8/site-packages/onnx/helper.py", line 307, in make_model
        setattr(model, k, v)
    AttributeError: Assignment not allowed to repeated field "metadata_props" in protocol message object.
--- Change Lines 305 - 308 of helper.py to
    #for k, v in kwargs.items():
        #setattr(model, k, v)
--- OK Md 4b: /home/jinn/sconsvenv/lib/python3.8/site-packages/supercombo092.py
    Model saved to supercombo092.py
--- move supercombo092.py .onnx back to /home/jinn/openpilot/selfdrive/modeld/models/
--- supercombo092.py (in 'pytorch') not useful
--- Md 4c: onnx2torch => pytorch2keras
--- install onnx_opcounter
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ pip install onnx_opcounter
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ onnx_opcounter /home/jinn/snpe/dlc/supercombo0950.onnx
    Number of parameters in the model: 24145568
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ onnx_opcounter --calculate-macs /home/jinn/snpe/dlc/supercombo0950.onnx
      File "/home/jinn/sconsvenv/lib/python3.8/site-packages/onnx_opcounter/onnx_opcounter.py", line 64, in calculate_macs
        dtype=type_mapping[graph_input.type.tensor_type.elem_type])
    KeyError: 10
--- NG Md 4c >
--- Md 4d: "h5toonnx" (OK Md 4a) + "onnx2py" (OK Md 4b) + (NG Md 4c) > Hand code py

231121: PID 2: Pipe 1-3: PC: Read "LatControlTorque" + Run Read "test_latcontrolJLL.py"
--- PID 2: [Liu23a] + "PIDController" + "LatControlPID" + "LatControlTorque"
--- Read "self.LaC = LatControlTorque"
--- 230213 Pipe 2: controlsd.py > main() > Controls() > self.CI, self.CP = get_car() >
    /selfdrive/controls/controlsd.py
      elif self.CP.lateralTuning.which() == 'torque':
        self.LaC = LatControlTorque(self.CP, self.CI)
--- Read "lateralTuning"
    /OP092/selfdrive/car/toyota/interface.py
    class CarInterface(CarInterfaceBase):
      def _get_params(ret, candidate,,,,):
        ret.carName = "toyota"
        if candidate in ANGLE_CONTROL_CAR:
        else:  # CAR.PRIUS
          CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
            /selfdrive/car/interfaces.py
            class CarInterfaceBase(ABC):
              def get_params(cls, candidate: str,,,,):
                ret = CarInterfaceBase.get_std_params(candidate)
                ret = cls._get_params(ret, candidate,,,,)
              def get_std_params(candidate):
                ret = car.CarParams.new_message()
                ret.carFingerprint = candidate
              def configure_torque_tune(candidate, tune,,):
                tune.init('torque')
        if candidate == CAR.PRIUS:
          stop_and_go = True
--- 230110: OK: Stop and Go (OP0812 + smartDSU in Toyota Prius)
    /selfdrive/controls/lib/latcontrol_torque.py
      class LatControlTorque(LatControl):
        def __init__(self, CP, CI):
          self.torque_params = CP.lateralTuning.torque
          self.pid = PIDController(self.torque_params.kp, self.torque_params.ki,
--- "LatControlTorque" > "PIDController"
--- Old "interface.py"
    /OP079C2/selfdrive/car/toyota/interface.py
      if candidate not in [CAR.PRIUS, CAR.RAV4, CAR.RAV4H]:  # These cars use LQR/INDI
      if candidate == CAR.PRIUS:
        ret.lateralTuning.init('indi')
--- Pipe 1: controlsd.py > Controls() > self.CP = get_car() > CP = CarInterface.get_params(candidate,,,,) >
    cls._get_params(,candidate,,,,) > CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning) >
    tune.init('torque')
--- Q: What (PRIUS, PRIUS_V, or PRIUS_TSS2) is my Prius ???
      PRIUS = "TOYOTA PRIUS 2017"
      PRIUS_V = "TOYOTA PRIUS v 2017"
      PRIUS_TSS2 = "TOYOTA PRIUS TSS2 2021"
    A: It is CAR.PRIUS from /home/jinn/OP/values075Liu.py
--- Q: But is CAR.PRIUS for values092.py in C2B ???
    A: Yes. See Line 144 in /home/jinn/CarXJ/values092.py.
--- Read "PRIUS"
    /car/toyota/values.py
    class CAR:
      PRIUS = "TOYOTA PRIUS 2017"
    @dataclass
    class ToyotaCarInfo(CarInfo):
--- Run CAN 32 for "@dataclass"
--- Read "candidate"
    /selfdrive/car/fw_versions.py
    def match_fw_to_car_fuzzy(fw_versions_dict, log=True, exclude=None):
              candidate = candidates[0]
    def match_fw_to_car(fw_versions, allow_exact=True, allow_fuzzy=True):
      exact_matches = []
        exact_matches.append((False, match_fw_to_car_fuzzy))
    /selfdrive/car/car_helpers.py
    def fingerprint(logcan, sendcan, num_pandas):
      fixed_fingerprint = os.environ.get('FINGERPRINT', "")
          car_fw = get_fw_versions_ordered(logcan, sendcan, ecu_rx_addrs, num_pandas=num_pandas)
        exact_fw_match, fw_candidates = match_fw_to_car(car_fw)
      if len(fw_candidates) == 1:
        car_fingerprint = list(fw_candidates)[0]
      if fixed_fingerprint:
        car_fingerprint = fixed_fingerprint
      return car_fingerprint,,,,, exact_match
    def get_car()
      candidate,,,,, = fingerprint(,,)
      CarInterface, CarController, CarState = interfaces[candidate]
      CP = CarInterface.get_params(candidate, fingerprints,,,)
      return CarInterface(CP, CarController, CarState), CP
--- Pipe 2: values.py > class CAR > PRIUS = "TOYOTA PRIUS 2017" > CAR.PRIUS > interface.py > candidate == CAR.PRIUS
--- Pipe 3: fw_versions.py > match_fw_to_car() > car_helpers.py > get_car() > candidate = fingerprint()
    controlsd.py > self.CP = get_car()
--- ToDo: Run 'FINGERPRINT', "launch_openpilotJLL.sh"
--- Read "LatControlPID"
--- Run Read "test_latcontrolJLL.py"
    #@parameterized.expand([(TOYOTA.PRIUS, LatControlPID)])
      #--- ERROR: test_saturation_0_TOYOTA_PRIUS_2017 (__main__.TestLatControl)
    #@parameterized.expand([(HONDA.CIVIC, LatControlPID)])
      #--- FAIL: test_saturation_0_HONDA_CIVIC_2016 (__main__.TestLatControl)
--- Q1: Why ERROR not FAIL ??? A1: HONDA.CIVIC uses LatControlPID but TOYOTA.PRIUS not
    if candidate == CAR.PRIUS:
          CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning, steering_angle_deadzone_deg=0.2)
            def configure_torque_tune(candidate, tune, steering_angle_deadzone_deg=0.0, use_steering_angle=True):
              tune.init('torque')
    #--- CP.lateralTuning.which() = torque
    if candidate == CAR.CIVIC:
        ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.3], [0.1]]
    #--- CP.lateralTuning.which() = pid
--- Run Read "test_latcontrolJLL.py"
    #@parameterized.expand([(HONDA.CIVIC, LatControlPID)])
      #--- FAIL: test_saturation_0_HONDA_CIVIC_2016 (__main__.TestLatControl)
    @parameterized.expand([(TOYOTA.PRIUS, LatControlAngle)])
      #--- OK
--- Q2: Why FAIL not OK ??? A2: controller.update() for LatControlAngle not LatControlPID
--- "lateralTuning" > (a) lateralTuning.pid, init('pid') (b) init('torque'). No cars use'indi' anymore (OP092)
--- NISSAN.LEAF > "LatControlAngle" (no "lateralTuning" > no 'pid')
--- HONDA.CIVIC > "LatControlPID" > "PIDController"
--- TOYOTA.PRIUS > "LatControlTorque" > "PIDController"
--- Read "PIDController" + 230521
--- Read Run "test_latcontrolJLL.py" > "latcontrol_pid.py" > "pid.py"
    @parameterized.expand([(HONDA.CIVIC, LatControlPID)])
    def test_saturation(self, car_name, controller):
      output_steer, angle_steers_des, = controller.update(True, CS, VM, params, last_actuators, True, 1, 0, 0)
    class LatControlPID(LatControl):
      self.pid = PIDController((CP.lateralTuning.pid.kpBP, CP.lateralTuning.pid.kpV),
                               (CP.lateralTuning.pid.kiBP, CP.lateralTuning.pid.kiV),
                               k_f=CP.lateralTuning.pid.kf, pos_limit=self.steer_max, neg_limit=-self.steer_max)
      def update(self, active, CS, VM, params, last_actuators, steer_limited, desired_curvature, desired_curvature_rate, llk):
        angle_steers_des_no_offset = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll))
        angle_steers_des = angle_steers_des_no_offset + params.angleOffsetDeg
        error = angle_steers_des - CS.steeringAngleDeg
        output_steer = self.pid.update(error, override=CS.steeringPressed,
                                       feedforward=steer_feedforward, speed=CS.vEgo)
    class PIDController():
      def __init__(self, k_p, k_i, k_f=0., k_d=0., pos_limit=1e308, neg_limit=-1e308, rate=100):
      def update(self, error, error_rate=0.0, speed=0.0, override=False, feedforward=0., freeze_integrator=False):
--- from "test_latcontrolJLL.py"
    @parameterized.expand([(TOYOTA.PRIUS, LatControlAngle)])
    #--- vars(VM) = {'m': 1517.1876220703125, 'j': 2594.3701171875, 'l': 2.700000047683716, 'aF': 1.187999963760376, 'aR': 1.5120000839233398, 'chi': 0.0, 'cF_orig': 118570.515625, 'cR_orig': 147271.0, 'cF': 118570.515625, 'cR': 147271.0, 'sR': 15.739999771118164}
    #--- vars(controller) = {'sat_count_rate': 0.01, 'sat_limit': 0.4000000059604645, 'sat_count': 0.0, 'sat_check_min_speed': 5.0, 'steer_max': 1.0}
    #--- CS = ( vEgo = 30, gas = 0, gasPressed = false, brake = 0,brakePressed = false, steeringAngleDeg = 0,
      steeringTorque = 0, steeringPressed = false, gearShifter = unknown, steeringRateDeg = 0, aEgo = 0,
      vEgoRaw = 0, standstill = false, brakeLightsDEPRECATED = false, leftBlinker = false, rightBlinker = false,
      yawRate = 0, genericToggle = false, doorOpen = false, seatbeltUnlatched = false, canValid = false,
      steeringTorqueEps = 0, clutchPressed = false, steeringRateLimitedDEPRECATED = false, stockAeb = false,
      stockFcw = false, espDisabled = false, leftBlindspot = false, rightBlindspot = false, steerFaultTemporary = false,
      steerFaultPermanent = false, steeringAngleOffsetDeg = 0, brakeHoldActive = false, parkingBrake = false,
      canTimeout = false, fuelGauge = 0, accFaulted = false, charging = false, vEgoCluster = 0, regenBraking = false )
    #--- last_actuators = ( gas = 0, brake = 0, steer = 0, steeringAngleDeg = 0, accel = 0, longControlState = off,
      speed = 0, curvature = 0, steerOutputCan = 0 )
    #--- params = ( valid = false, gyroBias = 0, angleOffsetDeg = 0, angleOffsetAverageDeg = 0, stiffnessFactor = 0,
      steerRatio = 0, sensorValid = false, yawRate = 0, posenetSpeed = 0, posenetValid = false, angleOffsetFastStd = 0,
      angleOffsetAverageStd = 0, stiffnessFactorStd = 0, steerRatioStd = 0, roll = 0 )
--- from "pid.py"
    @parameterized.expand([(HONDA.CIVIC, LatControlPID)])
    #--- self.k_p, self.k_d, self.k_f = 1.100000023841858 0.0 5.999999848427251e-05
    #---   self.p,   self.d,   self.f = -4080.4470148132327 0.0 -200.3128440524755
    #--- control, self.neg_limit, self.pos_limit, self.speed, self.i = -4280.759858865708 -1.0 1.0 30.0 0.0
--- Q3: -4080.4, -200.3 ??? Same as A2:

231113: OK Md 4a: NG UI 2a: PC: "h5toonnx.py" > "modelB6.onnx"
--- Do Md 4a: h5 => onnx:
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ python h5toonnx.py
--- Error
    pass1 convert failed for name: "modelB6/stem_conv/Conv2D"
    op: "Conv2D"
    input: "modelB6/permute/transpose"
    input: "modelB6/stem_conv/Conv2D/ReadVariableOp"
    , ex=Could not infer attribute `explicit_paddings` type from empty iterator
    Error during conversion: Could not infer attribute `explicit_paddings` type from empty iterator
--- Error
    custom_ops = {"TF_OperationName": Conv2D}
      NameError: name 'Conv2D' is not defined
    custom_ops = {"TF_OperationName": "Conv2D"}
      Error during conversion: Could not infer attribute `explicit_paddings` type from empty iterator
--- back to
    custom_ops = {}
--- verify onnx
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ pip list | grep onnx
      onnx                          1.15.0
      tf2onnx                       1.15.1
--- Do Md 4a: downgrade onnx
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ pip install onnx==1.14.1
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ pip list | grep onnx
      onnx                          1.14.1
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ python h5toonnx.py
      ONNX model saved at: /home/jinn/openpilot/aJLL/Model/saved_model/modelB6.onnx
--- OK
--- Run modelB6.onnx
--- Move modelB6.onnx to /home/jinn/openpilot/selfdrive/modeld/models
--- Rename supercombo.onnx, modelB6.onnx to supercombo092.onnx, supercombo.onnx
    (sconsvenv) jinn@Liu:~/openpilot$ ./selfdrive/ui/ui
    (sconsvenv) jinn@Liu:~/openpilot/tools/replay$ ./replayJLL --data_dir dataC "8bfda98c9c9e4291|2020-05-11--03-00-57--61"
--- NG: replayJLL only replays the video (not by modelB6.onnx or supercombo.onnx)
--- UI 2a: How to display PC/C2 UI run by modelB6.onnx ???
          How to display the yellow triangle by supercombo.onnx ???
--- Rename back to supercombo.onnx, modelB6.onnx

231107: NG RT 2: C2A+car: Road Test: "supercombo079.dlc", "B5YJ0421.dlc"
--- 230506 Install LP092 > Error on C2: Screen Black > 1107: Delete LP092
--- Road Test on B5YJ0421.dlc >
--- NG: same as 220426
--- FTP OP079C2 from PC to C2A > Error on C2: Screen Black > Delete it
--- C2A has openpilot079 + supercombo079.dlc + supercomboB5YJ0421.dlc
--- OK: Road Test on supercombo079.dlc

231030: NG Md 4a: PC: "h5toonnx.py" > "modelB6.onnx"
--- Md 4a: "h5toonnx.py" > "modelB6.onnx"
--- Project 2A Step 4 in AI Projects.docx (https://docs.google.com/document/d/1Gr-vrxQy4fDD-Q5iHLiULs8y4OxaJzNBSiTjJtDIPbQ/edit)
--- Run B3TT211124.dlc > B5YJ220421.dlc > modelB6.dlc on Taiwan data
--- Convert modelB6.h5 to modelB6.onnx
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model/saved_model$ pip install tf2onnx
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model/saved_model$ pip install onnxmltools
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ python h5toonnx1.py
      File "h5toonnx1.py", line 7, in <module>
        model = load_model('./saved_model/modelB6.h5')
      File "/home/jinn/sconsvenv/lib/python3.8/site-packages/tensorflow/python/keras/saving/save.py", line 200, in load_model
        return hdf5_format.load_model_from_hdf5(filepath, custom_objects,
      File "/home/jinn/sconsvenv/lib/python3.8/site-packages/tensorflow/python/keras/utils/generic_utils.py", line 556, in class_and_config_for_serialized_keras_object
        raise ValueError(
      ValueError: Unknown optimizer: Custom>Adam. Please ensure this object is passed to the `custom_objects` argument. See https://www.tensorflow.org/guide/keras/save_and_serialize#registering_the_custom_object for details.
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ python -m tf2onnx.convert --saved-model ./saved_model/modelB3d.pb --output modelB3d.onnx --extra_opset 'ai.onnx.contrib:1'
      OSError: SavedModel file does not exist at: ./saved_model/modelB3d.pb/{saved_model.pbtxt|saved_model.pb}
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ python -m tf2onnx.convert --saved-model ./saved_model/modelB3d.pb --input_dim Input "1,12,128,256" \
      --out_node "Identity" --out_node "Identity_1" --out_node "Identity_2" --out_node "Identity_3" --out_node "Identity_4" \
      --out_node "Identity_5" --out_node "Identity_6" --out_node "Identity_7" --out_node "Identity_8" --out_node "Identity_9" \
      --out_node "Identity_10" --output_path ./saved_model/modelB3d.onnx
      convert.py: error: unrecognized arguments: --input_dim Input 1,12,128,256 --out_node Identity --out_node Identity_1 --out_node Identity_2 --out_node Identity_3 --out_node Identity_4 --out_node Identity_5 --out_node Identity_6 --out_node Identity_7 --out_node Identity_8 --out_node Identity_9 --out_node Identity_10 --output_path ./saved_model/modelB3d.onnx
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ python -m tf2onnx.convert --saved-model ./saved_model/modelB3d.pb --output ./saved_model/modelB3d.onnx --opset 10
      OSError: SavedModel file does not exist at: ./saved_model/modelB3d.pb/{saved_model.pbtxt|saved_model.pb}
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$
      python -m tf2onnx.convert --graphdef ./saved_model/modelB3d.pb --output ./saved_model/modelB3d.onnx --inputs input:0 --outputs output0:"Identity", output1:"Identity_1", output2:"Identity_2"
      convert.py: error: unrecognized arguments: output2:Identity_1, output3:Identity_2
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model$ python -m tf2onnx.convert --graphdef ./saved_model/modelB3d.pb --output ./saved_model/modelB3d.onnx --inputs input:0 --outputs output0:"Identity"
        File "/home/jinn/sconsvenv/lib/python3.8/site-packages/tensorflow/python/framework/graph_util_impl.py", line 198, in _assert_nodes_are_present
          assert d in name_to_node, "%s is not in graph" % d
      AssertionError: output0 is not in graph
        https://blog.csdn.net/weixin_43836242/article/details/122730148
--- onnx2py
    /home/jinn/sconsvenv/lib/python3.8/site-packages/onnxconverter_common/onnx2py.py
    (sconsvenv) jinn@Liu:~/openpilot$ pip install onnxruntime
    (sconsvenv) jinn@Liu:~/openpilot$ pip install git+https://github.com/onnx/tensorflow-onnx
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/Model/saved_model$ python -m tf2onnx.convert modelB6.h5 --output modelB6.onnx --opset 11 --verbose
      convert.py: error: unrecognized arguments: modelB6.h5
--- NG: tf2onnx, onnx2py

230922: OK "VMware" PC: openpilot$ "scons -i" then "scons -u -j$(nproc)"
--- scons openpilot on "VMware" in 2 Steps: 1. scons -i, 2. scons -u -j$(nproc)
    (sconsvenv) jinn@ubuntu:~$ pip install scons==4.4.0
    (sconsvenv) jinn@ubuntu:~$ scons --version
       SCons by Steven Knight et al.:
      	 SCons: v4.4.0
    (sconsvenv) jinn@ubuntu:~$ cd openpilot
    (sconsvenv) jinn@ubuntu:~/openpilot$ scons -i
    scons: done building targets.
    (sconsvenv) jinn@ubuntu:~/openpilot$ scons -u -j$(nproc)
--- Error 1: _ui: selfdrive/ui/qt/widgets/cameraview.cc:279: virtual void
    CameraWidget::paintGL(): Assertion `glGetError() == GL_NO_ERROR' failed.
    ModuleNotFoundError: No module named 'casadi'
    scons: *** [selfdrive/controls/lib/lateral_mpc_lib/c_generated_code/Makefile] Error 1
    Missing SConscript 'selfdrive/navd/SConscript'
    File "/home/jinn/openpilot/SConstruct", line 432, in <module>
    cd /home/jinn/openpilot/selfdrive/controls/lib/lateral_mpc_lib && python3 lat_mpc.py
    cd /home/jinn/openpilot/selfdrive/controls/lib/longitudinal_mpc_lib && python3 long_mpc.py
    Traceback (most recent call last):
      File "lat_mpc.py", line 11, in <module>
        from third_party.acados.acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
    SyntaxError: unknown encoding: future_fstrings
    scons: *** [selfdrive/controls/lib/lateral_mpc_lib/c_generated_code/Makefile] Error 1
    scons: *** [selfdrive/controls/lib/longitudinal_mpc_lib/c_generated_code/Makefile] Error 1
    scons: building terminated because of errors.
--- change Line 279, 285 in /selfdrive/ui/qt/widgets/cameraview.cc to
    //assert(glGetError() == GL_NO_ERROR);
--- pip3 install
    (sconsvenv) jinn@ubuntu:~/openpilot$ pip3 install casadi
    (sconsvenv) jinn@ubuntu:~/openpilot$ pip3 install libusb1
--- Errors
--- change Line 422, 423, 432, 439 in /openpilot/SConstruct to
    #SConscript(['selfdrive/controls/lib/lateral_mpc_lib/SConscript'])
    #SConscript(['selfdrive/controls/lib/longitudinal_mpc_lib/SConscript'])
    #SConscript(['selfdrive/navd/SConscript'])
      #SConscript(['tools/cabana/SConscript'])
    (sconsvenv) jinn@ubuntu:~/openpilot$ scons -u -j$(nproc)
    scons: done building targets.
--- OK but No Good (NG) for Step (D) below. (I’ll talk about this in class.)
--- Step (D) Run ./replayJLL
    (sconsvenv) jinn@ubuntu:~/openpilot/tools/replay$ ./replayJLL --data_dir dataC "8bfda98c9c9e4291|2020-05-11--03-00-57--61"
    (sconsvenv) jinn@ubuntu:~/openpilot$ ./selfdrive/ui/ui
    Killing old publisher: uiDebug  ...
    (sconsvenv) jinn@ubuntu:~/openpilot/tools/replay$ ./replayJLL  --demo
    (sconsvenv) jinn@ubuntu:~/openpilot$ ./selfdrive/ui/ui
    Killing old publisher: uiDebug
    selfdrive/ui/qt/onroad.cc: slow frame rate: 13.88 fps  ...

230919: How To Connect a Projector to Ubuntu Laptop?
    Settings => Displays => Resolution => Old: 1920 x 1080 => New: 800 x 600 => Mirror displays => Apply

230913: OK. Md 3: PC: Change Run "tiny1.py" "tiny2.py"
    (sconsvenv) jinn@Liu:~/openpilot/aJLL/tinygrad$ python tiny1.py
      x = Tensor([[3.0]], requires_grad=True)
--- OK
--- Md 3: aJLL/tinygrad/test/test_nn.py

230912: OK CBQ 9: AR 2: XJ 9: PC+C2+body: Change "controlsd.py" "longcontrol.py" "carcontroller.py"
--- XJ 9a: Change "controlsd.py"
    #if CC.longActive:  # XJ 8a 6)
    #if CC.latActive:  # XJ 8a 6)
        #self.desired_curvature, self.desired_curvature_rate = get_lag_adjusted_curvature(self.CP, CS.vEgo,
    if self.odometer < 3.0:  # XJ 6c: forward
      actuators.accel = self.LoC.update()  # XJ 7a 3)
    elif 3.0 <= self.odometer and self.odometer < 3.1:  # XJ 6c: standstill
      actuators.accel = 0.  # XJ 9
    elif 3.1 <= self.odometer and self.odometer < 6.5:  # XJ 6c: backward
      #actuators.accel = -self.LoC.update()  # XJ 9, NG: no backward
      #actuators.accel = -1.5*clip(0.5, -1, 1)
      actuators.accel = -2.5*clip(0.5, -1, 1)
    else:  # XJ 6c: standstill
      actuators.accel = 0.
--- NG 1: -self.LoC no backward
--- NG 2: -1.5*clip very slow
--- OK: -2.5*clip
--- CBQ 9: CB keeps going forward when self.odometer > 6.5 Why ???
--- Run "controlsdXJ7.py"
--- Error 1: openpilot Unavailable Waiting for controls to start
--- Run "controlsdXJ8.py"
--- CBQ 9: same
--- Run "controlsdXJ9.py"
--- XJ 9b: Change "controlsd.py"
    # Update VehicleModel
    #lp = self.sm['liveParameters']
    #x = max(lp.stiffnessFactor, 0.1)
    #sr = max(lp.steerRatio, 0.1)
    #self.VM.update_params(x, sr)
    #long_plan = self.sm['longitudinalPlan']
    # Check which actuators can be enabled
    #standstill = CS.vEgo <= max(self.CP.minSteerSpeed, MIN_LATERAL_CONTROL_SPEED) or CS.standstill
    #CC.latActive = self.active and not standstill
    #CC.longActive = self.enabled and self.CP.openpilotLongitudinalControl
    actuators.accel = 0.
    if self.odometer < 3.0:  # XJ 6c: forward
      actuators.accel = self.LoC.update()  # XJ 7a 3)
    if 3.1 <= self.odometer and self.odometer < 6.5:  # XJ 6c: backward
      actuators.accel = 4.5*clip(-0.5, -1, 1)
--- OK: 4.5*clip(-0.5 backward normal speed, but
--- CBQ 9: same, due to "self.CI.update" ???
--- Read "CI.update"
    self.CC = car.CarControl.new_message()
    def data_sample(self):
      can_strs = messaging.drain_sock_raw(self.can_sock, wait_for_one=True)
      CS = self.CI.update(self.CC, can_strs)
        /selfdrive/car/interfaces.py
        def update(self, c: car.CarControl, can_strings: List[bytes]) -> car.CarState:
--- 230213: Pipe 3: /body/carcontroller.py > CarController > def update(,CC,,):
      return new_actuators, can_sends >
    /body/interface.py > CarInterface > def apply(,c car.CarControl,): return self.CC.update(c,,) >
    controlsd.py > publish_logs() > self.last_actuators, can_sends = self.CI.apply(CC,)
--- XJ 9c: Change "longcontrol.py"
    def update(self):
      self.accel = 1.5*clip(0.5, -1, 1)  # XJ 7a 5)
      #self.accel = 0.  # XJ 9c
      #self.accel = 1.5*clip(-0.5, -1, 1)  # XJ 9c
--- XJ 9c: Change "controlsd.py"
    #if self.odometer > 6.5:  # XJ 9c
    #  actuators.accel = 0.5*clip(-0.5, -1, 1)  # XJ 9c
--- CBQ 9: all same > def update(self) no use > "self.CI.update" ??? > A: No.
--- CBQ 9: due to "self.speed_pid" ???
--- XJ 9d: Change "/body/carcontroller.py"
    self.speed_pid = PIDController(0., k_i=0.23, rate=1/DT_CTRL)
--- Error: CB fell down! Head fell off! Unchange:
    self.speed_pid = PIDController(0.115, k_i=0.23, rate=1/DT_CTRL)
--- > due to "self.speed_pid" ??? > A: No.
--- OK CBQ 9 solved. Due to CB PowerOff (or "self.speed_pid.update") not only C2 PowerOff.
--- XJ 9d: Change "longcontrol.py" "controlsd.py" for CB speed
    self.accel = 2.5*clip(0.5, -1, 1)  # XJ 7a 5)
      actuators.accel = 2.5*clip(-0.5, -1, 1)  # XJ 9d
--- OK
