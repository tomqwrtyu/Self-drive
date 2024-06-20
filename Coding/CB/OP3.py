---------- 1 Run ui.cc on Ubuntu --------- JLL 2020.7.26
icms@jinnliu:~/openpilot077X/selfdrive/ui$
ToDo: Run paint.cc
XXX means errors encountered.
--- run ui.cc
cp ui.cc uiX.cc
./ui
XXX icms@jinnliu:~/openpilot077X/selfdrive/ui$ ./ui
XXX ./ui: 3: exec: ./_ui: not found
gedit ui
--- cp _ui from comma 2 to here
ls _ui
./ui
XXX ./_ui: 18: ./_ui: Syntax error: "(" unexpected
XXX gedit _ui  # Can't see it.
---------- End 1 Run ui.cc on Ubuntu --------- JLL 2020.7.26

---------- 2 Run view_steering_model.py --------- JLL 2020.8.16-8.23
Note: Got errors. Not yet complete.
https://github.com/commaai/research/blob/master/view_steering_model.py

----- use OP virtualenv (python 3.7.3) in InstallOP.txt to run view_steering_model.py
jinn@Liu:~$ cd openpilot
(OP) jinn@Liu:~/openpilot$ mkdir research1
(OP) jinn@Liu:~/openpilot$ cd research1
--- mv the contents of /research-master0 to /research1
(OP) jinn@Liu:~/openpilot/research1$ pip install keras
(OP) jinn@Liu:~/openpilot/research1$ python view_steering_model.py ./outputs/steering_model/steering_angle55.json jinn
xxxxx Error 13
(OP) jinn@Liu:~/openpilot/research1$ pip install sklearn
(OP) jinn@Liu:~/openpilot/research1$ python view_steering_model.py ./outputs/steering_model/steering_angle55.json jinn
xxxxx Error 14
(OP) jinn@Liu:~/openpilot/research1$ python view_steering_model.py ./outputs/steering_model/steering_angle55.json jinn
xxxxx Error 15
(OP) jinn@Liu:~/openpilot/research1/tools/lib$ gedit logreader.py
(OP) jinn@Liu:~/openpilot/research1$ python view_steering_model.py ./outputs/steering_model/steering_angle55.json jinn
xxxxx Error 16
(OP) jinn@Liu:~/openpilot/research1$ pip install libarchive
(OP) jinn@Liu:~/openpilot/research1$ python view_steering_model.py ./outputs/steering_model/steering_angle55.json jinn
xxxxx Error 17
(OP) jinn@Liu:~/openpilot/research1/tools/lib$ gedit framereader.py
(OP) jinn@Liu:~/openpilot/research1$ python view_steering_model.py ./outputs/steering_model/steering_angle55.json jinn
xxxxx Error 18
--- https://stackoverflow.com/questions/46363871/no-module-named-queue
--- On Python 2, the module is named Queue, on Python 3, it was renamed to queue. 
(OP) jinn@Liu:~/openpilot/research1/tools/lib$ gedit framereader.py
(OP) jinn@Liu:~/openpilot/research1$ python view_steering_model.py ./outputs/steering_model/steering_angle55.json jinn
xxxxx Error 19
--- cPickle comes with the standard libraryÃ¢â‚¬Â¦ in python 2.x. You are on python 3.x it's easier just to use pickle.
(OP) jinn@Liu:~/openpilot/research1/tools/lib$ gedit framereader.py
--- change to #import cPickle as pickle
(OP) jinn@Liu:~/openpilot/research1$ python view_steering_model.py ./outputs/steering_model/steering_angle55.json jinn
xxxxx Error 20
--- The StringIO and cStringIO modules are gone. For Python 3, it is from io import StringIO.
(OP) jinn@Liu:~/openpilot/research1/tools/lib$ gedit framereader.py
(OP) jinn@Liu:~/openpilot/research1$ python view_steering_model.py ./outputs/steering_model/steering_angle55.json jinn
xxxxx Error 21
--- You are trying to run a Python 2 codebase with Python 3. xrange() was renamed to range() in Python 3.
(OP) jinn@Liu:~/openpilot/research1/common/transformations$ gedit orientation.py
(OP) jinn@Liu:~/openpilot/research1$ python view_steering_model.py ./outputs/steering_model/steering_angle55.json jinn
xxxxx Error 22
(OP) jinn@Liu:~/openpilot/research1$ pip install pyqt5
(OP) jinn@Liu:~/openpilot/research1$ python view_steering_model.py ./outputs/steering_model/steering_angle55.json jinn
xxxxx Error 23
--- https://stackoverflow.com/questions/52472063/valueerror-bad-marshal-data-unknown-type-code
--- pip3 install -U setuptools
(OP) jinn@Liu:~/openpilot/research1$ pip3 install -U setuptools
(OP) jinn@Liu:~/openpilot/research1$ python view_steering_model.py ./outputs/steering_model/steering_angle55.json jinn
xxxxx Error 23 (same)
--- https://stackoverflow.com/questions/30861493/how-to-fix-python-valueerrorbad-marshal-data
--- If you get that error, the compiled version of the Python module (the .pyc file) is corrupt probably.
--- .pyc files are created by the Python interpreter when a .py file is imported. They contain the "compiled bytecode".
--- https://stackoverflow.com/questions/785519/how-do-i-remove-all-pyc-files-from-a-project
(OP) jinn@Liu:~/openpilot/research1$ find . -name '*.pyc' -delete
(OP) jinn@Liu:~/openpilot/research1$ python view_steering_model.py ./outputs/steering_model/steering_angle55.json jinn
xxxxx Error 23 (same)
root@Liu:~# find /usr -name '*.pyc' -delete
root@Liu:~# find /home -name '*.pyc' -delete
(OP) jinn@Liu:~/openpilot/research1$ python view_steering_model.py ./outputs/steering_model/steering_angle55.json jinn
xxxxx Error 23 (same)
root@Liu:~# apt-get purge awscli
root@Liu:~# apt-get autoremove
root@Liu:~# apt-get install awscli
(OP) jinn@Liu:~/openpilot/research1$ python view_steering_model.py ./outputs/steering_model/steering_angle55.json jinn
xxxxx Error 23 (same)
--- https://github.com/keras-team/keras/issues/7440
--- MyadaRoshdi commented on 5 Dec 2017
(OP) jinn@Liu:~/openpilot/research1$ pip install --upgrade tensorflow
(OP) jinn@Liu:~/openpilot/research1$ pip install --upgrade keras
xxxxx Error 23 (same)
--- https://stackoverflow.com/questions/63187967/loading-a-pre-trained-model-throws-valueerror-bad-marshal-data-unknown-type-co
--- I am trying to load a pre-trained keras model from .h5 file using the code- self.model = load_model('Facenet/facenet_model.h5').
--- https://stackoverflow.com/questions/63484172/tensorflow-load-data-bad-marshal-data
--- The Model might have been built and saved in Python 2.x and you might be using Python 3.x. Solution is to use the same Python Version using which the Model has been Built and Saved. 
--- Use the same version of Keras (and, may be, tensorflow), on which your Model was Built and Saved.
xxxxx Abandon this approach: Run Python 2.x model on pyenv using Pythone 3.x xxxxx

---------- End ---------
xxxxx Error 23
...
/home/jinn/.pyenv/versions/3.7.3/lib/python3.7/importlib/__init__.py:127: MatplotlibDeprecationWarning: 
The matplotlib.backends.backend_qt4agg backend was deprecated in Matplotlib 3.3 and will be removed two minor releases later.
  return _bootstrap._gcd_import(name[level:], package, level)
Traceback (most recent call last):
  File "view_steering_model.py", line 173, in <module>
    model = model_from_json(json.load(jfile))
...
  File "/home/jinn/.pyenv/versions/OP/lib/python3.7/site-packages/tensorflow/python/keras/utils/generic_utils.py", line 471, in func_load
    code = marshal.loads(raw_code)
ValueError: bad marshal data (unknown type code)

or

  File "/home/jinn/.pyenv/versions/OP/lib/python3.7/site-packages/tensorflow/python/keras/utils/generic_utils.py", line 457, in func_load
    code = marshal.loads(raw_code)
ValueError: bad marshal data (unknown type code)

xxxxx Error 22
...
  File "/home/jinn/.pyenv/versions/OP/lib/python3.7/site-packages/matplotlib/backends/qt_compat.py", line 173, in <module>
    raise ImportError("Failed to import any qt binding")
ImportError: Failed to import any qt binding

xxxxx Error 21
...
  File "view_steering_model.py", line 15, in <module>
    from common.transformations.model import (MODEL_CX, MODEL_CY, MODEL_INPUT_SIZE,
  File "/home/jinn/openpilot/research1/common/transformations/model.py", line 66, in <module>
    get_view_frame_from_road_frame(0, 0, 0, model_height))
  File "/home/jinn/openpilot/research1/common/transformations/camera.py", line 50, in get_view_frame_from_road_frame
    device_from_road = orient.rot_from_euler([roll, pitch, yaw]).dot(np.diag([1, -1, -1]))
  File "/home/jinn/openpilot/research1/common/transformations/orientation.py", line 117, in euler2rot
    return rotations_from_quats(euler2quat(eulers))
  File "/home/jinn/openpilot/research1/common/transformations/orientation.py", line 32, in euler2quat
    for i in xrange(len(quats)):
NameError: name 'xrange' is not defined

xxxxx Error 20
...
  File "/home/jinn/openpilot/research1/tools/lib/framereader.py", line 13, in <module>
    from cStringIO import StringIO
ModuleNotFoundError: No module named 'cStringIO'

xxxxx Error 19
...
  File "/home/jinn/openpilot/research1/tools/lib/framereader.py", line 12, in <module>
    import cPickle as pickle
ModuleNotFoundError: No module named 'cPickle'

xxxxx Error 18
...
  File "view_steering_model.py", line 13, in <module>
    from tools.lib.framereader import FrameReader
  File "/home/jinn/openpilot/research1/tools/lib/framereader.py", line 10, in <module>
    import Queue as queue
ModuleNotFoundError: No module named 'Queue'

xxxxx Error 17
...
  File "view_steering_model.py", line 13, in <module>
    from tools.lib.framereader import FrameReader
  File "/home/jinn/openpilot/research1/tools/lib/framereader.py", line 463
    print "TOOK OVER 10 seconds to fetch", frame_info, get_time, get_time2
                                        ^
SyntaxError: Missing parentheses in call to 'print'. Did you mean print("TOOK OVER 10 seconds to fetch", frame_info, get_time, get_time2)?

xxxxx Error 16
...
  File "view_steering_model.py", line 12, in <module>
    from tools.lib.logreader import LogReader
  File "/home/jinn/openpilot/research1/tools/lib/logreader.py", line 17, in <module>
    import libarchive.public
ModuleNotFoundError: No module named 'libarchive'

xxxxx Error 15
...
  File "view_steering_model.py", line 12, in <module>
    from tools.lib.logreader import LogReader
  File "/home/jinn/openpilot/research1/tools/lib/logreader.py", line 67
    print "LogReader:", log_path
                     ^
SyntaxError: Missing parentheses in call to 'print'. Did you mean print("LogReader:", log_path)?

xxxxx Error 14
(OP) jinn@Liu:~/openpilot/research1$ python view_steering_model.py ./outputs/steering_model/steering_angle55.json jinn
pygame 2.0.0.dev8 (SDL 2.0.12, python 3.7.3)
Hello from the pygame community. https://www.pygame.org/contribute.html
Traceback (most recent call last):
  File "view_steering_model.py", line 172, in <module>
    with open(args.model, 'r') as jfile:
FileNotFoundError: [Errno 2] No such file or directory: './outputs/steering_model/steering_angle55.json'

xxxxx Error 13
(OP) jinn@Liu:~/openpilot/research1$ python view_steering_model.py ./outputs/steering_model/steering_angle55.json jinn
pygame 2.0.0.dev8 (SDL 2.0.12, python 3.7.3)
Hello from the pygame community. https://www.pygame.org/contribute.html
Traceback (most recent call last):
  File "view_steering_model.py", line 20, in <module>
    from sklearn.metrics import r2_score
ModuleNotFoundError: No module named 'sklearn'

--- use /home/jinn/research-master0/view_steering_model.py

--- go to browser and download view_steering_model.py
https://drive.google.com/file/d/1u2vwUjhegiEPjEno_8JHvSxIFwSeNJ52/view?usp=sharing
(OP) jinn@Liu:~/openpilot/research1$ mv ~/Downloads/view_steering_model.py .

xxxxx Error 12
(OP) jinn@Liu:~/openpilot/research1$ python view_steering_model.py ./outputs/steering_model/steering_angle55.json jinn
  File "view_steering_model.py", line 282
    print r2_score(log['steering_angle'],pre)
                 ^
SyntaxError: invalid syntax

xxxxx Error 11: could not do "pyenv install 2.7.12" 2.7.12 too old.
--- download https://github.com/commaai/research
jinn@Liu:~$ git clone https://github.com/commaai/research.git

--- create a virtualenv for the folder research 
jinn@Liu:~$ pyenv versions
jinn@Liu:~$ pyenv virtualenv 3.7.3 OPr
jinn@Liu:~$ cd research
jinn@Liu:~$ pyenv local OPr

--- set up OP in Ubuntu
(OPr) jinn@Liu:~/research$ cp /home/jinn/openpilot/tools/ubuntu_setup.sh .
(OPr) jinn@Liu:~/research$ pip install --upgrade pip
(OPr) jinn@Liu:~/research$ ./ubuntu_setup.sh

--- change jinn to yours and run the code
(OPr) jinn@Liu:~/research$ python view_steering_model.py ./outputs/steering_model/steering_angle55.json jinn
xxxxx got Error 7
--- change line 107 to print(log.keys())
gedit view_steering_model.py
--- rerun the code
xxxxx got Error 8
pip install keras
--- rerun the code
xxxxx got Error 9
--- change Lines 104 and 105 in view_steering_model.py to
user=args.user
path='//home//'+user+'//10//'
log = h5py.File(path+"log.h5", "r+")
cam = h5py.File(path+"camera.h5", "r")
---
--- run the code (without 55)
(OPr) jinn@Liu:~/research$ python view_steering_model.py ./outputs/steering_model/steering_angle.json
xxxxx got Error 10

xxxxx Error 10
(OPr) jinn@Liu:~/research$ python view_steering_model.py ./outputs/steering_model/steering_angle.json
pygame 2.0.0.dev8 (SDL 2.0.12, python 3.7.3)
Hello from the pygame community. https://www.pygame.org/contribute.html
Traceback (most recent call last):
  File "view_steering_model.py", line 94, in <module>
    model = model_from_json(json.load(jfile))
...
  File "/home/jinn/.pyenv/versions/OPr/lib/python3.7/site-packages/tensorflow/python/keras/utils/generic_utils.py", line 325, in class_and_config_for_serialized_keras_object
    for key, item in cls_config.items():
AttributeError: 'list' object has no attribute 'items'

xxxxx Error 9
(OPr) jinn@Liu:~/research$ python view_steering_model.py ./outputs/steering_model/steering_angle55.json jinn
pygame 2.0.0.dev8 (SDL 2.0.12, python 3.7.3)
Hello from the pygame community. https://www.pygame.org/contribute.html
usage: view_steering_model.py [-h] [--dataset DATASET] model
view_steering_model.py: error: unrecognized arguments: jinn

xxxxx Error 8
(OPr) jinn@Liu:~/research$ python view_steering_model.py ./outputs/steering_model/steering_angle55.json jinn
pygame 2.0.0.dev8 (SDL 2.0.12, python 3.7.3)
Hello from the pygame community. https://www.pygame.org/contribute.html
Traceback (most recent call last):
  File "view_steering_model.py", line 8, in <module>
    from keras.models import model_from_json
ModuleNotFoundError: No module named 'keras'

xxxxx Error 7
jinn@Liu:~/research-master0$ python view_steering_model.py ./outputs/steering_model/steering_angle55.json jinn
  File "view_steering_model.py", line 107
    print log.keys()
          ^
SyntaxError: invalid syntax

xxxxx Error 6
(OPrm) jinn@Liu:~/research-master0$  python view_steering_model.py ./outputs/steering_model/steering_angle55.json jinn
  File "view_steering_model.py", line 282
    print r2_score(log['steering_angle'],pre)
                 ^
SyntaxError: invalid syntax

xxxxx Errors: Old Step 3 No Good. 
http://www.nhcue.edu.tw/~jinnliu/teaching/AI17/step3.txt
xxxxx Error 5
jinn@Liu:~/research-master0$ python --version
Python 3.8.2
jinn@Liu:~/research-master0$ pyenv version
3.8.2 (set by /home/jinn/research-master0/.python-version)
jinn@Liu:~/research-master0$ python view_steering_model.py ./outputs/steering_model/steering_angle55.json jinn
  File "view_steering_model.py", line 282
    print r2_score(log['steering_angle'],pre)
          ^
SyntaxError: invalid syntax

xxxxx Error 2
pyenv virtualenv --python=/usr/bin/python2.7 OPrm
jinn@Liu:~$ cd research-master0
jinn@Liu:~/research-master0$ pyenv local OPrm
(OPrm) jinn@Liu:~/research-master0$ python view_steering_model.py ./outputs/steering_model/steering_angle55.json jinn
Traceback (most recent call last):
  File "view_steering_model.py", line 4, in <module>
    import numpy as np
ImportError: No module named numpy

jinn@Liu:~$ pyenv install 2.7.12
jinn@Liu:~$ pyenv virtualenv 2.7.12 OPrm
xxxxx Error 1
ERROR: Could not install packages due to an EnvironmentError: [Errno 13] Permission denied: '/usr/bin/pip'
Consider using the `--user` option or check the permissions.
error: failed to install pip via get-pip.py

---------- Appendix
https://github.com/pyenv/pyenv-virtualenv

How to remove python2.7 from Ubuntu 16.04
https://howtoinstall.co/en/ubuntu/xenial/python2.7?action=remove

--- purge python2.7 package itself, configuration, and data files:
sudo apt-get update
--- if it is a default python
sudo apt purge python2.7-minimal
--- else
sudo apt-get purge python2.7

--- delete all and it's dependencies:
sudo apt-get autoremove --purge python2.7

--- uninstall old virtualenvs
pyenv virtualenvs
pyenv uninstall 2.7.12
--- delete OPrm
pyenv virtualenv-delete OPrm

pip list | grep -i tensorflow
tensorflow             2.2.0
tensorflow-estimator   2.2.0
pip list | grep -i keras

curl -O https://raw.githubusercontent.com/commaai/research/master/view_steering_model.py

---------- End 2 Run view_steering_model.py --------- JLL 2020.8.16-8.23
