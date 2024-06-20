'''
CAN 32  2023.12.1
for "@dataclass" in /car/toyota/values.py  231121
for "DataFrame" in /selfdrive/debug/can_table.py
from https://www.maxlist.xyz/2022/04/30/python-dataclasses/
(sconsvenv) jinn@Liu:~/openpilot/aJLL/Coding$ python zc.py
'''
from dataclasses import dataclass
  # dataclass provides a decorator and functions for automatically adding generated special
  #   methods such as __init__() and __repr__() to user-defined classes.
  # We don't need to write __init__() and self. etc. like
  #   class Product: def __init__(self, name:str, qty:int): self.name = name self.qty = qty
import pandas as pd
@dataclass
class Product:
  name: str
  qty: int
products = []
for i in range(3):
  products.append(Product(name=i, qty=i*2))
df = pd.DataFrame(products)
  # DataFrame: Two-dimensional, size-mutable, potentially heterogeneous tabular data.
  # class pandas.DataFrame(data=None, index=None, columns=None, dtype=None, copy=None)[source]
  #print("#--- products:", products)
  #--- products: [Product(name=0, qty=0), Product(name=1, qty=2), Product(name=2, qty=4)]
print("#--- df:", df)
d = {'col1': ["a", "b"], 'col2': [1, 2], 'col3': [3, 4]}
df1 = pd.DataFrame(data=d)
print("#--- df1:", df1)
'''
CAN 31  2023.8.24
for sys.argv[1:] in uiJLL.py  230822
(sconsvenv) jinn@Liu:~/openpilot$ python zc.py 1 2 3 4
'''
import sys
print("#--- This is the name of the program:", sys.argv[0])
print("#--- Argument List:", str(sys.argv))
print("#--- str(sys.argv[1:]):", str(sys.argv[1:]))
print("#--- sys.argv[1:]:", sys.argv[1:])
  #--- This is the name of the program: zc.py
  #--- Argument List: ['zc.py', '1', '2', '3', '4']
  #--- str(sys.argv[1:]): ['1', '2', '3', '4']
  #--- sys.argv[1:]: ['1', '2', '3', '4']
'''
CAN 30  2023.7.14
from A) https://pypi.org/project/parameterized/
     B) /car/tests/test_models.py
for @parameterized_class  test_models.py  230714
Test parameterization is a type of data-driven testing that allows you to execute the
  same test (test_add()), multiple times using different parameters (test_cases).
'''
import unittest
import math
import numpy as np
from parameterized import parameterized, parameterized_class
# from A) + B)
test_cases = [(1, 2, 3, 2),(5, 5, 10, 25),]
class TestMathClassBase(unittest.TestCase):
  a = 0
  b = 0
  expected_sum = 0
  expected_product = 0
  @classmethod
  def test_add(cls):
    np.testing.assert_equal(cls.a + cls.b, cls.expected_sum)
  @classmethod
  def test_multiply(cls):
    np.testing.assert_equal(cls.a * cls.b, cls.expected_product)
@parameterized_class(('a', 'b', 'expected_sum', 'expected_product'), test_cases)
class TestMathClass(TestMathClassBase):
  pass
if __name__ == "__main__":
  unittest.main()
'''
# from A)
class TestPow(unittest.TestCase):
  @parameterized.expand([(2, 2, 4),(2, 3, 8),(1, 9, 1),(0, 9, 0),])
  def test_pow(self, base, exponent, expected):
     np.testing.assert_equal(math.pow(base, exponent), expected)
# from A)
class TestMathUnitTest(unittest.TestCase):
   @parameterized.expand([("negative", -1.5, -2.0),("integer", 1, 1.0),("large fraction", 1.6, 1),])
   def test_floor(self, name, input, expected):
       np.testing.assert_equal(math.floor(input), expected)
# from A)
@parameterized_class(('a', 'b', 'expected_sum', 'expected_product'), [(1, 2, 3, 2),(5, 5, 10, 25),])
class TestMathClass(unittest.TestCase):
   def test_add(self):
      assert_equal(self.a + self.b, self.expected_sum)
   def test_multiply(self):
      assert_equal(self.a * self.b, self.expected_product)
'''
'''
CAN 29  2023.5.15
from selfdrive/controls/lib/pid.py, common/numpy_fast.py
for 230521 interp(self.speed, self._k_p[0], self._k_p[1])
Run on W3
'''
def interp(x, xp, fp):
  N = len(xp)
  print("N =", N)
  def get_interp(xv):
    hi = 0
    while hi < N and xv > xp[hi]:
      hi += 1
    low = hi - 1
    print("low =", low)
    print("hi =", hi)
    print("xp[low] =", xp[low])
    print("xp[hi] =", xp[hi])
    return fp[-1] if hi == N and xv > xp[low] else (
      fp[0] if hi == 0 else
      (xv - xp[low]) * (fp[hi] - fp[low]) / (xp[hi] - xp[low]) + fp[low])
  return [get_interp(v) for v in x] if hasattr(x, '__iter__') else get_interp(x)
fp = [3.0]
xp = [2.0]
z = interp(0.5, xp, fp)  # hi == 0 => get_interp(0.5) = fp[0] = z
fp = [0, 2.0, 4.0]
xp = [0, 1.0, 2,0]
z = interp(0.5, xp, fp)  # z = (0.5 - 0) * (2.0 - 1.0) / (1.0 - 0) + 0
print(z)  # = 1
z = interp(1.5, xp, fp)  # z = (xv - xp[low]) * (fp[hi] - fp[low]) / (xp[hi] - xp[low]) + fp[low]
print(z)  # = 3
print("fp[-1] = ", fp[-1])
'''
CAN 28  2023.4.5
xattr_example.py
from Python Module xattr has no attribute list
for 230405 os.setxattr, os.getxattr, UPLOAD_ATTR_NAME, UPLOAD_ATTR_VALUE in /loggerd

(sconsvenv) jinn@Liu:~/openpilot$ python zc.py
'''
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from selfdrive.loggerd.uploader import UPLOAD_ATTR_NAME, UPLOAD_ATTR_VALUE
  #UPLOAD_ATTR_NAME = 'user.upload'
  #UPLOAD_ATTR_VALUE = b'1'
  # The b denotes a byte string. Bytes are the actual data. Strings are an abstraction.
  # b'A' == b'\x41' => True
print("DEC HEX ASC")
for b in bytearray(b'1A'):
  print(b, hex(b), chr(b))
  # DEC HEX ASC
  # 49 0x31 1
res = b'A' == b'\x41'
print("res = ", res)
print("{}".format(os.__file__))
  # /home/jinn/.pyenv/versions/3.8.2/lib/python3.8/os.py
  # '/usr/local/lib/python3.7/site-packages/xattr/__init__.py'
def show_me_the_meta(file_name):
    """ List Key Meta Names for File. """
    print("Showing Initial Names & Values.")
    attrz = os.listxattr(file_name)
    result = ("A. Info Showcased Init: {}".format(attrz))
    print("{}".format(result))
    return result
def update_the_meta(file_name):
    """ Update Key Meta Names for File. """
    os.setxattr(file_name, UPLOAD_ATTR_NAME, UPLOAD_ATTR_VALUE)
    get_the_meta_values(file_name)
    return
def get_the_meta_values(file_name):
    print("B. List Meta for: {}".format(file_name))
    attrz = os.listxattr(file_name)
    print("")

    for i in reversed(attrz):
        abc = os.getxattr(file_name, i)
        result = ("{} : {}".format(i, abc))
        print("   {}".format(result))
    print("")
    return
def remove_the_meta(file_name):
    os.removexattr(file_name, UPLOAD_ATTR_NAME)
    attrz = os.listxattr(file_name)
    result = ("C. Info Removed Meta: {}".format(attrz))
    print("{}".format(result))
    return result
if __name__ == '__main__':
    show_me_the_meta('zc.py')
    update_the_meta('zc.py')
    #remove_the_meta('zc.py')
'''
OK: w/o from selfdrive.loggerd.uploader import UPLOAD_ATTR_NAME, UPLOAD_ATTR_VALUE
  /home/jinn/.pyenv/versions/3.8.2/lib/python3.8/os.py
  Showing Initial Names & Values.
  A. Info Showcased Init: []
  B. List Meta for: zc.py
     user.upload : b'1'
       Where are file extended attributes saved?
         These attributes are stored totally separate from the file itself,
           in a special section of the disk volume. So adding, removing or modifying
           extended attributes does not alter the file in any way.
         An inode is a data structure that stores metadata about eachfile and directory,
           which is identified by an integer known as “inode number”.
           “Inode” is the abbreviation for “index node”.
         The name of a file is kept in a directory, paired with its inode number.
         The file’s actual attributes and pointers to disk blocks are kept elsewhere,
           in the inode for the file. Attributes are stored with the inode, not the name.
         Tutorial: Unix/Linux File System – directories, inodes, hard links
           (sconsvenv) jinn@Liu:~/openpilot$ ls -l -i zc.py
             8922566 -rwxrwxrwx 1 jinn jinn 3823 Apr  6 12:30 zc.py
           (sconsvenv) jinn@Liu:~/openpilot$ lsattr zc.py
             --------------e----- zc.py
           (sconsvenv) jinn@Liu:~/openpilot$ ls -li /dev/mem /dev/sda /dev/tty1
             ls: cannot access '/dev/sda': No such file or directory
             4 crw-r----- 1 root kmem 1, 1 Apr  4 12:52 /dev/mem
             21 crw--w---- 1 root tty  4, 1 Apr  4 12:52 /dev/tty1
Error: w from selfdrive.loggerd.uploader import UPLOAD_ATTR_NAME, UPLOAD_ATTR_VALUE
    File "/home/jinn/openpilot/selfdrive/loggerd/uploader.py", line 15, in <module>
      from common.api import Api
    File "/home/jinn/openpilot/common/api/__init__.py", line 9, in <module>
      API_HOST = os.getenv('API_HOST', 'https://api.commadotai.com') if not Params().get_b
    File "common/params_pyx.pyx", line 52, in common.params_pyx.Params.check_key
      raise UnknownKeyName(key)
    common.params_pyx.UnknownKeyName: b'dp_api_custom'
'''
'''
CAN 27   2023.3.21
for 230318: "os.system", "LD_LIBRARY_PATH"

(sconsvenv) jinn@Liu:~/openpilot$ env | grep '^LD_LIBRARY_PATH'
--- nothing
(sconsvenv) jinn@Liu:~/openpilot$ python zc.py
  aCB			      opendbc ...

FTP zc.py to C2
root@localhost:/data/openpilot$ env | grep '^LD_LIBRARY_PATH'
  LD_LIBRARY_PATH=/data/phonelibs:/data/data/com.termux/files/usr/lib
root@localhost:/data/openpilot$ python zc.py
  CANNOT LINK EXECUTABLE: library "libiconv.so" not found
  page record for 0x7f98133010 was not found (block_size=32)
'''
import os
os.system("LD_LIBRARY_PATH= ls")
'''
CAN 26a   2023.3.19, 7.18
for CAN 26, /cerealJLL/SConstruct  230714
/openpilot/zc2.py is run by /openpilot/zc1.py in CAN 26
'''
import platform
import subprocess
#p = subprocess.Popen(["python", "--help"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
  # The subprocess.Popen() creates a new process and execute the command list ["python", "--help"].
#output, errors = p.communicate()
  #print("#--- zc2.py output =\n", output)
arch = subprocess.check_output(["uname", "-m"], encoding='utf8').rstrip()
  # check_output is similar to run(), but it only returns the standard output of the command ["uname", "-m"],
  #   and raises a CalledProcessError exception if the return code is non-zero.
  # Uname stands for UNIX name. It is a utility to check the system information of your Linux computer.
print("#--- arch =", arch)
  #--- arch = x86_64
print("#--- platform.system() =", platform.system())
  #--- platform.system() = Linux
'''
CAN 26   2023.3.19
for tools/replay/can_replay.py
    230319: UI: PC+C2: "loggerd": JLL #subprocess.call
from An Introduction to Python Subprocess: Basics and Examples

(sconsvenv) jinn@Liu:~/openpilot$ python zc1.py
zc1.py runs zc2.py in CAN 26a
'''
import os
import subprocess
from common.basedir import BASEDIR
  # Example 1: Running shell commands:
#result = subprocess.run(["dir"], shell=True, capture_output=True, text=True)
  # subprocess.run() runs a subprocess with the command "dir".
  # ["dir"] == jinn@Liu:~/openpilot/aJLL/Coding/py$ dir
  #print("#--- zc.py output 1 =\n", result.stdout)
  # Example 2: Running another Python script zc2.py
result = subprocess.run(["python", "zc2.py"], capture_output=True, text=True)
print("#--- zc.py running zc2.py result:\n", result.stdout)
  # Example 3: Running the executable file selfdrive/loggerd/bootlog
subprocess.call("./bootlog", cwd=os.path.join(BASEDIR, "selfdrive/loggerd"))
  # call() runs "./bootlog" (a process) in selfdrive/loggerd/ and waits for it to complete.
  # Output: selfdrive/loggerd/bootlog.cc: bootlog to /home/jinn/.comma/media/0/realdata/boot/2023-03-19--19-19-44
'''
CAN 25   2023.3.2
for tools/replay/can_replay.py
    230302: UI: PC+C2: "Joystick Mode", "testJoystick"
jinn@Liu:~/openpilot/aJLL/Coding/py$ python zc.py
'''
import os
IGN_ON = int(os.getenv("ON", "0"))
IGN_OFF = int(os.getenv("OFF", "0"))
ENABLE_IGN = IGN_ON > 0 and IGN_OFF > 0
if ENABLE_IGN:
  print(f"Cycling ignition: on for {IGN_ON}s, off for {IGN_OFF}s")
'''
CAN 24   2023.2.15
for class LatControl(ABC)
jinn@Liu:~/openpilot/acan$ python zc.py
'''
from abc import ABC, abstractmethod
class Calculation(ABC):
    @abstractmethod
    def add(self):
        pass
    @abstractmethod
    def subtract(self):
        pass
    #@abstractmethod
    def multiply(self):
        print('Method of an Abstract Class can be called without error',
              'because abstract method decorator is not used')
class Calculator(Calculation):
    def __init__(self, a, b):
        self.a = a
        self.b = b
    def add(self):
        print(self.a + self.b)
    def subtract(self):
        print(self.a - self.b)

take = Calculator(10, 5)
take.add() # 15
take.subtract() # 5
take.multiply() # Method of an Abstract ...
print(issubclass(Calculator, Calculation)) # True
print(issubclass(Calculator, ABC)) # True

'''
CAN 23   2022.12.15
for /home/jinn/YPN/tinygrad
'''
/tinygrad/jl1.py
/tinygrad/openpilot/compile.py
  #!/usr/bin/env python3
    The #! characters are used to denote the beginning of a shebang.
    A shebang refers to a set of unique characters included at the beginning of a script file.
    A Shebang indicates which Bash or Python interpreter to use to interpret an executable file.
  /usr/bin/env python is explicit, and means "use the environment's default python"
  #!/usr/bin/env python3 indicates this module can be run as a script.
    Scripts are runnable Python programs that do something when executed.
    Modules are Python files that are intended to be imported into scripts and
      other modules so that their defined members—like classes and functions—can be used.

/tinygrad/tensor.py
  @classmethod
  def zeros(cls, *shape, **kwargs): return cls(np.zeros(shape, dtype=np.float32), **kwargs)
  How to Use *args and **kwargs in Python

  self.grad : Optional[Tensor] = None
  When To Use Colon (:) in Python?

import numpy as np
arr = np.array([1, 2, 3, 4, 5])
a = np.where(arr > 3, arr, 0)
print(a)
input_bytes = b"\x00\x01"  # A byte string is automatically a list of numbers.
output = list(input_bytes)
print(output)
a = 4
lis1 = [1, 2, 3, 4, 5]
# No argument case
print ("Byte conversion with no arguments : " + str(bytes()))
# conversion to bytes
print ("The integer conversion results in : "  + str(bytes(a)))
print ("The iterable conversion results in : "  + str(bytes(lis1)))

'''
CAN 22   2022.4.20
for /home/jinn/YPN/Leon/parserB5.py
run on https://www.w3schools.com/python/numpy/trypython.asp?filename=demo_numpy_array_copy
'''
import numpy as np

def softmax(x):
  x = np.copy(x)  # it does not modify x in softmax(x).
  axis = 1 if len(x.shape) > 1 else 0
  x -= np.max(x, axis=axis, keepdims=True)
  if x.dtype == np.float32 or x.dtype == np.float64:
    #np.exp(x, out=x)
    np.exp(x)
  else:
    x = np.exp(x)
  x /= np.sum(x, axis=axis, keepdims=True)
  return x

x = np.array([1, 2, 3, 4, 5])
print(softmax(x))
print(x)
print(x.shape)
print(len(x.shape))
print(x.dtype)
axis = 1 if len(x.shape) > 1 else 0
x -= np.max(x, axis=axis, keepdims=True)
print(x)

x = np.array([[1, 2, 3, 4, 5],
[2, 4, 6, 8, 10],
[3, 6, 9, 12, 15]])
print(x.shape)
print(len(x.shape))
print(x)
print(softmax(x))
axis = 1 if len(x.shape) > 1 else 0
x -= np.max(x, axis=axis, keepdims=True)
print(x)

x = np.array([1, 2])
print(softmax(x))
axis = 1 if len(x.shape) > 1 else 0
print(x)
x -= np.max(x, axis=axis, keepdims=True)
print(x)
x = np.exp(x)
print(x)
x /= np.sum(x, axis=axis, keepdims=True)
print(x)

x = np.array([1, 2])
print(x)
x = np.exp(x)
print(x)
x /= np.sum(x)
print(x)

---
[-1 0] = [1, 2] - [2, 2]
[e^-1, e^0]/(e^-1 + e^0)
[e^1 e^2]/(e^1 + e^2)
e^-1/(e^-1 + e^0) =? e^1/(e^1 + e^2) OK
e^0/(e^-1 + e^0) =? e^2/(e^1 + e^2) OK
---

'''
CAN 21   2022.1.26
Notes: datagenB6.py, train_modelB6.py
'''
datagenB6.py
  Correct modelB5 errors: /127.5: redo and use yuv.h5; /100: move to custom_loss()
    Use yuv.h5 (see datagenB6a Line 111)
  test foo = ['a', 'b', 'c', 'd', 'e'],
  ranIdx = list(range(len(foo)))
train_modelB6.py
  def custom_loss(y_true, y_pred):
    2021 Paper: Multi-Loss Weighting with Coefficient of Variations
      Coefficient of Variations; Semantic Segmentation; CityScapes dataset
