W3 Code (Run on W3) JLL

240124====================240201
''' for
loss = tf.keras.losses.CosineSimilarity(y_true, y_pred, delta=1.0)
'''
import tensorflow as tf
from numpy.linalg import norm
import numpy
x = tf.constant([[1, 2, 3], [1, 1, 1]]) # x.dtype is tf.int32
tf.math.reduce_euclidean_norm(x)  # returns 4 as dtype is tf.int32
y = tf.constant([[1, 2, 3], [1, 1, 1]], dtype = tf.float32)
tf.math.reduce_euclidean_norm(y)  # returns 4.1231055 which is sqrt(17)
tf.math.reduce_euclidean_norm(y, 0)  # [sqrt(2), sqrt(5), sqrt(10)]
tf.math.reduce_euclidean_norm(y, 1)  # [sqrt(14), sqrt(3)]
tf.math.reduce_euclidean_norm(y, 1, keepdims=True)  # [[sqrt(14)], [sqrt(3)]]
tf.math.reduce_euclidean_norm(y, [0, 1])  # sqrt(17)
y_true = [[0., 1.], [1., 1.]]
y_pred = [[1., 0.], [1., 1.]]
l2_normalize_y_true = tf.keras.backend.l2_normalize(y_true)
l2_norm_y_true0 = tf.norm(y_true[0])
l2_norm_y_true1 = tf.norm(y_true[1])
l2_norm_y_trueAll = tf.norm(y_true)
l2_norm_y_trueAx0 = tf.norm(y_true, axis=0)
l2_norm_y_trueAx1 = tf.norm(y_true, axis=1)
l2_norm_y_trueAA = tf.norm(y_true[:])
print(l2_normalize_y_true)
print(l2_norm_y_true0)
print(l2_norm_y_true1)
print(l2_norm_y_trueAll)
print(l2_norm_y_trueAx0)
print(l2_norm_y_trueAx1)
print(l2_norm_y_trueAA)
# Using 'auto'/'sum_over_batch_size' reduction type.
cosine_loss = tf.keras.losses.CosineSimilarity(axis=1)
loss1 = cosine_loss(y_true, y_pred).numpy()
# l2_norm(y_true) = [[0., 1.], [1./1.414, 1./1.414]]
# l2_norm(y_pred) = [[1., 0.], [1./1.414, 1./1.414]]
# l2_norm(y_true) . l2_norm(y_pred) = [[0., 0.], [0.5, 0.5]]
# loss = mean(sum(l2_norm(y_true). l2_norm(y_pred), axis=1))
#      = -((0. + 0.) +  (0.5 + 0.5)) / 2
print(loss1)
y_true = [0., 1.]
y_pred = [1., 0.]
cosine_loss2 = tf.keras.losses.CosineSimilarity()
loss2 = cosine_loss2(y_true, y_pred).numpy()
print(loss2)
y_true = [1., 1.]
y_pred = [1., 1.]
cosine_loss3 = tf.keras.losses.CosineSimilarity()
loss3 = cosine_loss3(y_true, y_pred).numpy()
print(loss3)
y_true = [1., 1.]
y_pred = [-1., -1.]
cosine_loss4 = tf.keras.losses.CosineSimilarity()
loss4 = cosine_loss3(y_true, y_pred).numpy()
print(loss4)
====================240124
import tensorflow as tf
inputs = tf.random.uniform(shape=(1,3)) # (batch_size, num_features)
#dense_layer = tf.keras.layers.Dense(unit=4, use_bias=False) # Initialize a fully connected layer w/o bias.
  # units:	Positive integer, dimensionality of the output space.
dense_layer = tf.keras.layers.Dense(units=4, use_bias=True)
print("#--- dense_layer(inputs):", dense_layer(inputs))
print("#--- weights:", dense_layer.weights)
print("#--- no. weights:", len(dense_layer.weights))
print("#--- no. trainable_weights:", len(dense_layer.trainable_weights))
print("#--- no. non_trainable_weights:", len(dense_layer.non_trainable_weights))
print("#--- weights[0].shape:", dense_layer.weights[0].shape)
  #--- dense_layer(inputs): tf.Tensor([[-0.68183887  0.15179414 -0.48280513  0.59659487]], shape=(1, 4), dtype=float32)
  #--- weights: [<tf.Variable 'dense/kernel:0' shape=(3, 4) dtype=float32, numpy=
  array([[-0.61847675, -0.7999939 , -0.8977555 ,  0.47125006],
         [-0.47232476, -0.04174364, -0.06141204,  0.35267484],
         [-0.68840134,  0.33275616, -0.59482676,  0.6438323 ]],
        dtype=float32)>, <tf.Variable 'dense/bias:0' shape=(4,) dtype=float32, numpy=array([0., 0., 0., 0.], dtype=float32)>]
  #--- no. weights: 2
  #--- no. trainable_weights: 2
  #--- no. non_trainable_weights: 0
  #--- weights[0].shape: (3, 4)

import numpy as np
import tensorflow as tf
a = tf.constant([1,2.])
b = tf.constant([2,3.])
c = tf.constant([[1.0, 2.0], [3.0, 4.0]])
d = tf.constant([[1.0, 1.0], [0.0, 1.0]])
e = tf.matmul(c, d)
print("c =", c)
c = tf.Tensor(
[[1. 2.]
 [3. 4.]], shape=(2, 2), dtype=float32)
c.shape = (2, 2)
c.numpy() = [[1. 2.]
 [3. 4.]]
c[0] = [1. 2.]
c[1] = [3. 4.]
c[:,0] = [1. 3.]
c[1,:] = [3. 4.]
a[0] = 1.0
a[:] = [1. 2.]
a.numpy() = [1. 2.]
tf.tensordot(a, b, 0) = [[2. 3.]
 [4. 6.]]
tf.tensordot(a, b, 1) = 8.0
231229====================240104
''' for
loss = tf.keras.losses.huber(y_true, y_pred, delta=1.0)
loss = 0.5 * x^2             if |x| <= d = delta
loss = d * |x| - 0.5 * d^2   if |x| >  d
'''
import numpy as np
import tensorflow as tf
#y_true = [[0, 1], [0, 0]]
#y_pred = [[0.6, 0.4], [0.4, 0.6]]
y_true = [1, 0]
y_pred = [0.5, 0.5]
h = tf.keras.losses.Huber()
hl = h(y_true, y_pred).numpy()
print('#--- hl =', hl)
#--- hl = 0.125
231121====================1212
''' for
return set() in match_fw_to_car_fuzzy()
'''
def match_fw_to_car_fuzzy():
  candidate = None
  if candidate is None:
    candidate = 'apple'
  if candidate != 'banana':  # 'apple' or 'banana'
    return set()  # empty set
  else:
    return {candidate}
matches = match_fw_to_car_fuzzy()
if len(matches) == 1:
  print(matches)
else:
  print(len(matches))
  print(matches)
230812====================0812
''' for
f'{gen}/lat_model/lat_expl_ode_fun.c'
f-strings (formatted string literals) provide a much more readable and cleaner
  syntax by using inline variables.
'''
first = "1st"
second = "2nd"
text = "This is the {0} string. This is the {1} string.".format(first, second)
print(text)  # This is the 1st string. This is the 2nd string.
text = "This is the %s string. This is the %s string." % (first, second)
print(text)  # This is the 1st string. This is the 2nd string.
text = f"This is the {first} string. This is the {second} string."
print(text)  # This is the 1st string. This is the 2nd string.
gen = "c_generated_code"
casadi_model = [
  f'{gen}/lat_model/lat_expl_ode_fun.c',
  f'{gen}/lat_model/lat_expl_vde_forw.c',
]
print(casadi_model)
# ['c_generated_code/lat_model/lat_expl_ode_fun.c', 'c_generated_code/lat_model/lat_expl_vde_forw.c']
230725====================0811
''' for
self.initial_x in class CarKalman(KalmanFilter)
'''
import numpy as np
class CarKalman:
  initial_x = np.array([ 1.0, 15.0, 0.0, 0.0,  10.0, 0.0, 0.0, 0.0, 0.0 ])
  def __init__(self):
    self.dim_state = self.initial_x.shape[0]
ck = CarKalman()
print(ck.dim_state)  # = 9
t = (0, 0)
initial_x = np.zeros((0, 0))
print(t, initial_x)  # = []
a = np.zeros((1,3)) # matrix (a list of lists) with 1 rows and 3 columns.
print(a)  # = [[0. 0. 0.]]
print(len(a[0]))  # = 3
230725====================0804
''' for
this->build_live_location(liveLoc);
from https://www.tutorialspoint.com/cplusplus/cpp_this_pointer.htm
Every object in C++ has access to its own address through an important pointer called
	this pointer. The this pointer is an implicit parameter to all its members (data and functions).
	Therefore, inside a member function, this may be used to refer to the invoking object.
'''
#include <iostream>
using namespace std;
class Box {
   public:
      // Constructor definition
      Box(double l = 2.0, double b = 2.0, double h = 2.0) {
         cout <<"Constructor called." << endl;
         length = l;
         breadth = b;
         height = h;
      }
      double Volume() {
         return length * breadth * height;
      }
      int compare(Box box) {
         return this->Volume() > box.Volume();
      }
   private:
      double length;     // Length of a box
      double breadth;    // Breadth of a box
      double height;     // Height of a box
};
int main(void) {
   Box Box1(3.3, 1.2, 1.5);    // Declare box1
   Box Box2(8.5, 6.0, 2.0);    // Declare box2
   if(Box1.compare(Box2)) {
      cout << "Box2 is smaller than Box1" <<endl;
   } else {
      cout << "Box2 is equal to or larger than Box1" <<endl;
   }
   return 0;
}
230719====================0720
''' for
self.state in ACTIVE_STATES
'''
#ACTIVE_STATES = (enabled, softDisabling, overriding)
ACTIVE_STATES = (2, 3, 4)  # see log.capnp
state = 2
print('#--- state in ACTIVE_STATES =', state in ACTIVE_STATES)
#--- state in ACTIVE_STATES = True
230714====================0720
''' for
sec_since_boot, 'logMonoTime'
'''
import time
import datetime  # for now()
import pytz      # for timezone()
TPcurrent_time = datetime.datetime.now(pytz.timezone('Asia/Taipei'))
UKcurrent_time = datetime.datetime.now(pytz.timezone('Europe/London'))
print("#--- Taipei Current Time =", TPcurrent_time)
print("#--- London Current Time =", UKcurrent_time)
named_tuple = time.localtime() # get struct_time
time_string = time.strftime("%m/%d/%Y, %H:%M:%S", named_tuple)
print("#--- London time_string  =", time_string)
seconds = time.time()
local_time = time.ctime(seconds)
print("#--- Local time in UTC   =", local_time)
print("#--- London sec_since_boot =", seconds)  # in sec
# Output: Seconds since epoch = 1672214933.6804628
sec_since_boot = time.time
#print("#--- sec_since_boot =", sec_since_boot)
logMonoTime = int(sec_since_boot() * 1e9)  # in nano sec
print("Printed immediately    :", time.time())
print("logMonoTime            :", logMonoTime)
time.sleep(2.0)
logMonoTime = int(sec_since_boot() * 1e9)  # in nano sec
print("Printed after 2 seconds:", time.time())
print("logMonoTime            :", logMonoTime)
'''
In Python, the time() function returns the number of seconds passed since epoch (the point
	where time begins). For the Unix system, January 1, 1970, 00:00:00 at UTC is epoch.
	Coordinated Universal Time (UTC) is the base point for all other time zones in the world.
	UTC is based on mean solar time at the prime meridian running through Greenwich, UK.
#--- Taipei Current Time = 2023-07-20 14:06:45.424647+08:00
#--- London sec_since_boot = 1689833205.4269104
Printed immediately    : 1689833205.4382417
logMonoTime            : 1689833205438238464
Printed after 2 seconds: 1689833207.4452305
logMonoTime            : 1689833207445224960
'''
230708====================0713
''' for
@property in simple_kalman_impl.pyx
'''
class House:
	def __init__(self, price):
		self._price = price
	@property  # getter
	def price(self):
		return self._price
	@price.setter
	def price(self, new_price):
		if new_price > 0 and isinstance(new_price, float):
			self._price = new_price
		else:
			print("Please enter a valid price")
house = House(50000.0)
print(house.price)
house.price = -45000.0
print(house.price)
#--- 50000.0
#--- Please enter a valid price
#--- 50000.0
230702====================0705, 0114
''' for
STIFFNESS in run_paramsd_on_routeJLL.py
from selfdrive/locationd/models/car_kf.py
for 240113 'hidden_state': slice(6254, -2, None) in get_model_metadata.py
'''
i = 0
def _slice(n):
  global i
  s = slice(i, i + n)  # slice(start, end, step)
  i += n
  return s
STIFFNESS = _slice(1)  # [-]
STEER_RATIO = _slice(1)  # [-]
ANGLE_OFFSET = _slice(1)  # [rad]
ANGLE_OFFSET_FAST = _slice(1)  # [rad]
VELOCITY = _slice(2)  # (x, y) [m/s]
YAW_RATE = _slice(1)  # [rad/s]
STEER_ANGLE = _slice(1)  # [rad]
ROAD_ROLL = _slice(1)  # [rad]
print(STIFFNESS)  # = slice(0, 1, None)
print(VELOCITY)   # = slice(4, 6, None)
print(YAW_RATE)   # = slice(6, 7, None)
a = ("a", "b", "c", "d", "e", "f", "g", "h")
#x = slice(3, 5)  # ('d', 'e') The slice() function returns a slice object in a tuple.
#x = slice(5, -2)  # ('f',)
#x = slice(5, -1)  # ('f', 'g')
#x = slice(-1, -4, -1)  # ('h', 'g', 'f')
#x = slice(-1, -4)  # ()
#x = slice(4, -2, None)  # ('e', 'f')
x = slice(-2, None, None)  # ('g', 'h')
print(a[x])
230702====================0703
''' for
sm['controlsState'].curvature
from cereal/messaging/__init__.py
__getitem__() is a magic method in Python, which when used in a class, allows
  its instances to use the [] (indexer) operators. Say x is an instance of this class,
  then x[i] is roughly equivalent to type(x).__getitem__(x, i).
'''
from typing import List
class SubMaster:
  def __init__(self, services: List[str]):
    for s in services:
    	pass
  def __getitem__(self, s: str):  #__getitem__(self, key) defines the notation self[key]
    pass
sm = SubMaster(['a', 'b'])
print("#--- sm['a'] =", sm['a'])  #--- sm['a'] = None

from typing import List, Optional
def new_message(service: Optional[str] = None):
  if service is not None:
    dat = service
  return dat
class SubMaster:
  def __init__(self, services: List[str]):
    self.data = {}
    for s in services:
      data = new_message(s)
      self.data[s] = data
  def __getitem__(self, s: str):
    return self.data[s]
sm = SubMaster(['a', 'b'])
print("#--- sm['a'] =", sm['a'])  #--- sm['a'] = a
230702====================0702
''' for
@abstractmethod
def apply(self, c: car.CarControl, now_nanos: int)
'''
# Python program showing abstract base class work
from abc import ABC, abstractmethod
class Polygon(ABC):
    @abstractmethod
    def noofsides(self):
        pass
class Triangle(Polygon):
    # overriding abstract method
    def noofsides(self):
        print("I have 3 sides")
class Pentagon(Polygon):
    def noofsides(self):
        print("I have 5 sides")
# Driver code
R = Triangle()
R.noofsides()
R = Pentagon()
R.noofsides()
230702====================0704
''' for @classmethod, @staticmethod, instance method
from Python's Instance, Class, and Static Methods Demystified
'''
class MyClass:
    def method(self):
        return 'instance method called', self
    @classmethod
    def classmethod(the_class):
        return 'class method called', the_class
    @staticmethod
    def staticmethod():
        return 'static method called'
obj = MyClass()
print(obj.method())
print(MyClass.method(obj))
# instance method has access to the object instance (printed as <MyClass instance>) via the self argument
# instance methods can also access the class itself through the self.__class__ attribute. This means instance methods can also modify class state.
#print(obj.__class_)
print(obj.classmethod())
print(obj.staticmethod())
# static methods can neither access the object instance state nor the class state.
print(MyClass.classmethod())
print(MyClass.staticmethod())
#print(MyClass.method())
#
'''
-- class methods as factory functions, a factory is an object for creating other objects
-- Instance methods need a class instance and can access the instance through self.
-- Class methods can’t access the instance (self) but access the class itself via cls.
-- Static methods don’t have access to cls or self. They work like regular functions but belong to the class’s namespace.
-- Static and class methods communicate and enforce developer intent about class design. This can have maintenance benefits.
'''
230702====================0702
''' for
@classmethod
def get_params(cls, candidate: str, fingerprint: Dict[int, Dict[int, int]], car_fw:
'''
from datetime import date
class Person:
    def __init__(self, name, age):
        self.name = name
        self.age = age
    @classmethod
    def fromBirthYear(cls, name, year):
        return cls(name, date.today().year - year)
    @staticmethod
    def isAdult(age):
        return age > 18
person1 = Person('a', 21)
person2 = Person.fromBirthYear('b', 1996)
print(person1.age)
print(person2.age)
print(Person.isAdult(person1.age))
print(Person.isAdult(person2.age))
230619====================0629
''' for
tireStiffnessFront, tireStiffnessRear
'''
STD_CARGO_KG = 136.
MASS = 1326. + STD_CARGO_KG
WHEELBASE = 2.70
CENTER_TO_FRONT = WHEELBASE * 0.4
CENTER_TO_REAR = WHEELBASE - CENTER_TO_FRONT
ROTATIONAL_INERTIA = 2500
TIRE_STIFFNESS_FRONT = 192150
TIRE_STIFFNESS_REAR = 202500
LB_TO_KG = 0.453592
wheelbase = 2.70
steerRatio = 15.74   # unknown end-to-end spec
tire_stiffness_factor = 0.6371   # hand-tune
mass = 3045. * LB_TO_KG + STD_CARGO_KG
centerToFront = wheelbase * 0.44
class CivicParams:
  MASS = 1326. + STD_CARGO_KG
  WHEELBASE = 2.70
  CENTER_TO_FRONT = WHEELBASE * 0.4
  CENTER_TO_REAR = WHEELBASE - CENTER_TO_FRONT
  ROTATIONAL_INERTIA = 2500
  TIRE_STIFFNESS_FRONT = 192150
  TIRE_STIFFNESS_REAR = 202500
def scale_tire_stiffness(mass, wheelbase, center_to_front, tire_stiffness_factor=1.0):
  center_to_rear = wheelbase - center_to_front
  tire_stiffness_front = (CivicParams.TIRE_STIFFNESS_FRONT * tire_stiffness_factor) * mass / CivicParams.MASS * \
                         (center_to_rear / wheelbase) / (CivicParams.CENTER_TO_REAR / CivicParams.WHEELBASE)

  tire_stiffness_rear = (CivicParams.TIRE_STIFFNESS_REAR * tire_stiffness_factor) * mass / CivicParams.MASS * \
                        (center_to_front / wheelbase) / (CivicParams.CENTER_TO_FRONT / CivicParams.WHEELBASE)

  return tire_stiffness_front, tire_stiffness_rear
tireStiffnessFront, tireStiffnessRear = scale_tire_stiffness(mass, wheelbase, centerToFront, tire_stiffness_factor=tire_stiffness_factor)
print("#--- tireStiffnessFront, tireStiffnessRear =", round(tireStiffnessFront, 2), round(tireStiffnessRear, 2))
#--- tireStiffnessFront, tireStiffnessRear = 118570.51 147271.0
230619====================0626
''' for
W = np.asfortranarray
'''
print ("D =", np.diag([1, 2, 3, 4, 5]))
W = np.asfortranarray(np.diag([1, 2, 3, 4, 5]))
print ("W.shape, W =", W.shape, W)
COST_E_DIM = 3
COST_DIM = COST_E_DIM + 2
print ("W[:COST_E_DIM,:COST_E_DIM]) =", W[:COST_E_DIM,:COST_E_DIM])
#ocp.cost.yref = np.zeros((COST_DIM, ))
yref = np.zeros((COST_DIM, ))
print ("yref =", yref)
yref2 = np.zeros((COST_DIM))
print ("yref2 =", yref2)

230619====================0623
''' for
order='F', order='C', np.asfortranarray
'''
import numpy as np
a = np.array([1, 2, 3, 4])
b = np.arange(8,dtype='int8')
c = b.reshape(2,4,order='F')
d = b.reshape(2,4,order='C')
print(a)
print(b)
print(c)
print(d)
arr = np.array([[-1, 2, 0, 4],
                [4, -0.5, 6, 0],
                [2.6, 0, 7, 8],
                [3, -7, 4, 2.0]])
# Slicing array
temp = arr[:2, ::2]
print ("Array with first 2 rows and alternate"
                    "columns(0 and 2):\n", temp)
# Integer array indexing example
temp = arr[[0, 1, 2, 3], [3, 2, 1, 0]]
print ("Elements at indices (0, 3), (1, 2), (2, 1),"
                                    "(3, 0):\n", temp)
# boolean array indexing example
cond = arr > 0 # cond is a boolean array
temp = arr[cond]
print ("\nElements greater than 0:\n", temp)
my_tuple = ([1, 3, 9], [8, 2, 6])
print ("Input  tuple : ", my_tuple)
out_arr = np.asfortranarray(my_tuple, dtype ='int8')
print ("output fortran array from input tuple : ", out_arr)
