''' JLL  2023.12.16
for 2401_Auto.PPTX
'''

class PIDController():
  def __init__(self, k_p, k_i, k_f=0., k_d=0., pos_limit=1e308, neg_limit=-1e308, rate=100):
    self.pos_limit = pos_limit
    self.neg_limit = neg_limit
    self.i_unwind_rate = 0.3 / rate
    self.i_rate = 1.0 / rate
  def k_p(self):
    return interp(self.speed, self._k_p[0], self._k_p[1])
  def k_i(self):
    return interp(self.speed, self._k_i[0], self._k_i[1])
  def k_d(self):
    return interp(self.speed, self._k_d[0], self._k_d[1])
  def reset(self):
  def update(self, error, error_rate=0.0, speed=0.0, override=False, feedforward=0., freeze_integrator=False):
    self.speed = speed
    self.p = float(error) * self.k_p
    self.f = feedforward * self.k_f
    self.d = error_rate * self.k_d
    if override:
      self.i -= self.i_unwind_rate * float(np.sign(self.i))
    else:
      i = self.i + error * self.k_i * self.i_rate
      control = self.p + i + self.d + self.f
      if ((error >= 0 and (control <= self.pos_limit or i < 0.0)) or
          (error <= 0 and (control >= self.neg_limit or i > 0.0))) and not freeze_integrator:
        self.i = i
    control = self.p + self.i + self.d + self.f
    self.control = clip(control, self.neg_limit, self.pos_limit)
    return self.control
