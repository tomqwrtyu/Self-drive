# must be build with scons
import os
import capnp

from typing import Optional, List, Union
from collections import deque

from cerealJLL import logJLL
from cerealJLL.servicesJLL import service_list
  #from common.realtime import sec_since_boot
import time
sec_since_boot = time.time

def new_message(service: Optional[str] = None, size: Optional[int] = None) -> capnp.lib.capnp._DynamicStructBuilder:
  dat = logJLL.Event.new_message()
  dat.logMonoTime = int(sec_since_boot() * 1e9)
  dat.valid = True
  if service is not None:
    if size is None:
      dat.init(service)
    else:
      dat.init(service, size)
  return dat

class SubMaster:
  def __init__(self, services: List[str], poll: Optional[List[str]] = None,
               ignore_alive: Optional[List[str]] = None, ignore_avg_freq: Optional[List[str]] = None,
               addr: str = "127.0.0.1"):
    self.data = {}
    self.valid = {}
    self.logMonoTime = {}

    for s in services:
      try:
        data = new_message(s)
      except capnp.lib.capnp.KjException:  # pylint: disable=c-extension-no-member
        data = new_message(s, 0) # lists

      self.data[s] = getattr(data, s)
      self.logMonoTime[s] = 0
      self.valid[s] = data.valid

  def __getitem__(self, s: str) -> capnp.lib.capnp._DynamicStructReader:
    return self.data[s]
