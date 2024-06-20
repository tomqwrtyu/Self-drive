# pylint: skip-file
import os
import capnp

CEREAL_PATH = os.path.dirname(os.path.abspath(__file__))
capnp.remove_import_hook()

logJLL = capnp.load(os.path.join(CEREAL_PATH, "logJLL.capnp"))
carJLL = capnp.load(os.path.join(CEREAL_PATH, "carJLL.capnp"))
