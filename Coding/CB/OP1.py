---------- Notes for Reading OP Code ---------- JLL 2020.6.25 - 8.9

Code 1: SConstruct --- \openpilot\SConstruct
SCons User Guide 1.1.0
   https://scons.org/doc/1.1.0/HTML/scons-user/
   https://scons.org/doc/3.1.2/HTML/scons-man.html
   Software construction tool
   SCons guarantees a correct build, builds software as quickly as possible, and
   does as much for you out of the box.
SConstruct
   file is the input file (a Python script) at the "top" of the tree
scons
   that reads to control the build.
SConscript
   is a subsidiary script in the build.
   SConstruct can use SConscripts to include other subsidiary scripts in the build.
scons
   is normally executed in a "top-level" directory containing a SConstruct file.
   SCons can
Export()
   variables from a parent SConscript file to its subsidiary SConscript files,
   which allows you to share common initialized values throughout your build hierarchy.
Import()
   makes parent variables available to subsidiary SConscripts.
import os
   The OS module in python provides functions for interacting with the operating system.
import subprocess
   The Python subprocess module is a tool that allows you to run other programs or commands
     from your Python code. Source code: Lib/subprocess.py.
   An Introduction to Python Subprocess: Basics and Examples
import sys
   The sys module provides information about constants, functions and methods of the Python
   interpreter.
import platform
   The platform module in Python is used to access the underlying platform's data,
     such as, hardware, operating system, and interpreter version information.

   env = Environment()
   Export('env')
   objs = []
   for subdir in ['foo', 'bar']:
      o = SConscript('%s/SConscript' % subdir)
      objs.append(o)
   env.Library('prog', objs)

   Import('env')
   obj = env.Object('foo.c')
   Return('obj')

   % scons -Q
   cc -o bar/bar.o -c bar/bar.c
   cc -o foo/foo.o -c foo/foo.c
   ar rc libprog.a foo/foo.o bar/bar.o
   ranlib libprog.a

   The object files from the subsidiary subdirectories are all correctly archived in the
     desired library.
AddOption('--test', action='store_true', help='build test files')
AddOption('--asan', action='store_true', help='turn on ASAN')
   SCons allows you to define your own command-line options with the AddOption function.
   AddressSanitizer (ASAN) is a fast memory error detector in Clang Compiler,
     an open-source compiler for the C family of programming languages.
   We can use single quotes ' or double quotes " within Python.
arch = subprocess.check_output(["uname", "-m"], encoding='utf8').rstrip()
  check_output is similar to run(), but it only returns the standard output of the command,
    and raises a CalledProcessError exception if the return code is non-zero.
   arch: architecture

   py2.py:
   import sys
   print sys.argv
   py3.py:
   import subprocess
   py2output = subprocess.check_output(['python', 'py2.py', '-i', 'test.txt'])
   print('py2 said:', py2output)
   Running it:
   $ python3 py3.py
   py2 said: b"['py2.py', '-i', 'test.txt']\n"

   UTF-8 (8-bit Unicode Transformation Format) is a variable-width character encoding
   capable of encoding all 1,112,064 valid character code points in Unicode.

   txt = "     banana     "
   x = txt.rstrip()
   print("of all fruits", x, "is my favorite")
   of all fruits     banana is my favorite

   platform.uname() returns a tuple of strings (system, node, release, version, machine,
   processor) identifying the underlying platform.

if platform.system() == "Darwin":
if arch == "aarch64" or arch == "larch64"
   # "aarch64" == EON, "larch64" == C2, TICI == C3 ???
   Darwin is an open-source Unix-like operating system first released by Apple Inc.
   AArch64 is the 64-bit extension of the ARM architecture.
   openpilot/phonelibs/snpe/larch64

webcam = bool(ARGUMENTS.get("use_webcam", 0))
   SCons provides an ARGUMENTS dictionary that stores all of the variable=value
   assignments from the command line. ARGUMENTS.get() allows to specify a default value.

lenv
   language environment (guess)

"LD_LIBRARY_PATH": '/data/data/com.termux/files/usr/lib'
   LD_LIBRARY_PATH
     is Linux specific and is an environment variable pointing to directories where the
     dynamic loader should look for shared libraries.
     PATH is for specifying directories of executable programs.
   Program Library HOWTO: http://www.tldp.org/HOWTO/Program-Library-HOWTO/
     This HOWTO for programmers discusses how to create and use program libraries on Linux
     using the GNU toolset. A ``program library'' is simply a file containing compiled code
     (and data) that is to be incorporated later into a program; program libraries allow
     programs to be more modular, faster to recompile, and easier to update. Program libraries
     can be divided into three types: static libraries, shared libraries, and dynamically
     loaded libraries.
   Shared libraries are libraries that are loaded by programs when they start.
   Dynamically loaded libraries are libraries that are loaded at times other than
     during the startup of a program.
   \comma2\data\data\com.termux\files (I don't see files/usr/lib)

#phonelibs/capnp-cpp/include
"PATH": "#external/bin:" + os.environ['PATH']
   #dir is a directory relative to the top-level SConstruct directory

cereal = [File('#cereal/libcereal.a')]
   File() converts a string into a File instance relative to the target being built.
   env.Program('#other/foo', 'foo.c')
     # An initial '#' or '#/' are equivalent.
     # This builds the program "other/foo" (relative to the top-level
     # SConstruct directory) from "subdir/foo.c":
   env = Environment(CPPPATH='#/include')
     To force scons to look-up a directory relative to the root of the source tree use #
   env = Environment(LIBPATH=['#libA', '#libB'])
     The '#' in the LIBPATH directories specify that they're relative to the top-level directory

cereal = abspath([File('cereal/libcereal_shared.so')])
   The .so file is the shared object library which can be dynamically loaded at the runtime.
   Library files are bigger in size, typically in the range of 2MB to 10MB.
   A .a file is a static (archive) library (in gcc -c), while a .so file is a shared object dynamic
     library (in gcc -o).

Code 2: openpilot --- \openpilot
.github
   In Unix-like operating systems, any file or folder that starts with a dot character
   (for example, /home/user/. config), commonly called a dot file or dotfile, is to be
   treated as hidden that is, the ls commend does not show it unless ls -a is used.
   They are commonly used for storing user preferences or preserving the state of a utility,
   and are frequently created implicitly by using various utilities.
   These are . (current directory) and .. (parent directory).
apk
   The apk files used for the UI.
   ai.comma.plus.offroad.apk: An APK file is an app created for Android, Google's mobile
     operating system.
cereal
   The messaging spec and libs used for all logs
   cereal is both a messaging spec for robotics systems and generic high performance
     interprocess communication (IPC) pub sub messaging with a single publisher and multiple subscribers.
bt_firmware
   Le Pro 3 Elite (X722) Lineage OS /bt_firmware/image/btnv32.bin (boot firmware)
   Sync: On Unix-like operating systems, the sync command synchronizes corresponding file
     data in volatile memory and permanent storage.
d
   smmu-bus-client-d00000.arm,smmu: ARM System Memory Management Unit Architecture

Code 3: run_docker_tests.sh --- \openpilot\run_docker_tests.sh
set -e
   what is set -e in bash
   set -e causes a script immediatly exits when it encounters an error.
SETUP="cd /tmp/openpilot && "
   What is the purpose of && in a shell command?
   $ false || echo "Oops, fail"
     Oops, fail
   $ true || echo "Will not be printed"
   $ true && echo "Things went well"
     Things went well
   $ false && echo "Will not be printed"
   $ false ; echo "This will always run"
     This will always run

   Temporarily change current working directory in bash to run a command.
   $ pwd
     /home/abhijit
   $ (cd /tmp && pwd)  # directory changed in the subshell
     /tmp
   $ pwd               # parent shell's pwd is still the same
     /home/abhijit

BUILD="cd /tmp/openpilot && scons -c && scons -j$(nproc) && "
   what is scons in shell script
   SCons is a computer software build tool that automatically analyzes source code file dependencies
     and operating system adaptation requirements from a software project description and generates final
     binary executables for installation on the target operating system platform.
   The scons utility builds software (or other files) by determining which component
     pieces must be rebuilt and executing the necessary commands to rebuild them.
   Synopsis: scons [ options... ] [ name=val... ] [ targets... ]
   The -c flag removes all files necessary to build the specified target: scons -c .
     to remove all target files, or:
   scons -c build export to remove target files under build and export.
   scons supports building multiple targets in parallel via a -j option that takes,
     as its argument, the number of simultaneous tasks that may be spawned:
   scons -j 4 builds four targets in parallel, for example.
   scons -i, --ignore-errors Ignore all errors from commands executed to rebuild files.

RUN="docker run --shm-size 1G --rm tmppilot /bin/bash -c"
   What does docker run do?
   Docker is a platform that allows you to develop, test, and deploy applications with
   self-sufficient containers that run virtually anywhere. The docker run command creates
   a container from a given image and starts the container using a given command.
   what is --shm-size
   The shm-size parameter allows you to specify the shared memory that a container can use.

Code 4: driving.cc --- \openpilot\selfdrive\modeld\models\driving.cc
Eigen::Matrix<float, MODEL_PATH_DISTANCE, POLYFIT_DEGREE - 1> vander;
   In Eigen, all matrices and vectors are objects of the Matrix template class.
   Matrix<typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
   vander: 200613 Vandermonde matrix.pdf
