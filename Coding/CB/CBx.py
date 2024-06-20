
----- Long History

230405: old /openpilot/launch_openpilotJLL.sh for 230330
#!/bin/bash
# JLL  2023.3.30, 4.6
# for 230330a, 230406
# from /OP091/tools/sim/launch_openpilot.sh
# . ~/DP091venv/bin/activate
# (DP091venv) jinn@Liu:~/openpilot$ bash launch_openpilotJLL.sh

export PASSIVE="0"
export NOBOARD="1"
export SKIP_FW_QUERY="1"
export FINGERPRINT="HONDA CIVIC 2016"

export BLOCK="camerad,loggerd,encoderd,micd"

  # handle pythonpath, from /home/jinn/OP091/launch_chffrplus.sh
  #ln -sfn $(pwd) /home/jinn/DP091venv/bin/python  # Error: no NOT python
  #ln -sfn $(pwd) /home/jinn/DP091venv/bin  # Error: do Not use this
  #export PYTHONPATH="$PWD"
  #DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null )"
  #--- DIR =
  #DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
  #echo "#--- pwd = $( pwd )"
  #echo "#--- BASH_SOURCE[0] = $BASH_SOURCE[0]"
  #echo "#--- DIR = $DIR"
  #dirname "${BASH_SOURCE[0]}"
  #--- pwd = /home/jinn/openpilot/tools/JLL
  #--- BASH_SOURCE = launch_openpilotJLL.sh
  #--- BASH_SOURCE[0] = launch_openpilotJLL.sh[0]
  #--- DIR = /home/jinn/openpilot/tools/JLL
  #cd ../../selfdrive/manager && exec ./manager.py
cd selfdrive/manager && ./manager.py
  #--- ModuleNotFoundError: No module named 'capnp'

230330: NG. Run UI: PC: DP091aL: 230309: "Replay" "USA Data" (./replay --demo)
--- do 230329a: ???
--- move /uiview.py to /openpilot/selfdrive/debug
  (DP091venv) jinn@Liu:~/openpilot/selfdrive/manager$ ./manager.py
--- Error
    File "./manager.py", line 10, in <module>
      import cereal.messaging as messaging
    ModuleNotFoundError: No module named 'cereal'
--- need to use "launch_openpilot.sh"
  /openpilot/tools/sim/launch_openpilot.sh
    DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
      -- it gives the absolute path to the script
         ${} quotes for variables.
         BASH_SOURCE is an array of source file pathnames. This variable is available
           when a script is being sourced or executed by bash.
           It contains nothing in an interactive session.
         $(command) will call the command and return the stdout in a variable
         (DP091venv) jinn@Liu:~$ dirname /home/jinn/openpilot/tools/JLL
           /home/jinn/openpilot/tools
         cd changes the current directory.
         pwd gives the current path.
         /dev/null is a special device (file) that, if written to it, discards and
           if read from, reads null.
    cd ../../selfdrive/manager && exec ./manager.py
  /openpilot/launch_openpilot.sh
    exec ./launch_chffrplus.sh
--- Run Change "launch_openpilotJLL.sh"
  (DP091venv) jinn@Liu:~/openpilot/tools/JLL$ bash launch_openpilotJLL.sh
    ModuleNotFoundError: No module named 'cereal'
--- same Error
  (DP091venv) jinn@Liu:~/openpilot$ ln -s launch_openpilotJLL.sh JLL.sh
  (DP091venv) jinn@Liu:~/openpilot$ ls -l JLL.sh
    lrwxrwxrwx 1 jinn jinn 22 Mar 31 11:10 JLL.sh -> launch_openpilotJLL.sh
  (DP091venv) jinn@Liu:~/openpilot$ rm JLL.sh
  (DP091venv) jinn@Liu:~/openpilot$ which python
    /home/jinn/DP091venv/bin/python
xxxxx add ln -sfn $(pwd) /home/jinn/DP091venv/bin/python  xxxxx Error source
      Linux ln – How to Create a Symbolic Link
        -s soft links
  (DP091venv) jinn@Liu:~/openpilot$ bash launch_openpilotJLL.sh
    ModuleNotFoundError: No module named 'capnp'
--- OK by ln but new Error
  (DP091venv) jinn@Liu:~/openpilot$ ls -l /home/jinn/DP091venv/bin/python
    lrwxrwxrwx 1 jinn jinn 20 Mar 31 11:30 /home/jinn/DP091venv/bin/python -> /home/jinn/openpilot
  (DP091venv) jinn@Liu:~/openpilot$ unlink /home/jinn/DP091venv/bin/python
  (DP091venv) jinn@Liu:~/openpilot$ deactivate
  (DP091venv) jinn@Liu:~/openpilot$ which python
    /home/jinn/.pyenv/shims/python
xxxxx Error: should be /home/jinn/DP091venv/bin/python
  (DP091venv) jinn@Liu:~/openpilot$ which pythonpath
xxxxx Error: the current directory
xxxxx Link (broken) (inode/symlink): /home/jinn/DP091venv/bin/python3
  (DP091venv) jinn@Liu:~/openpilot$ poetry shell
    bash: /home/jinn/DP091venv/bin/poetry: /home/jinn/DP091venv/bin/python: bad interpreter: No such file or directory
  (DP091venv) jinn@Liu:~/openpilot$ pyenv versions
    * system (set by /home/jinn/.pyenv/version)
  (DP091venv) jinn@Liu:~/openpilot$ python --version
  Python 2.7.18
--- Error: /DP091venv/bin/"python3" is bad now
--- redo venv for good "python3"
--- remove /DP091venv
  jinn@Liu:~$ rm -rf DP091venv
--- add this to /openpilot/..
    /OP091/.python-version
--- redo venv
  jinn@Liu:~$ python -m venv ~/DP091venv
--- Error
    /usr/bin/python: No module named venv
jinn@Liu:~$ python --version
Python 2.7.18
jinn@Liu:~$ python3 --version
Python 3.8.10
jinn@Liu:~$ sudo apt-get install python3-pip
jinn@Liu:~$ sudo apt install python3-venv
jinn@Liu:~$ python3 -m venv ~/DP091venv
jinn@Liu:~$ . ~/DP091venv/bin/activate
(DP091venv) jinn@Liu:~$ pyenv versions
* system (set by /home/jinn/.pyenv/version)
  3.8.2
  3.8.2/envs/YPN
  3.8.10
(DP091venv) jinn@Liu:~$ deactivate
jinn@Liu:~/openpilot$ . ~/DP091venv/bin/activate
(DP091venv) jinn@Liu:~/openpilot$ pyenv versions
  system
  3.8.2
  3.8.2/envs/YPN
* 3.8.10 (set by /home/jinn/openpilot/.python-version)
(DP091venv) jinn@Liu:~/openpilot$ poetry shell
Spawning shell within /home/jinn/DP091venv
jinn@Liu:~/openpilot$ . /home/jinn/DP091venv/bin/activate
(DP091venv) jinn@Liu:~/openpilot$ tools/ubuntu_setup.sh

--- redo
jinn@Liu:~$ rm -rf .pyenv
--- close terminal
(DP091venv) jinn@Liu:~/openpilot$ tools/ubuntu_setup.sh
--- Error
  CalledProcessError
  at ~/.pyenv/versions/3.8.10/lib/python3.8/subprocess.py:516 in run
    →  516│             raise CalledProcessError(retcode, process.args,
(DP091venv) jinn@Liu:~/openpilot$ pyenv versions
  system
* 3.8.10 (set by /home/jinn/openpilot/.python-version)
--- close terminal
--- redo
(DP091venv) jinn@Liu:~/openpilot$ poetry shell
Spawning shell within /home/jinn/DP091venv
jinn@Liu:~/openpilot$ . /home/jinn/DP091venv/bin/activate
(DP091venv) jinn@Liu:~/openpilot$ tools/ubuntu_setup.sh
--- Error 1
  CalledProcessError
  at ~/.pyenv/versions/3.8.10/lib/python3.8/subprocess.py:516 in run
    →  516│             raise CalledProcessError(retcode, process.args,

--- cp .pyenv_OP091 to .pyenv
jinn@Liu:~$ cp -avr .pyenv_OP091 .pyenv
jinn@Liu:~$ pyenv versions
pyenv: version `3.8.2' is not installed (set by /home/jinn/.pyenv/version)
jinn@Liu:~$ pyenv install 3.8.2
jinn@Liu:~$ pyenv versions
  system
  3.6.5
  3.6.5/envs/snpe
  3.8.10
* 3.8.2 (set by /home/jinn/.pyenv/version)
  snpe
jinn@Liu:~$ cd openpilot
jinn@Liu:~/openpilot$ . ~/DP091venv/bin/activate
(DP091venv) jinn@Liu:~/openpilot$ poetry shell
(DP091venv) jinn@Liu:~/openpilot$ tools/ubuntu_setup.sh
--- close terminal
--- redo
(DP091venv) jinn@Liu:~$ pyenv versions
* 3.8.2 (set by /home/jinn/.pyenv/version)
(DP091venv) jinn@Liu:~/openpilot$ poetry shell
Spawning shell within /home/jinn/DP091venv
jinn@Liu:~/openpilot$ . /home/jinn/DP091venv/bin/activate
(DP091venv) jinn@Liu:~/openpilot$ tools/ubuntu_setup.sh
  • Installing cryptography (37.0.4): Failed
  CalledProcessError
  at ~/.pyenv/versions/3.8.10/lib/python3.8/subprocess.py:516 in run
   →  516│             raise CalledProcessError(retcode, process.args,
--- Error 1

(DP091venv) jinn@Liu:~$ pyenv versions
* system (set by /home/jinn/.pyenv/version)
  3.8.2
  3.8.10
(DP091venv) jinn@Liu:~$ python --version
Python 3.8.10
--- Change to 3.8.2 in .python-version

jinn@Liu:~$ python3 -m venv ~/DP091venv
(DP091venv) jinn@Liu:~$ pyenv versions
* system (set by /home/jinn/.pyenv/version)
  3.8.2
jinn@Liu:~$ gedit ~/.bashrc
--- Change .bashrc to
# . ~/.pyenvc
source /home/jinn/openpilot/tools/openpilot_env.sh
--- close terminal

(DP091venv) jinn@Liu:~$ pyenv versions
* system (set by /home/jinn/.pyenv/version)
  3.8.2
(DP091venv) jinn@Liu:~/openpilot$ poetry shell
(DP091venv) jinn@Liu:~/openpilot$ tools/ubuntu_setup.sh
  • Installing cryptography (37.0.4): Failed
  CalledProcessError
  at ~/.pyenv/versions/3.8.2/lib/python3.8/subprocess.py:512
    →  512│             raise CalledProcessError(retcode, process.args,
--- Error 2

--- close terminal
(DP091venv) jinn@Liu:~/openpilot$ pyenv versions
  system
* 3.8.2 (set by /home/jinn/openpilot/.python-version)
(DP091venv) jinn@Liu:~/openpilot$ poetry shell
(DP091venv) jinn@Liu:~/openpilot$ tools/ubuntu_setup.sh
--- Error 2

--- close terminal
--- Change to 3.8.2 in .python-version
jinn@Liu:~$ python --version
Python 2.7.18
jinn@Liu:~$ python3 -m venv ~/DP091venv
(DP091venv) jinn@Liu:~/openpilot$ python --version
Python 3.8.10
(DP091venv) jinn@Liu:~/openpilot$ pyenv versions
  system
  3.8.2
* 3.8.10 (set by /home/jinn/openpilot/.python-version)
(DP091venv) jinn@Liu:~/openpilot$ poetry shell
(DP091venv) jinn@Liu:~/openpilot$ tools/ubuntu_setup.sh
--- Error 1
(DP091venv) jinn@Liu:~/openpilot$ pyenv versions
  system
  3.8.2
* 3.8.10 (set by PYENV_VERSION environment variable)
jinn@Liu:~$ python -m venv ~/DP091venv

(DP091venv) jinn@Liu:~/openpilot$ pyenv versions
  system
  3.8.2
* 3.8.10 (set by PYENV_VERSION environment variable)
--- Error 1
jinn@Liu:~$ poetry --version
Poetry (version 1.2.2)
--- Error 3: not Poetry (version 1.3.2)
--- re-install Poetry
jinn@Liu:~$ curl -sSL https://install.python-poetry.org | python3 -
jinn@Liu:~$ poetry --version
Poetry (version 1.4.2)
(DP091venv) jinn@Liu:~/openpilot$ tools/ubuntu_setup.sh
--- Error 1

jinn@Liu:~/openpilot$ python3 -m pip install "poetry==1.3.2"
(DP091venv) jinn@Liu:~/openpilot$ poetry --version
Poetry (version 1.3.2)

jinn@Liu:~$ python --version
Python 2.7.18
jinn@Liu:~$ sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.8 1
--- sudo update-alternatives --install <symlink_origin> <name_of_config> <symlink_destination> <priority>
--- Power Off
jinn@Liu:~$ python --version
Python 3.8.10
jinn@Liu:~$ curl -sSL https://install.python-poetry.org | python3 -
jinn@Liu:~$ gedit ~/.bashrc
--- add the following line to the bottom of bashrc
export PATH=$PATH:$HOME/.local/bin
--- close the terminal and open it again

--- final try
jinn@Liu:~$ python --version
Python 3.8.10
jinn@Liu:~$ python -m venv ~/DP091venv
jinn@Liu:~/openpilot$ git submodule update --init
jinn@Liu:~/openpilot$ . /home/jinn/DP091venv/bin/activate
(DP091venv) jinn@Liu:~/openpilot$ python --version
Python 3.8.10
(DP091venv) jinn@Liu:~/openpilot$ pyenv versions
  system
  3.8.2
* 3.8.10 (set by /home/jinn/openpilot/.python-version)
(DP091venv) jinn@Liu:~/openpilot$ poetry --version
Poetry (version 1.3.2)
(DP091venv) jinn@Liu:~/openpilot$ poetry shell
Virtual environment already activated: /home/jinn/DP091venv
(DP091venv) jinn@Liu:~/openpilot$ tools/ubuntu_setup.sh
(DP091venv) jinn@Liu:~/openpilot$ which pythonpath
--- the current directory
jinn@Liu:~/openpilot$ which python
/home/jinn/.pyenv/shims/python
jinn@Liu:~/openpilot$ which pythonpath
--- the current directory

--- NG.

230324: Change Run UI: PC+C2: "loggerd", "/data/media/0/realdata/", "fcamera.hevc"
--- Change: see 230319 Read ToDo
  Ch1. Change PC/openpilot to PC/OP091 and PC/DP091 to PC/openpilot == PC/DP091L (L: long, S: short)
    bash: /home/jinn/openpilot/tools/openpilot_env.sh: No such file or directory
      jinn@Liu:~$ gedit ~/.bashrc
        # rc: "run commands"
        # open these 2 lines for (sconsvenv) jinn@Liu:~/openpilot$
        #source /home/jinn/openpilot/tools/openpilot_env.sh
        #export PATH=$PATH:$HOME/.local/bin

        # open these 4 lines for (YPN) jinn@Liu:~/YPN$  (snpe) jinn@Liu:~/snpe/dlc$
        export PYENV_ROOT="$HOME/.pyenv"
        export PATH="$PYENV_ROOT/bin:$PATH"
        eval "$(pyenv init --path)"
        eval "$(pyenv virtualenv-init -)"
          -- Shell Scripting for Beginners – How to Write Bash Scripts in Linux
            $ command --options -a arguments
            Linux Command Cheat Sheet
            Linux Commands Cheat Sheet
            Linux Command 命令列指令與基本操作入門教學
            Scripting helps you write a sequence of commands in a file and then execute them.
            The Linux command line is provided by a program called the shell.
            source is a shell built-in command to read and execute the content
              of a file (generally set of commands).
            /tools/openpilot_env.sh: if [ -z "$OPENPILOT_ENV" ]; then ...
              -z flag returns true if the string is empty.
              linux "-z" flag
          -- create OP virtualenv
          jinn@Liu:~$ pyenv virtualenv 3.8.2 OP082
          jinn@Liu:~/openpilot$ pyenv local OP082
          (OP082) jinn@Liu:~/openpilot$ pyenv versions
          -- set up openpilot environment
          jinn@Liu:~$ python -m venv ~/sconsvenv
          jinn@Liu:~$ . ~/sconsvenv/bin/activate
          (sconsvenv) jinn@Liu:~$ pyenv versions
            /home/jinn/sconsvenv/bin
            /home/jinn/sconsvenv/pyvenv.cfg
              home = /home/jinn/.pyenv/versions/3.8.2/bin
            venv (for Python 3) and virtualenv (for Python 2) allows us to manage
              separate package installations for different projects.
            jinn@Liu:~/sconsvenv$ $HOME
              bash: /home/jinn: Is a directory
  Ch2. Run openpilot on /OP091
    jinn@Liu:~$ gedit ~/.bashrc
    -- Change .bashrc Line 121 to
      source /home/jinn/OP091/tools/openpilot_env.sh
    jinn@Liu:~/OP091$ . ~/sconsvenv/bin/activate
    (sconsvenv) jinn@Liu:~/OP091$ pyenv versions
      * 3.8.10 (set by /home/jinn/OP091/.python-version)
    (sconsvenv) jinn@Liu:~/OP091$ python uiviewJLL.py
      ./_ui: error while loading shared libraries: libqmapboxgl.so: cannot open shared o
    -- Error: Need to Change /OP091 to /openpilot
    -- delete
      source /home/jinn/OP091/tools/openpilot_env.sh
      source /home/jinn/openpilot/tools/openpilot_env.sh
  Ch3. Do Step (B) in InstallOP.docx for DP091L
    -- add these to /openpilot
      /home/jinn/OP091/tools/openpilot_env.sh
      /home/jinn/OP091/tools/ubuntu_setup.sh
      /home/jinn/OP091/pyproject.toml
      /home/jinn/OP091/poetry.lock
      /home/jinn/OP091/update_requirements.sh
      /home/jinn/OP091/.python-version
    -- Do Step (B)
      jinn@Liu:~/openpilot$ git submodule update --init
      jinn@Liu:~$ python -m venv ~/DP091venv
        /home/jinn/DP091venv
      jinn@Liu:~$ . ~/DP091venv/bin/activate
      (DP091venv) jinn@Liu:~$ pyenv versions
        * 3.8.2 (set by /home/jinn/.pyenv/version)
      (DP091venv) jinn@Liu:~$  cd openpilot && poetry shell
        Poetry could not find a pyproject.toml file in /home/jinn/openpilot or its parents
      (DP091venv) jinn@Liu:~/openpilot$ poetry shell
        Spawning shell within /home/jinn/DP091venv
        . /home/jinn/DP091venv/bin/activate
        jinn@Liu:~/openpilot$ . /home/jinn/DP091venv/bin/activate
      (DP091venv) jinn@Liu:~/openpilot$ tools/ubuntu_setup.sh
        qt5-default is already the newest version (5.12.8+dfsg-0ubuntu2.1).
        cat: .python-version: No such file or directory
          /home/jinn/openpilot/tools/ubuntu_setup.sh
            $SUDO apt-get install -y --no-install-recommends \
              libavresample-dev \
              qt5-default \
              python-dev
                -- Error: python-dev
          /home/jinn/OP091/.python-version
            3.8.10
      (DP091venv) jinn@Liu:~/openpilot$ tools/ubuntu_setup.sh
        Resolving dependencies... Downloading https://files.pythonhosted.org/packages/f6/f0/10642828a8dfb741e5f3fbaac830550a518a775c7fff6f04a007259b0548/py-1.11.0-py2.py3-noResolving dependencies... (4921.6s)
        Writing lock file
        • Installing cffi (1.15.1): Failed
        CalledProcessError
        Command '['/home/jinn/DP091venv/bin/python', '-m', 'pip', 'install', '--disable-pip-version-check', '--prefix', '/home/jinn/DP091venv', '--no-deps', '/home/jinn/.cache/pypoetry/artifacts/2e/1f/e8/f40e2683115242bc3619999fb378ea22b4357893c0a1cae9d0b5ffe10e/cffi-1.15.1-cp38-cp38-manylinux_2_17_x86_64.manylinux2014_x86_64.whl']' returned non-zero exit status 1.
        at ~/.pyenv/versions/3.8.2/lib/python3.8/subprocess.py:512 in run
             508│             # We don't call process.wait() as .__exit__ does that for us.
          →  512│             raise CalledProcessError(retcode, process.args,
        The following error occurred when trying to handle this error:
        EnvCommandError
        Command ['/home/jinn/DP091venv/bin/python', '-m', 'pip', 'install', '--disable-pip-version-check', '--prefix', '/home/jinn/DP091venv', '--no-deps', '/home/jinn/.cache/pypoetry/artifacts/2e/1f/e8/f40e2683115242bc3619999fb378ea22b4357893c0a1cae9d0b5ffe10e/cffi-1.15.1-cp38-cp38-manylinux_2_17_x86_64.manylinux2014_x86_64.whl'] errored with the following return code 1, and output:
        ERROR: cffi-1.15.1-cp38-cp38-manylinux_2_17_x86_64.manylinux2014_x86_64.whl is not a supported wheel on this platform.
        PoetryException
        Failed to install /home/jinn/.cache/pypoetry/artifacts/c0/8d/d3/bf01f7d4157c6cca7371507be5f94e18d82ae02ed23dabc36aaa8f9d35/MarkupSafe-2.1.2-cp38-cp38-manylinux_2_17_x86_64.manylinux2014_x86_64.whl
        -- Error:
        • Installing cffi (1.15.1): Failed
      -- Again:
      -- Teminal 1
      (DP091venv) jinn@Liu:~/openpilot$ tools/ubuntu_setup.sh
        [notice] A new release of pip available: 22.3.1 -> 23.0.1
        [notice] To update, run: pip install --upgrade pip
        pip packages install...
        Installing dependencies from lock file
        Package operations: 184 installs, 1 update, 0 removals
          • Installing cffi (1.15.1): Installing...
          • Installing charset-normalizer (3.1.0): Installing...
          • Installing markupsafe (2.1.2): Failed
          CalledProcessError
          Command '['/home/jinn/DP091venv/bin/python', '-m', 'pip', 'install', '--disable-pip-version-check', '--prefix', '/home/jinn/DP091venv', '--no-deps', '/home/jinn/.cache/pypoetry/artifacts/c0/8d/d3/bf01f7d4157c6cca7371507be5f94e18d82ae02ed23dabc36aaa8f9d35/MarkupSafe-2.1.2-cp38-cp38-manylinux_2_17_x86_64.manylinux2014_x86_64.whl']' returned non-zero exit status 1.
          at ~/.pyenv/versions/3.8.2/lib/python3.8/subprocess.py:512 in run
          →  51│         raise PoetryException(f"Failed to install {path.as_posix()}") from e
             52│
        • Installing pyparsing (3.0.9)
      (DP091venv) jinn@Liu:~/openpilot$ pyenv versions
        * 3.8.2 (set by PYENV_VERSION environment variable)
      -- pip packages install...
        /home/jinn/openpilot/update_requirements.sh
          echo "pip packages install..."
          poetry install --no-cache --no-root $POETRY_INSTALL_ARGS
      -- Teminal 2
      (DP091venv) jinn@Liu:~/openpilot$ pyenv versions
        * 3.8.10 (set by /home/jinn/openpilot/.python-version)
      -- Teminal 1
      (DP091venv) jinn@Liu:~/openpilot$ deactivate
      -- Teminal 2
      (DP091venv) jinn@Liu:~/openpilot$ tools/ubuntu_setup.sh
      -- Error: same
      -- Error at
          poetry install --no-cache --no-root $POETRY_INSTALL_ARGS
      -- delete
        source /home/jinn/openpilot/tools/openpilot_env.sh
      -- redo
      (DP091venv) jinn@Liu:~/openpilot$ poetry shell
      (DP091venv) jinn@Liu:~/openpilot$ tools/ubuntu_setup.sh
        WARNING: Can not proceed with installation. Kindly remove the
          '/home/jinn/.pyenv' directory first.
      -- rename .pyenv to .pyenv_OP091
      (DP091venv) jinn@Liu:~/openpilot$ tools/ubuntu_setup.sh
        • Installing poetry-dotenv-plugin (0.1.0)
        pre-commit hooks install...
        The virtual environment found in /home/jinn/DP091venv seems to be broken.
        Recreating virtualenv openpilot-s2XI-zPs-py3.8 in /home/jinn/.cache/pypoetry/virtualenvs/openpilot-s2XI-zPs-py3.8
        Command not found: pre-commit
      -- close terminal
      -- redo
      (DP091venv) jinn@Liu:~/openpilot$ poetry shell
        The virtual environment found in /home/jinn/DP091venv seems to be broken.
        Recreating virtualenv openpilot-s2XI-zPs-py3.8 in /home/jinn/.cache/pypoetry/virtualenvs/openpilot-s2XI-zPs-py3.8
        Spawning shell within /home/jinn/.cache/pypoetry/virtualenvs/openpilot-s2XI-zPs-py3.8
        jinn@Liu:~/openpilot$ . /home/jinn/.cache/pypoetry/virtualenvs/openpilot-s2XI-zPs-py3.8/bin/activate
      (openpilot-py3.8) jinn@Liu:~/openpilot$ tools/ubuntu_setup.sh
        ----   OPENPILOT SETUP DONE   ----
        Open a new shell or configure your active shell env by running:
        source ~/.bashrc
      (openpilot-py3.8) jinn@Liu:~/openpilot$ gedit ~/.bashrc
      -- ??? why changed to (openpilot-py3.8)
      -- rename .pyenv to .pyenv_DP091, .pyenv_OP091 to .pyenv
      -- close terminal
      jinn@Liu:~$ cd openpilot
      jinn@Liu:~/openpilot$ . ~/DP091venv/bin/activate
      (DP091venv) jinn@Liu:~/openpilot$

Ch3. Do scons in Step (B)
  -- added files these to /openpilot
/home/jinn/openpilot/SConstruct
/home/jinn/openpilot/common/SConscript

(DP091venv) jinn@Liu:~/openpilot$ scons --version
  SCons by Steven Knight et al.:
    script: v3.1.2
  -- need new version
    (sconsvenv) jinn@Liu:~$ scons --version
    SCons by Steven Knight et al.:
    	SCons: v4.4.0
(DP091venv) jinn@Liu:~$ pip install scons
  bash: /home/jinn/DP091venv/bin/pip: /home/jinn/DP091venv/bin/python:
  bad interpreter: No such file or directory
-- Properties > /home/jinn/DP091venv/bin/python >
  Link (broken) (inode/symlink)
  /home/jinn/.pyenv/versions/3.8.2/bin/python
-- /home/jinn/.pyenv_OP091/versions "no /3.8.2"
-- move /home/jinn/.pyenv_OP091/versions/3.8.2 to /home/jinn/.pyenv/versions
(DP091venv) jinn@Liu:~$ pip install scons
  Successfully installed scons-4.5.2
(DP091venv) jinn@Liu:~$ scons --version
  SCons by Steven Knight et al.:
  	script: v3.1.2.
-- ??? v3.1.2
(DP091venv) jinn@Liu:~$ deactivate
(DP091venv) jinn@Liu:~/openpilot$ scons --version
  SCons by Steven Knight et al.:
  	SCons: v4.5.2.
-- OK
(DP091venv) jinn@Liu:~/openpilot$ pip install numpy
  Successfully installed numpy-1.24.2
(DP091venv) jinn@Liu:~/openpilot$ scons -u -j$(nproc)
-- Error 1
  scons: *** Deprecated tool 'qt' renamed to 'qt3'. Please update your build accordingly. 'qt3' will be removed entirely in a future
-- change qt to qt3 in File "/home/jinn/openpilot/SConstruct", line 318
  scons: warning: Could not detect qt3, using moc executable as a hint (QT3DIR=/usr)
-- Error 2
  scons: *** [common/clutil.o] Source `common/clutil.cc' not found, needed by target `common/clutil.o'.
  scons: *** [common/gpio.o] Source `common/gpio.cc' not found, needed by target `common/gpio.o'.
  ...
-- add all above files
-- Error 3
  sh: 1: cythonize: not found
  scons: *** [common/clock.cpp] Error 127
(DP091venv) jinn@Liu:~/openpilot$ pip install Cython
  Successfully installed Cython-0.29.33
-- Error 4
  /usr/bin/ld: cannot find -ljson11
  clang: error: linker command failed with exit code 1 (use -v to see invocation)
scons: *** [common/params_pyx.so] Error 1
-- NG below
/usr/bin/ld: cannot find -l*
sudo apt-get install lib*-dev

jinn@Liu:/usr/bin$ ls -al
lrwxrwxrwx  1 root root          19 Aug 30  2022  ld -> x86_64-linux-gnu-ld
lrwxrwxrwx  1 root root          23 Aug 30  2022  x86_64-linux-gnu-ld -> x86_64-linux-gnu-ld.bfd
-rwxr-xr-x  1 root root     1740384 Aug 30  2022  x86_64-linux-gnu-ld.bfd

jinn@Liu:/usr/lib/x86_64-linux-gnu/libjson-c.so.4

(DP091venv) jinn@Liu:~/openpilot$ ld -ljson11 --verbose
attempt to open /usr/local/lib/x86_64-linux-gnu/libjson11.so failed
attempt to open /usr/local/lib/x86_64-linux-gnu/libjson11.a failed
attempt to open /lib/x86_64-linux-gnu/libjson11.so failed
attempt to open /lib/x86_64-linux-gnu/libjson11.a failed
attempt to open /usr/lib/x86_64-linux-gnu/libjson11.so failed
attempt to open /usr/lib/x86_64-linux-gnu/libjson11.a failed
attempt to open /usr/lib/x86_64-linux-gnu64/libjson11.so failed
attempt to open /usr/lib/x86_64-linux-gnu64/libjson11.a failed
attempt to open /usr/local/lib64/libjson11.so failed
attempt to open /usr/local/lib64/libjson11.a failed
attempt to open /lib64/libjson11.so failed
attempt to open /lib64/libjson11.a failed
attempt to open /usr/lib64/libjson11.so failed
attempt to open /usr/lib64/libjson11.a failed
attempt to open /usr/local/lib/libjson11.so failed
attempt to open /usr/local/lib/libjson11.a failed
attempt to open /lib/libjson11.so failed
attempt to open /lib/libjson11.a failed
attempt to open /usr/lib/libjson11.so failed
attempt to open /usr/lib/libjson11.a failed
attempt to open /usr/x86_64-linux-gnu/lib64/libjson11.so failed
attempt to open /usr/x86_64-linux-gnu/lib64/libjson11.a failed
attempt to open /usr/x86_64-linux-gnu/lib/libjson11.so failed
attempt to open /usr/x86_64-linux-gnu/lib/libjson11.a failed
ld: cannot find -ljson11

(DP091venv) jinn@Liu:~/openpilot$ sudo ln -s /usr/lib/libz.so.1.2.8 /usr/lib/libjson11.so
-- Error 5
scons: *** [common/params_pyx.so] Implicit dependency `/usr/lib/libjson11.so' not found, needed by target `common/params_pyx.so'.
(DP091venv) jinn@Liu:~/openpilot$ sudo ln -s /usr/lib/libz.so.1.2.8 /usr/lib/x86_64-linux-gnu/libjson11.so
-- Error 4
  /usr/bin/ld: cannot find -ljson11
jinn@Liu:/usr/lib$ sudo apt-get install libjson11-dev
E: Unable to locate package libjson11-dev
jinn@Liu:/usr/lib$ sudo apt-get install libjson11-1
-- Error 4
jinn@Liu:/usr/lib$ sudo apt-get purge libjson11-1
jinn@Liu:/usr/lib$ sudo ln -s /usr/lib/x86_64-linux-gnu/libjson11.so.1 /usr/lib/libjson11.so
-- Error 5
