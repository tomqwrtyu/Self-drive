CAN   JLL, 2021.4.16-4.29
Copy and Paste Code Snippets to zc.py
--------------------------------------------------
CCCCCCCCCCCCCCCCCCCCCCCC Code Snippets
CAN 3    2021.4.16, 4.29
https://github.com/eerimoq/cantools
https://python-can.readthedocs.io/en/2.1.0/interfaces/socketcan.html
http://cannibalbusproject.blogspot.com/2017/12/hello-socketcan-world.html
https://medium.com/@yogeshojha/car-hacking-101-practical-guide-to-exploiting-can-bus-using-instrument-cluster-simulator-part-i-cd88d3eb4a53
https://medium.com/@yogeshojha/car-hacking-101-practical-guide-to-exploiting-can-bus-using-instrument-cluster-simulator-part-ee998570758
https://www.sans.org/reading-room/whitepapers/awareness/paper/37825
https://www.varonis.com/blog/how-to-use-wireshark/
https://www.wireshark.org/docs/wsug_html/

----- CAN data by Ubuntu commands
----- install SocketCAN: cantools, can-utils
jinn@Liu:~/can$ python3 -m pip install cantools
jinn@Liu:~/can$ sudo apt install can-utils
jinn@Liu:~/can$ pip3 install matplotlib
jinn@Liu:~/can$ sudo modprobe vcan
jinn@Liu:~/can$ sudo ip link add dev vcan0 type vcan
jinn@Liu:~/can$ sudo ip link set vcan0 up
jinn@Liu:~/can$ lsmod | grep can
jinn@Liu:~/can$ ifconfig vcan0
jinn@Liu:~/can$ ip -details -statistics link show vcan0
jinn@Liu:~/can$ cansend --help
jinn@Liu:~/can$ candump -help
jinn@Liu:~/can$ cantools monitor motohawk.dbc
jinn@Liu:~/can$ cantools dump motohawk.dbc

--- Terminal 1
jinn@Liu:~/can$ cansend vcan0 01a#11223344AABBCCDD
jinn@Liu:~/can$ cansend vcan0 123#DEADBEEF
jinn@Liu:~/can$ sudo cangen vcan0
- press ctrl+c to exit
--- Terminal 2
jinn@Liu:~/can$ candump vcan0
- press ctrl+c to exit
jinn@Liu:~/can$ candump -L vcan0
jinn@Liu:~/can$ candump -cae vcan0,0:0,#FFFFFFFF

--- run CAN Code 5 and save messages in log file
--- Terminal 1
jinn@Liu:~/can$ candump -l vcan0
- type ctrl+c to exit
jinn@Liu:~/can$ candump vcan0
--- Terminal 2
jinn@Liu:~/can$ python zc.py
- see log file
jinn@Liu:~/can$ cat candump-2021-04-20_132329.log
jinn@Liu:~/can$ cat candump-2021-04-20_132329.log | python3 -m cantools plot motohawk.dbc

--- Terminal 1
jinn@Liu:~/can$ sudo cangen vcan0
--- Terminal 2
jinn@Liu:~/can$ candump vcan0 | python3 -m cantools decode motohawk.dbc

----- install Wireshark
--- Wireshark displays CAN oupt and filters messages based on message ID
jinn@Liu:~/can$ sudo apt install wireshark-qt
- press esc to exit configuration mode

--- Terminal 1
jinn@Liu:~/can$ sudo cangen vcan0
--- Terminal 2
jinn@Liu:~/can$ wireshark
-- Error: void DBusMenuExporterPrivate::addAction(QAction*, int): Already tracking action "" under id 195
jinn@Liu:~/can$ sudo chmod 0 /usr/lib/x86_64-linux-gnu/qt5/plugins/platformthemes/libappmenu-qt5.so
-- OK
-- Error: wireshark not capturing vcan0 packets
jinn@Liu:~/can$ sudo dpkg-reconfigure wireshark-common
-- Select "yes"
jinn@Liu:~/can$ sudo usermod -a -G wireshark jinn
jinn@Liu:~/can$ reboot
-- OK

CCCCCCCCCCCCCCCCCCCCCCCC
'''
CAN 2
openpilot/selfdrive/car/fw_versions.py

--- 0xHH is literally, that is, the hexa literal number,
--- \xHH is a character representation used as a string.
$$$$$  b0, b1, b2 =  26 136 1 228
'''
b = b'\x1a\x88\x01' # b = TOYOTA_VERSION_REQUEST, Python bytes object
print("$$$$$  str(b) = ", str(b))
b0 = b[0]
b1 = b[1]
b2 = b[2]
b3 = 0xE4
print("$$$$$  b0, b1, b2 = ", b0, b1, b2, b3)
print("$$$$$  str(b0) = ", str(b0))
print("$$$$$  b'\x1a' = ", b'\x1a')
print("$$$$$  str(b'\x1a') = ", str(b'\x1a'))
print("$$$$$  str('0xE4') = ", str('0xE4'))
print("$$$$$  str(0xE4) = ", str(0xE4))
print("$$$$$  0xE4 = ", 0xE4)

CCCCCCCCCCCCCCCCCCCCCCCC
'''
CAN 1
openpilot/selfdrive/test/test_models.py
'''
# https://docs.python.org/3/library/unittest.html
class TestStringMethods(unittest.TestCase):
    def test_upper(self):
        self.assertEqual('foo'.upper(), 'FOO')
    def test_isupper(self):
        self.assertTrue('FOO'.isupper())
        self.assertFalse('Foo'.isupper())
    def test_split(self):
        s = 'hello world,'
        self.assertEqual(s.split(), ['hello', 'world'])
        # check that s.split fails when the separator is not a string
        with self.assertRaises(TypeError):
            s.split(2)
if __name__ == '__main__':
    unittest.main()
# Meaning of unittest.main() in Python unittest module
# Understanding if __name__ == “__main__” in Python
# A Python module is a file that has a .py extension.
# Naming with Underscores in Python

CCCCCCCCCCCCCCCCCCCCCCCC
CARLA
Failed. https://github.com/commaai/openpilot/wiki/CARLA
--- Update Nvidia Drivers
(OP) jinn@Liu:~/openpilot/tools/sim$ sudo apt-get purge nvidia*
(OP) jinn@Liu:~/openpilot/tools/sim$ sudo reboot
(OP) jinn@Liu:~/openpilot/tools/sim$ sudo add-apt-repository ppa:graphics-drivers/ppa
(OP) jinn@Liu:~/openpilot/tools/sim$ ubuntu-drivers devices
(OP) jinn@Liu:~/openpilot/tools/sim$ sudo apt install nvidia-465
(OP) jinn@Liu:~/openpilot/tools/sim$ sudo reboot
(OP) jinn@Liu:~/openpilot/tools/sim$ ./bridge.py
--- Error: WARNING: NO CARLA
--- Install docker.io
(OP) jinn@Liu:~/openpilot/tools/sim$ sudo curl -sSL https://get.docker.com/ | sh
(OP) jinn@Liu:~/openpilot/tools/sim$ sudo apt install docker.io
(OP) jinn@Liu:~/openpilot/selfdrive$ sudo docker run hello-world
--- OK: Hello from Docker!
(OP) jinn@Liu:~/openpilot/tools/sim$ ./bridge.py
--- Same Error: WARNING: NO CARLA
--- Re-download sim: https://github.com/commaai/openpilot/tree/master/tools/sim
(OP) jinn@Liu:~/openpilot/tools/sim$ sudo chmod -R a+rwx /home/jinn/openpilot/tools/sim
(OP) jinn@Liu:~/openpilot/tools/sim$ sudo INSTALL=1 ./start_carla.sh
(OP) jinn@Liu:~/openpilot/tools/sim$ sudo ./start_openpilot_docker.sh
--- Error: unknown flag: --gpus
(OP) jinn@Liu:~/openpilot/tools/sim$ sudo apt-get install apt-transport-https ca-certificates curl software-properties-common
(OP) jinn@Liu:~/openpilot/tools/sim$ curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
(OP) jinn@Liu:~/openpilot/tools/sim$ sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu xenial stable"
(OP) jinn@Liu:~/openpilot/tools/sim$ sudo apt update
(OP) jinn@Liu:~/openpilot/tools/sim$ apt-cache search docker-ce
(OP) jinn@Liu:~/openpilot/tools/sim$ sudo apt update
(OP) jinn@Liu:~/openpilot/tools/sim$ sudo apt install docker-ce
(OP) jinn@Liu:~/openpilot/tools/sim$ sudo apt-get install docker-ce-cli
(OP) jinn@Liu:~/openpilot/tools/sim$ sudo ./start_openpilot_docker.sh
--- docker: Error response from daemon: could not select device driver "" with capabilities: [[gpu]].
--- Failed. uninstall all packages
(OP) jinn@Liu:~/openpilot/tools/sim$ grep install /var/log/dpkg.log
--- uninstall all packages
(OP) jinn@Liu:~/openpilot/tools/sim$ grep "2021-04-16.*.install " /var/log/dpkg.log | awk '{ print $4 }' | cut -d: -f1 | xargs sudo apt-get --yes purge

--------------------------------------------------
----- Appendix
PDF
Install Adobe and Change default app for PDF
--- How to install Adobe Acrobat Reader on Ubuntu Linux
--- After install
jinn@Liu:~$ ls /usr/share/applications/adobe.desktop
--- /usr/share/applications/adobe.desktop
jinn@Liu:~$ for type in pdf x-pdf fdf xdp xfdf pdx ; do
    xdg-mime default adobe.desktop application/$type
done
jinn@Liu:~$ gedit ~/.config/mimeapps.list
