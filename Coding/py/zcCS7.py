CAN 19   JLL, 2021.7.23, 9.10

Run model opEffPN.dlc by uiview.py on comma two (c2) with OP079
--------------------------------------------------

--- 1. download c2key.pem and c2key.ppk
https://drive.google.com/file/d/1flZET8lv85UQwe2WY3VwWch2O4QhHDqb/view?usp=sharing
https://drive.google.com/file/d/1oEMJlDxGWDkmGFU1CLR-Rn8_pUNqccoJ/view?usp=sharing
--- 2. ssh to c2
jinn@Liu:~$ ssh root@10.0.0.13 -p 8022 -i c2key.pem
--- 3. scp opEffPN.dlc to c2 (scp — secure copy (remote file copy program))
jinn@Liu:~$ scp -i c2key.pem -P 8022 ./opEffPN.dlc root@10.0.0.13:/data/openpilot/models/opEffPN.dlc
--- it is easier to use FileZilla (Google to install) to transfer files to c2 and back by c2key.ppk
--- 4. rebuild OP
root@localhost:/data/openpilot$ scons
--- 5. tmux on c2 (Terminal Multiplexing allows multitasking in a terminal window)
root@localhost:/data/openpilot$ tmux at
--- now in bash 0
--- 6. kill all daemon (background) processes *d like thermald
--- type ctrl + c to kill all *d
everything is dead
--- type ` then c (` c) and now in  bash 1
root@localhost:/$ cd /data/openpilot
--- 7. run uiview.py
root@localhost:/data/openpilot/selfdrive/debug$ python uiview.py
--- See output on c2 screen. Done.
