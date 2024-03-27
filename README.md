# unitree_cameras

## Installation

ssh into the Jetson that you want to get the camera feed from (head/body)

Clone with submodules
```
git clone --recursive git@github.com:matval/unitree_cameras.git
```

## Time synchronization in multiple devices
To ensure the camera messages are synchronized with the other messages, install chrony in the camera's jetson devices:
```
sudo apt-get install chrony
```
Choose one machine as time server (usually 192.168.123.162).
On this machine, do:
```
sudo vim /etc/chrony/chrony.conf
```
Add these lines:
```
# make it serve time even if it is not synced (as it can't reach out)
local stratum 8
# allow the IP of your peer to connect
allow 192.168.123.13
allow 192.168.123.14
```
Then, on the clients (`192.168.123.13` and `192.168.123.14`):
```
sudo vim /etc/chrony/chrony.conf
```
Add this lines:
```
server 192.168.123.162 minpoll 0 maxpoll 5 maxdelay .05
```
Change maxdelay as you wish.

Reboot the robot.
