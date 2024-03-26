# unitree_cameras

## Installation

ssh into the Jetson that you want to get the camera feed from (head/body)

Clone with submodules
```
git clone --recursive git@github.com:matval/unitree_cameras.git
```

To ensure the camera messages are synchronized with the other messages, install chrony in the camera's jetson devices:
```
sudo apt-get install chrony
```
