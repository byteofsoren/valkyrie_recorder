# valkyrie_recorder
A ROS repository to capture images from a web cam.
## Installation
This version of the system requires a directory in the home directory called `repos` containing the following structure to work.

``` bash
HOME/
    |
    |-repos
    |   |-valkirye_data

```

To create the that structure write this in a terminal.

``` bash
$ cd
$ mkdir -p repos/valkyrie_data
$ cd repos

```

Then clone this module.

``` bash
$ git clone https://github.com/byteofsoren/valkyrie_recorder.git
```

## Installing dependency.
This ROS module uses already implemented code to get the camera working.
But it still needs to be installed in to the system.


``` bash
$ sudo apt install ros-{rosdistro}-usb-cam*
$ sudo apt install ros-{rosdistro}-rospy
```
Of course you replace the rosdirstro with what ever you use. This have only as this is written bean tested on ROS1 noetic on a Ubuntu 20.04.

If the intended system want to have the USB camera working remotely on a crane the system could be installed on a Raspberry PI and then send the frames over to a other system to do the image transformation.
The only difference is where you launch the usb_cam node.

### Compiling
To compile the program first run the rosdep.

``` bash
$ rosdep update
```
This needs to be done twice is you are running the camera node on a different system. Once on the camera sender system and once on the receiver.

## Start the program.
The firs step to start this program is as always to first start the roscore.

``` bash
$ roscore
```

Then start both the scripts form two different terminals.
``` bash
$ roslaunch userinterface usb_cam.launch
$ roslaunch userinterface terminal.launch
```

Once the terminal.launch starts it will ask you a question on what name the new directory where you want to store the images.
Let say that the user writes "Testimages_preson1_onsleftside"
Then the system will in the previously mentioned `$HOME/repos/valkyrie_data/` directory create an image.


Now the program have started and an image should soon appear on the screen.
When that happens you can star taking images by pressing the *enter* key.
The first image is stored like  "$HOME/repos/valkyrie_data/Testimages_person1_onliftside/img_001.png"
And consequently the next image is name "img_002.png"

To quit the image taker node just press the *Q* key on the keyboard.
While the sender node you just press the *CTRL+C* key combination.

