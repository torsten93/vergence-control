Vergence Control with the iCub robot
====================================

### :computer: Installation
To install the required dependencies, follow these [instructions](http://wiki.icub.org/wiki/ICub_Software_Installation).

### Run the code
First, you have to import the simulator's configuration file for customization:
```sh
>> yarp-config context --import simConfig iCub_parts_activation.ini
```

Then, edit the file `iCub_parts_activation.ini` in the path `%APPDATA%/yarp/contexts/simConfig`
(for Windows) or `$HOME/.local/share/yarp/contexts/simConfig` (for Linux)
by setting the option **screen** equal to **on** (under the group _RENDER_).

Now that all is set, launch the `yarpmanager` GUI and open the application file
[app-icubsim-system.xml](./icub/apps/app-icubsim-system.xml).

Finally, do the following things:
1. Run the modules contained in the application.
1. Launch from the console the binary `iCubSimScreenHandler`.
1. Launch from the console the binary `iCubVergenceControl`.
1. From within the GUI, connect the ports.
