# Duckie Lab

## Prerequisites

- Duckiebot 
- Laptop with VS Code and  the Docker and Dev Container extensions
- Laptop and the Duckiebot must be in the same network:

        SSID: robo-duckie, PSK: entenhausen

## Instructions
1. Create a ssh key pair and copy your public key to the duckiebot:

        ssh-copy-id duckie@alpha.local

   Do **not** use a passphrase for your ssh-key.

1. Configure Docker Environment in VS Code

   **Windows**: In VS Code, navigate to **File** > **Preferences** > **Settings** > **Docker Environment**.
   
   **Mac**: In VS Code, click on the **Manage** icon (located in the lower-left corner) > **Settings** > **Docker** > **Docker: Environment**.

    Add the following entry:
   - `DOCKER_HOST`: `ssh://duckie@ROBOT_NAME.local`
   - Replace `ROBOT_NAME` with your robot’s name, e.g., `alpha`.

1. Create a Docker context that points to the duckie robot. Issue the following command from terminal window:

        docker context create duckie --docker "host=ssh://duckie@alpha:22"


1. Use the Command Palette (Ctrl+Shift+P) to issue the `Docker Contexts: Use` command to activate the Docker context. 

2. In VS Code, right-click on `docker-compose.yml` and start it by selecting `Docker compose up`. Use the password `quackquack`.  This will create a new container on the duckiebot (its Nvidia Jetson Nano). Instead of entering the password multiple times, you can use a ssh key pair ([Link](https://code.visualstudio.com/docs/remote/troubleshooting)).

3. Press `F1` and type `Dev Containers: Attach to running container` and chose the container `/duckie_lab-ros`.

4. In the new VS Code window, you will see on the bottom left something like `Container duckie_lab-ros`. 

5. Have a look at the python files in `/code/catkin_ws/src/duckie-lab/src/`, for example `camera_subscriber.py`. Change the variable `robot_name` to the name of your robot. 

6. Run the python code of the `camera_subscriber.py` in the terminal 

        rosrun duckie_lab camera_subscriber.py

   **Warning**: The code in the `/code/catkin_ws/src/duckie-lab` folder is shared with your duckie robot as a docker volume, but not with your computer. In order to prevent code loss, use a proper version control to back up your code frequently!


## Troubleshooting

**Problem**: *I can't launch the Python node.*
        
1. Make sure the python file is executable. In a terminal, type

        chmod +x camera_subscriber.py

2. Make sure the workspace is built

        cd /code/catkin_ws && catkin build

3. Make sure the workspace is sourced:

        source /code/catkin_ws/devel/setup.bash

**Problem**: *I have installed some PIP packages in the container, but they are lost.*

+ After you have used the container it is recommended to stop the container (not compose down).
If you use the compose down command the container will be deleted,
and you will lose all the pip packages etc. you have installed.

**Problem**: *I want to use the Duckiebot at home with my home network.*

+ You need to connect to your duckiebot while connected to the `robo-duckie` network:

        ssh duckie@ROBOTNAME.local

+ The password is `quackquack` by default.

+ Add the SSID and password of your home network by modifying the a configuration file:

        sudo sed -i 's/yourhomenetwork/MYSSID/g' /etc/wpa_supplicant.conf 
        sudo sed -i 's/password/MYPASSWORD/g' /etc/wpa_supplicant.conf

   Replace `MYSSID` and `MYPASSWORD` with the correct ssid and password of your home network. 

   **WARNING**: other students can easily access this information too!