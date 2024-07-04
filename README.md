# gym2real
An advanced educational platform for analyzing reinforcement learning algorithms in realtime.

# Required Materials
- Jetson Nano development board running L4T version 32.6.1 or later (It has to be running docker)
- `docker-compose`

# Installing
First, clone this repository onto your Jetson development board with 

        git clone --recurse-submodules https://github.com/Sim2Capstone/twip/

From the `config/dockerfiles/ros`  directory, build the Docker image with

        docker build -t {image_name} .

Source the bash file 

        source setup.bash
# Usage
Run a new container from the twip Docker image with

          ros {image_name}
all the flags are set in the setup.bash file, that is sourced before. 

If you have an esp32 connected,run a separate docker container with the microros agent with the command

          run_agent 

Once the container has begun, make sure be in the `ros_ws` directory and run.

        colcon build
        
