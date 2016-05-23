# USARSimSampleClient
A sample client software for showing fundamental usage of RoboCupRescuePackage  

## Requirements  
* RoboCupRescuePackage (you can get it by a command:git clone https://github.com/m-shimizu/RoboCupRescuePackage)  
* Gazebo5 and ROS indigo(and hector_slam package)  


## How to setup  
        $ cd ~  
        $ git clone https://github.com/m-shimizu/USARSimSampleClient  
        $ cd USARSimSampleClient/src  
        $ catkin_init_workspace  
        $ cd USARSimSampleClient  
        $ catkin_make  


## How to use  
        Prepare 2 terminals.  
        
        * At Terminal1  
        $ cd ~/RoboCupRescuePackage  
        $ source ./.bashrc.USARGazebo  
        $ gazebo USARGazebo.world  (please use simnple block world)  
        
        * At Terminal2  
        $ cd ~/USARSimRsampleClient  
        $ source devel/setup.bash  
        $ ./start_sample_client.bash  


### What you can see with this sample client  
     - Spawning 4 robots (1 sec innterval is required between each robot spawing, please see ~/USARSimRsampleClient/start_sample_client.bash)
     - 4 robot's camera images transfered in jpeg format are shown on rviz (please see ~/USARSimRsampleClient/usarimage2ros.py, it gets a long jpeg camera image including 4 robot's camera image and cuts the long image into 4 ros image topic)
