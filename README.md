SIMUL_DSA
=========

[![IMAGE ALT TEXT HERE](http://img.youtube.com/vi/nK_O5MWdfBg/0.jpg)](http://www.youtube.com/watch?v=nK_O5MWdfBg)

![alt text](https://github.com/dsapandora/SIMUL_DSA/raw/master/agressive_agent.jpg "Images of the project 1")
![alt text](https://github.com/dsapandora/SIMUL_DSA/raw/master/car.jpg "Images of the project 1")
![alt text](https://github.com/dsapandora/SIMUL_DSA/raw/master/crash.jpg "Images of the project 1")
![alt text](https://github.com/dsapandora/SIMUL_DSA/raw/master/first_approuch_ros_agent.jpg "Images of the project 1")
![alt text](https://github.com/dsapandora/SIMUL_DSA/raw/master/in_progress.jpg	 "Images of the project 1")
![alt text](https://github.com/dsapandora/SIMUL_DSA/raw/master/neural_net.jpg	 "Images of the project 1")
![alt text](https://github.com/dsapandora/SIMUL_DSA/raw/master/simulation.jpg	 "Images of the project 1")
![alt text](https://github.com/dsapandora/SIMUL_DSA/raw/master/structure_ros.jpg	 "Images of the project 1")
![alt text](https://github.com/dsapandora/SIMUL_DSA/raw/master/neuronal.jpg	 "Images of the project 1")

ROS Controller for autonomous vehicle, using opencv for the neural network input. Using Kinect and Interface controller
In this project has been implemented a simulator for advanced driver assistance system (ADAS). This simulator allows us to measure the vehicle’s reactions in different test scenarios with other vehicles in the system. To achieve this goal a multi-agent architecture has been implemented to interact with the driver that use this simulator. 
To model world inside the simulator, we make uses of robotics simulator tool named Webot. With this tool, different kinds of test scenarios that include vehicles, traffic signals, buildings and parking’s lots, etc. were develop. In this every sceneries the vehicle has his own sensors and actuator that allow, to him and driver, the interaction with simulator.  

Simul-A2 has been created using the Webots useful tools. With the propose of interaction between the simulated environment in Webots, with a real environment (test car), we have been design communication scheme between the driven, agents, the simulator and a real car, using the tools and libraries developed by the ROS Project (Robot Operating System).  

In Simul-A2 we have all the car’s sensors information nodes integrated for communication (real car o simulated car), allowing the driver and/or the agent to get all the information of the scenario around them. In the other hand, all the actions of this elements. Will be processed and send it back to the simulator.  

With the goal to give a better and realistic experience for the user, Simul-A2, in all his scenarios integrate a of group agents that can drive the cars that will interact with the driver. This agents are 3T hybrid architecture control based agents This architecture has 3 layers of control that allow us to the divided the agent actions in two biggest groups: one for the reactive abilities and the other one or the deliberative strategies the 3T architecture was integrated with ROS allowing a better event responds with his complete communication system, and the ability to add different kinds of sensors to the system.. 


With the goal to enhance the user experience with the simulator, and make more realistic interaction with the other cars, we made a measure between the different kinds of agents in the system and the driver. From the data that we got from the 3T agent’s and the SPADE architecture agent (Smart Python multi-Agent Development Environment) we evaluate how good behavior of every agent against the human behavior with a driver interface that is connected to the simulator. All the result have been showing us that the 3T Architecture agent based with a behavior cloning learning strategy allow a better experience for the user.  This is because behavior cloning based in artificial neuronal network agent show us more like human behavior than the others agents when is driving the car. http://ros.org/wiki/vehiculo_autonomo


