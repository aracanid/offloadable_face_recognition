## AIM

The aim of this project was to design, build, and test a small robot using Lego Mindstorms EV3 that is capable of performing face tracking using an on board high definition webcam. 

The face tracking algorithm is split into multiple sections, hereby known as nodes, and these nodes are used to demonstrate computational offloading triggered based primarily on the robots Central Processing Units (CPU) current load. As the load on the robots CPU increases, its scheduler dynamically offloads nodes in order to keep the system responsive, providing that there is a sufficiently fast Internet connection. 

The robot should respond to the presence of a user by attempting to centre itself with respect to the user by rotating or elevating its motor positions. In order to demonstrate the computational offloading in an intuitive and interactive way, a Graphical User Interface (GUI) will be developed that allows the user to not only view the offloading occurring but to also interact with the system. This interaction will take the form of artificial CPU usage levels being sent to the robots scheduler and hence forcing offloading to occur at a time convenient to the user. Finally the system should be robust enough to cope with the possibility that nodes may fail, messages may be lost, or the remote server could become unresponsive.

## Motivation

Computational offloading is generally used to migrate an application or task to a more resourceful computer or network of computers. Deciding what tasks may be offloaded and when to offload them can only be determined once the aim of the offloading has been decided. With each passing year the number of portable devices sold increases dramatically as well as the number of robots. 

As these devices continue to offer increasing amounts of processing power and communication bandwidth, there is a larger burden placed on their battery capacity in order to ensure they can keep up with the energy requirements of said processors, whilst still providing a good user experience. Therefore one potential aim for the offloading of tasks is to conserve energy in order to extend the battery life by reducing the CPU load but also keeping the bandwidth to the minimum.
