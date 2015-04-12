#AMS Group L
Repository for Group L's Autonomous Mechatronic Systems robotics project.

#Aim
The aim of our project is to design a robot that will demonstrate capabilities useful for search and rescue and emergency response operations. Our intended design will be capable of navigating and mapping a largely unknown environment, detecting and localising sound-emitting objects or persons, and determining a person’s status based on observed body temperature, detected sounds (as of breathing, talking, cries for help) and any observed motion using the on-board camera.

This capability could then be used either to notify on-site responders to the presence and locations of potential rescue candidates, or possibly even to provide some form of immediate assistance or medical treatment – however, that is outside of the bounds of this project, and is simply an example of the real-world applications of the system. For the purposes of this project, the system will sound an audible alert if a potential survivor is detected.

#Required Hardware
Our system will use a LIDAR unit for localisation and environmental mapping (SLAM), an IR temperature sensor to measure object temperatures in order to differentiate between debris and living people, and an array of microphones in order to perform sound localization. 

#Needs
* Ability to simultaneously localise within and map a bounded, but largely unknown environment
* Ability to navigate the environment based on the internally generated map
*	Ability to locate potential survivors based on sound
*	Ability to determine if a potential survivor is living or not
*	Ability to alert others to presence of survivor

#Requirements
##Core Requirements
*	Able to map the immediate environment as an occupancy grid that correlates with 85% to the ideal occupancy grid.
*	Able to accurately measure temperature of nearby objects (< 50cm, in front of IR temp sensor) within +/- 2°C.
*	Able to choose the most efficient pathway through the mapped environment > 90% of the time.
*	Able to localise multiple impulse sound sources within a 10m radius to within +/- 50cm accuracy, in robot-centric coordinates (as in finding preplaced sonic beacons). In a multi-robot scenario, such beacons will notionally have been placed by a previous "pathfinder" robot, which we will simulate by manual placement of beacons.
*	Able to traverse the perimeter of a detected object (e.g. Lumelski-class algorithms) while maintaining a fixed distance from the perimeter within +/- 5%.
*	Able to determine (with 75% accuracy) whether or not the object of interest is a living person.
##Extension Requirements
*	Able to successfully image the object of interest with 75% certainty that the correct object is the subject.
*	Able to detect the motion of nearby objects in front of the platform (for objects > 10cm moving at > 0.1 m/s). This is simply detecting whether or not there is motion, not the properties of that motion.
*	Capable of selecting station points around an object of interest (ranging from the size of the robot to the size of a prone person) for the purposes of achieving efficient imaging coverage (with overlap between successive images of 10% +/- TBD) at a distance from the object dependent on the given cameras FOV.

#Group Members
*	Chris Barr
*	Bobby Nansel
*	Eric Lai
*	Premkumar Katti

#Allocated Roles
*	Sound Detection and Localisation – Bobby
*	SLAM & IR Temp – Chris
*	Close-Range Object Avoidance & Localisation (Motor Encoders + IMU) - Eric
*	Navigation / Path Planning - Prem

#Test Method
The proposed system will be required to navigate a test space consisting of unknown stationary obstacles, sporadic-sound sources both at temperatures in the defined human body-temperature range (simulated by an actual person), and at other temperatures outside this range (simulated by a radio or similar sound emitting object). 

The system will be evaluated on how accurately it can localise a sound source, how successfully it can navigate the unknown environment towards that sound source, and how effective its ability to differentiate between survivor candidates and other sound sources.
