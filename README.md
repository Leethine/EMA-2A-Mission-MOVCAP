# EMA-2A-Mission-MOVCAP
"projet MOVCAP": developing a tool for capturing the kinematics of human movement capable of tracking its target from an aerial drone.

MOVCAP is an R&D project by École des Mines d'Alès (EMA), an institute of higher education in the south of France. 

The development of this repository is part of the courses assigned to 2nd-year students at EMA, it aimes to develop a prototype and algorithms for the project.

The goal of the MOVCAP project is to develop a tool for capturing the kinematics of human movement capable of following its target during its movements from an aerial UAV.

Thus, the UAV must be able to position itself in such a way as to obtain quality images of the sportsman in order to analyse his movements but also to follow his target in motion throughout the sequence in real time.

This task represents the cornerstone of the MOVCAP project since it consists in positioning the UAV equipped with a MOCAP data capture system at a precise position in relation to the target. Without this step, the data captured by the MOCAP system will not be usable, since it is out of range or has poor perspectives. The different steps are :

- Image capture and quality improvement (noise removal, contrast...)

- Detecting the target in the image

- Follow the target throughout the sequence (send the target position to the UAV for a repositioning and adjustment of the image capture).

- Develop a real-time method for the algorithm to send information instantaneously.



The prototype of the project is developed in OpenCV and its C++ interface, but most algorithms are tested in its Python/Octave bindings.
