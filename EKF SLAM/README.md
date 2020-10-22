# EKF SLAM
A vehicle initially stops in an unknown environment. It detects two obstacles based on its onboard lidar. The initial vector in EKF SLAM is defined as

𝜇0 = [𝑥, 𝑦, 𝜃, 𝑥1, 𝑦1, 𝑥2, 𝑦2 ]

𝑇 = [0, 0, 0, 147, 102, 98, 53 ]𝑇 
where (𝑥, 𝑦, 𝜃) is the robot states, and (𝑥1, 𝑦1) and (𝑥2, 𝑦2) are locations of the two objects The vehicle moves in a straight line with a constant speed 𝑣𝑡 = 1 𝑚\𝑠, (𝜔𝑡 = 0). As the vehicle moves, the lidar detects the relative distances and relative angles of the two objects: (𝑟1,𝜙1) and (𝑟2 𝜙2) in units of meter and radian, which are saved in file “s1.mat” and “s2 mat”. The sampling time is Δ𝑡 = 1𝑠. 

The noises in the motion and sensing are both Gaussian 𝑁(0, 𝑅) and 𝑁(0,𝑄), where
![](https://github.com/Praj390/AuE8240_Autonomous_Driving_Technology/blob/master/EKF%20SLAM/matrix.jpg)
Implement EKF SLAM in Matlab to find out the locations of the robot and two objects, i.e., update 𝜇 based on the data in “s1.mat” and “s2.mat”.
Note: It is a simple straight line motion so steering can be ignored in the motion model.
