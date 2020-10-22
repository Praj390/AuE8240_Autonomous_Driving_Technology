# EKF SLAM
A vehicle initially stops in an unknown environment. It detects two obstacles based on its onboard lidar. The initial vector in EKF SLAM is defined as 
𝜇𝜇0 = [𝑥𝑥, 𝑦𝑦, 𝜃𝜃, 𝑥𝑥1, 𝑦𝑦1, 𝑥𝑥2, 𝑦𝑦2 ] 
𝑇𝑇 = [0, 0, 0, 147, 102, 98, 53 ]𝑇𝑇 where (𝑥𝑥, 𝑦𝑦, 𝜃𝜃) is the robot states, and (𝑥𝑥1, 𝑦𝑦1) and (𝑥𝑥2, 𝑦𝑦2) are locations of the two objects The vehicle moves in a straight line with a constant speed 𝑣𝑣𝑡𝑡 = 1 𝑚𝑚 𝑠𝑠, (𝜔𝜔𝑡𝑡 = 0). As the vehicle moves, the lidar detects the relative distances and relative angles of the two objects: (𝑟𝑟1,𝜙𝜙1) and (𝑟𝑟2 𝜙𝜙2) in units of meter and radian, which are saved in file “s1.mat” and “s2 mat”. The sampling time is Δ𝑡𝑡 = 1 𝑠𝑠. 

The noises in the motion and sensing are both Gaussian 𝑁𝑁(0, 𝑅𝑅) and 𝑁𝑁(0,𝑄𝑄), where
[](\matrix)
Implement EKF SLAM in Matlab to find out the locations of the robot and two objects, i.e., update 𝜇𝜇 based on the data in “s1.mat” and “s2.mat”.
Note: It is a simple straight line motion so steering can be ignored in the motion model.
