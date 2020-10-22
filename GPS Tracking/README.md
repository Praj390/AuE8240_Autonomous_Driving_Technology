# GPS Poition based on Satelite Position
The position of four satellites are given in the table below in the unit of Mm (megameter).
![Table](https://github.com/Praj390/AuE8240_Autonomous_Driving_Technology/blob/master/GPS%20Tracking/Table.jpg)
A GPS unit receives signals from the four satellites and the signals incorporate the positions
of each satellite and also the following information:
- Sending times of each signal at the satellites, which are stored in file “st.mat”
- Receiving times of each signal at the GPS unit, which are stored in file “rt.mat”
Please calculate the position of the GPS unit in the unit of Mm and also the clock difference between the GPS unit clock and Satellite system clock in the unit of s (second) using numerical optimization function in Matlab. You need to submit both the solution presentation and codes.
Additional Notes:
• Use Matlab command, <load ‘Name.mat’>, to load the data. For example, command <load ‘st.mat’> will load a variable vector “st” and “st(1), st(2), st(3) and st (4)” will be the sending times of satellite 1, 2, 3 and 4 respectively. Do the same for “rt.mat”.
• Speed of GPS radio wave: c = 300 Mm/s
• All the satellites share the same clock – satellite system clock.
• The available numerical optimization functions with examples in Matlab can be found at
https://www.mathworks.com/help/optim/referencelist.html?type=function. Please explore and try different functions, such as “fminsearch”, “fminumc”, “fmincon”, etc., to find out the best appropriate function.
• The radius of the earth is 6.378 Mm. The range of the GPS unit’s position coordinates is [5, 20] Mm and the range of clock differences is [-10, 10] s. 