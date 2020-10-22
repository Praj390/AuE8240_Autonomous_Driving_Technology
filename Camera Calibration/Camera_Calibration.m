%% Prajval Vaskar
% Advanced Driving Technologies
% HW02
clear 
clear all
%% 3.1. Finding 11 paramters for left camera
% For left camera and finding its 11 parameters based on Least Square
% Method
left = imread('Left.jpg');
figure(1)
imshow(left);
axis on
% Matrix A based on the given X, Y, Z co-ordinates of the object and its u
% and v values calculated from the image by reading the image in MATLAB.
AL = [0    0    0    1    0    0    0    0    0            0       0;
      0    0    0    0    0    0    0    1    0            0       0;
      0    35   0    1    0    0    0    0    0         -(35*2180) 0;
      0    0    0    0    0   35    0    1    0         -(35*2060) 0;
      100  35   0    1    0    0    0    0  -(100*2816) -(35*2816) 0;
      0    0    0    0   100  35    0    1  -(100*1772) -(35*1772) 0;
      100  0    0    1    0   0     0    0  -(100*3048)     0      0;
      0    0    0    0   100  0     0    1  -(100*1894)     0      0;
      75   0   -40   1    0   0     0    0  -(2874*75)      0 (40*2874);
      0    0    0    0   75   0    -40   1  -(2256*75)      0 (40*2256);
      25   0   -40   1   0    0     0    0  -(2562*25)      0 (2562*40);
      0    0     0   0   25   0    -40   1  -(2437*25)      0 (2437*40);
      50   0   -30   1   0    0     0    0  -(2735*50)      0 (2735*30);
      0    0     0   0   50   0    -30   1  -(2265*50)      0 (2265*30)];
 
% Matrx B is 14*1 size matrix contains the values of pixel co-ordinates of
% the 7 points given in the questions.

BL = [2397;2227;2180;2060;2816;1772;3048;1894;2874;2256;2562;2437;2735;2265];
XL = AL\BL;   % 11 parameters calculated from least square method (X= A\B)
%% 3.2. Finding 11 paramters for right camera
% For right camera
% For right camera and finding its 11 parameters based on Least Square
right = imread('Right.jpg');
figure(2)
imshow(right);
axis on

% Matrix A based on the given X, Y, Z co-ordinates of the object and its u
% and v values calculated from the image by reading the image in MATLAB.

AR = [0    0    0    1    0    0    0    0    0            0       0;
      0    0    0    0    0    0    0    1    0            0       0;
      0    35   0    1    0    0    0    0    0         -(35*1267) 0;
      0    0    0    0    0   35    0    1    0         -(35*1522) 0;
      100  35   0    1    0    0    0    0  -(100*1860) -(35*1860) 0;
      0    0    0    0   100  35    0    1  -(100*1727) -(35*1727) 0;
      100  0    0    1    0   0     0    0  -(100*1695)     0      0;
      0    0    0    0   100  0     0    1  -(100*1888)     0      0;
      75   0   -40   1    0   0     0    0  -(1558*75)      0 (40*1558);
      0    0    0    0   75   0    -40   1  -(2079*75)      0 (40*2079);
      25   0   -40   1   0    0     0    0  -(1263*25)      0 (1263*40);
      0    0     0   0   25   0    -40   1  -(1948*25)      0 (1948*40);
      50   0   -30   1   0    0     0    0  -(1399*50)      0 (1399*30);
      0    0     0   0   50   0    -30   1  -(1953*50)      0 (1953*30)];
  
% Matrx B is 14*1 size matrix contains the values of pixel co-ordinates of
% the 7 points given in the questions.
  
 BR = [1085;1651;1267;1522;1860;1727;1695;1888;1558;2078;1263;1948;1399;1953]; %B matrix containing u and v of all co-ordinates
 XR = AR\BR; % 11 parameters calculated from least square method (X= A\B)
 
 % pixel co-ordinates of the points whose X ,Y, Z is to found out
 UL = [2396 3041 2538 2317 2700 2933 2888 2567 2500 2955]; %u coordinate vector for left camera image
 VL = [2308 1958 2154 1996 1821 1952 2117 2293 2254 2009]; %v coordinate vector for left camera image
 UR = [1094 1701 1195 1377 1730 1563 1547 1242 1175 1607]; %u coordinate vector for right camera image
 VR = [1710 1954 1696 1560 1682 1837 1954 1831 1741 1916]; %v coordinate vector for right camera image
 
 %%  3.3. X,Y,Z co-ordinates of the remaining points. 
 %      [b11-b31u1 b12-b32u1 b13-b33u1;
 %  A=   b21-b31v1 b22-b32v1 b13-b33v1;
 %       c11-c31ur c12-b32ur c13-c33ur;
 %       c21-c31vr c22-c32vr c23-c33vr]
 %  x = [X Y Z];
 
 %  B = [u1-b14;
 %       v1-b24;
 %       ur-c14;
 %       vr-c24]
 
 IA =    [XL(1)-XL(9)*UL(1), XL(2)-XL(10)*UL(1), XL(3)-XL(11)*UL(1);  %For I
         XL(5)-XL(9)*VL(1), XL(6)-XL(10)*VL(1), XL(7)-XL(11)*VL(1);
         XR(1)-XR(9)*UR(1), XR(2)-XR(10)*UR(1), XR(3)-XR(11)*UR(1);
         XR(5)-XR(9)*VR(1), XR(6)-XR(10)*VR(1), XR(7)-XR(11)*VR(1)];
 IB =    [UL(1)-XL(4);
         VL(1)-XL(8);
         UR(1)-XR(4);
         VR(1)-XR(8)];
 IX = IA\IB
 
 JA =    [XL(1)-XL(9)*UL(2), XL(2)-XL(10)*UL(2), XL(3)-XL(11)*UL(2); %For J
         XL(5)-XL(9)*VL(2), XL(6)-XL(10)*VL(2), XL(7)-XL(11)*VL(2);
         XR(1)-XR(9)*UR(2), XR(2)-XR(10)*UR(2), XR(3)-XR(11)*UR(2);
         XR(5)-XR(9)*VR(2), XR(6)-XR(10)*VR(2), XR(7)-XR(11)*VR(2)];
 JB =    [UL(2)-XL(4);
         VL(2)-XL(8);
         UR(2)-XR(4);
         VR(2)-XR(8)];
 JX = JA\JB
 
 KA =    [XL(1)-XL(9)*UL(3), XL(2)-XL(10)*UL(3), XL(3)-XL(11)*UL(3);  %For K
         XL(5)-XL(9)*VL(3), XL(6)-XL(10)*VL(3), XL(7)-XL(11)*VL(3);
         XR(1)-XR(9)*UR(3), XR(2)-XR(10)*UR(3), XR(3)-XR(11)*UR(3);
         XR(5)-XR(9)*VR(3), XR(6)-XR(10)*VR(3), XR(7)-XR(11)*VR(3)];
 KB =    [UL(3)-XL(4);
         VL(3)-XL(8);
         UR(3)-XR(4);
         VR(3)-XR(8)];
 KX = KA\KB
 
 LA =    [XL(1)-XL(9)*UL(4), XL(2)-XL(10)*UL(4), XL(3)-XL(11)*UL(4); %For L
         XL(5)-XL(9)*VL(4), XL(6)-XL(10)*VL(4), XL(7)-XL(11)*VL(4);
         XR(1)-XR(9)*UR(4), XR(2)-XR(10)*UR(4), XR(3)-XR(11)*UR(4);
         XR(5)-XR(9)*VR(4), XR(6)-XR(10)*VR(4), XR(7)-XR(11)*VR(4)];
 LB =    [UL(4)-XL(4);
         VL(4)-XL(8);
         UR(4)-XR(4);
         VR(4)-XR(8)];
 LX = LA\LB
 
 MA =    [XL(1)-XL(9)*UL(5), XL(2)-XL(10)*UL(5), XL(3)-XL(11)*UL(5); %For M
         XL(5)-XL(9)*VL(5), XL(6)-XL(10)*VL(5), XL(7)-XL(11)*VL(5);
         XR(1)-XR(9)*UR(5), XR(2)-XR(10)*UR(5), XR(3)-XR(11)*UR(5);
         XR(5)-XR(9)*VR(5), XR(6)-XR(10)*VR(5), XR(7)-XR(11)*VR(5)];
 MB =    [UL(5)-XL(4);
         VL(5)-XL(8);
         UR(5)-XR(4);
         VR(5)-XR(8)];    
 MX = MA\MB
 
 NA =    [XL(1)-XL(9)*UL(6), XL(2)-XL(10)*UL(6), XL(3)-XL(11)*UL(6); %For N
         XL(5)-XL(9)*VL(6), XL(6)-XL(10)*VL(6), XL(7)-XL(11)*VL(6);
         XR(1)-XR(9)*UR(6), XR(2)-XR(10)*UR(6), XR(3)-XR(11)*UR(6);
         XR(5)-XR(9)*VR(6), XR(6)-XR(10)*VR(6), XR(7)-XR(11)*VR(6)];
 NB =    [UL(6)-XL(4);
         VL(6)-XL(8);
         UR(6)-XR(4);
         VR(6)-XR(8)];    
 NX = NA\NB

 OA =    [XL(1)-XL(9)*UL(7), XL(2)-XL(10)*UL(7), XL(3)-XL(11)*UL(7); %For O
         XL(5)-XL(9)*VL(7), XL(6)-XL(10)*VL(7), XL(7)-XL(11)*VL(7);
         XR(1)-XR(9)*UR(7), XR(2)-XR(10)*UR(7), XR(3)-XR(11)*UR(7);
         XR(5)-XR(9)*VR(7), XR(6)-XR(10)*VR(7), XR(7)-XR(11)*VR(7)];
 OB =    [UL(7)-XL(4);
         VL(7)-XL(8);
         UR(7)-XR(4);
         VR(7)-XR(8)];    
 OX = OA\OB
 
 PA =    [XL(1)-XL(9)*UL(8), XL(2)-XL(10)*UL(8), XL(3)-XL(11)*UL(8); %For P
         XL(5)-XL(9)*VL(8), XL(6)-XL(10)*VL(8), XL(7)-XL(11)*VL(8);
         XR(1)-XR(9)*UR(8), XR(2)-XR(10)*UR(8), XR(3)-XR(11)*UR(8);
         XR(5)-XR(9)*VR(8), XR(6)-XR(10)*VR(8), XR(7)-XR(11)*VR(8)];
 PB =    [UL(8)-XL(4);
         VL(8)-XL(8);
         UR(8)-XR(4);
         VR(8)-XR(8)];    
 PX = PA\PB
 
 QA =    [XL(1)-XL(9)*UL(9), XL(2)-XL(10)*UL(9), XL(3)-XL(11)*UL(9); %For Q
         XL(5)-XL(9)*VL(9), XL(6)-XL(10)*VL(9), XL(7)-XL(11)*VL(9);
         XR(1)-XR(9)*UR(9), XR(2)-XR(10)*UR(9), XR(3)-XR(11)*UR(9);
         XR(5)-XR(9)*VR(9), XR(6)-XR(10)*VR(9), XR(7)-XR(11)*VR(9)];
 QB =    [UL(9)-XL(4);
         VL(9)-XL(8);
         UR(9)-XR(4);
         VR(9)-XR(8)];    
 QX = QA\QB
 
 RA =    [XL(1)-XL(9)*UL(10), XL(2)-XL(10)*UL(10), XL(3)-XL(11)*UL(10); %For R
         XL(5)-XL(9)*VL(10), XL(6)-XL(10)*VL(10), XL(7)-XL(11)*VL(10);
         XR(1)-XR(9)*UR(10), XR(2)-XR(10)*UR(10), XR(3)-XR(11)*UR(10);
         XR(5)-XR(9)*VR(10), XR(6)-XR(10)*VR(10), XR(7)-XR(11)*VR(10)];
 RB =    [UL(10)-XL(4);
         VL(10)-XL(8);
         UR(10)-XR(4);
         VR(10)-XR(8)];    
 RX = RA\RB
 
%% 3.4 Finding dimensions of the Bar 
 % For length of the bar given in picture
 Ll1 =   [XL(1)-XL(9)*2334,  XL(2)-XL(10)*2334, XL(3)-XL(11)*2334;
         XL(5)-XL(9)*874, XL(6)-XL(10)*874, XL(7)-XL(11)*874;
         XR(1)-XR(9)*2014, XR(2)-XR(10)*2014, XR(3)-XR(11)*2014;
         XR(5)-XR(9)*772, XR(6)-XR(10)*772, XR(7)-XR(11)*772];
 Lr1 =   [2334-XL(4);
         874-XL(8);
         2014-XR(4);
         772-XR(8)]; 
 L1 = Ll1\Lr1;
 
 Ll2 =   [XL(1)-XL(9)*2332,  XL(2)-XL(10)*2332, XL(3)-XL(11)*2332;
         XL(5)-XL(9)*1865, XL(6)-XL(10)*1865, XL(7)-XL(11)*1865;
         XR(1)-XR(9)*2023, XR(2)-XR(10)*2023, XR(3)-XR(11)*2023;
         XR(5)-XR(9)*1668, XR(6)-XR(10)*1668, XR(7)-XR(11)*1668];
 Lr2 =   [2332-XL(4);
         1865-XL(8);
         2023-XR(4);
         1668-XR(8)]; 
 L2= Ll2\Lr2;
 
 Length = sqrt((L1(1)-L2(1))^2 + (L1(2)-L2(2))^2 + (L1(3)-L2(3))^2)
 fprintf("The length of the mold is %f mm \n", Length)
 
 %For width
 
 Wl1 =   [XL(1)-XL(9)*2334,  XL(2)-XL(10)*2334, XL(3)-XL(11)*2334;
         XL(5)-XL(9)*874, XL(6)-XL(10)*874, XL(7)-XL(11)*874;
         XR(1)-XR(9)*2014, XR(2)-XR(10)*2014, XR(3)-XR(11)*2014;
         XR(5)-XR(9)*772, XR(6)-XR(10)*772, XR(7)-XR(11)*772];
 Wr1 =   [2334-XL(4);
         874-XL(8);
         2014-XR(4);
         772-XR(8)]; 
 W1 = Wl1\Wr1;
 
 Wl2 =   [XL(1)-XL(9)*2473,  XL(2)-XL(10)*2473, XL(3)-XL(11)*2473;
         XL(5)-XL(9)*846, XL(6)-XL(10)*846, XL(7)-XL(11)*846;
         XR(1)-XR(9)*2136, XR(2)-XR(10)*2136, XR(3)-XR(11)*2136;
         XR(5)-XR(9)*806, XR(6)-XR(10)*806, XR(7)-XR(11)*806];
 Wr2 =   [2473-XL(4);
         846-XL(8);
         2136-XR(4);
         806-XR(8)]; 
 W2= Wl2\Wr2;
 
 Width = sqrt((W1(1)-W2(1))^2 + (W1(2)-W2(2))^2 + (W1(3)-W2(3))^2)
 fprintf("The width of the mold is %f mm \n", Width)
 
 % For height 
 
 Hl1 =   [XL(1)-XL(9)*2334,  XL(2)-XL(10)*2334, XL(3)-XL(11)*2334;
         XL(5)-XL(9)*874, XL(6)-XL(10)*874, XL(7)-XL(11)*874;
         XR(1)-XR(9)*2014, XR(2)-XR(10)*2014, XR(3)-XR(11)*2014;
         XR(5)-XR(9)*772, XR(6)-XR(10)*772, XR(7)-XR(11)*772];
 Hr1 =   [2334-XL(4);
         874-XL(8);
         2014-XR(4);
         772-XR(8)]; 
 H1 = Hl1\Hr1;
 
 Hl2 =   [XL(1)-XL(9)*2254,  XL(2)-XL(10)*2254, XL(3)-XL(11)*2254;
         XL(5)-XL(9)*847, XL(6)-XL(10)*847, XL(7)-XL(11)*847;
         XR(1)-XR(9)*2112, XR(2)-XR(10)*2025, XR(3)-XR(11)*2025;
         XR(5)-XR(9)*735, XR(6)-XR(10)*735, XR(7)-XR(11)*735];
 Hr2 =   [2254-XL(4);
         847-XL(8);
         2112-XR(4);
         735-XR(8)]; 
 H2= Hl2\Hr2;
 
 Height = sqrt((H1(1)-H2(1))^2 + (H1(2)-H2(2))^2 + (H1(3)-H2(3))^2)
 fprintf("The height of the mold is %f mm \n", Height)
 
 %% 3.5. Finding the volume of the bottle.
 % Volume of the bottle
 % For radius
 Rl1 =   [XL(1)-XL(9)*1212,  XL(2)-XL(10)*1212, XL(3)-XL(11)*1212;
         XL(5)-XL(9)*908, XL(6)-XL(10)*908, XL(7)-XL(11)*908;
         XR(1)-XR(9)*1698, XR(2)-XR(10)*1698, XR(3)-XR(11)*1698;
         XR(5)-XR(9)*586, XR(6)-XR(10)*586, XR(7)-XR(11)*586];
 Rr1 =   [1212-XL(4);
         908-XL(8);
         1698-XR(4);
         586-XR(8)]; 
 R1 = Rl1\Rr1;
 
 Rl2 =   [XL(1)-XL(9)*1713,  XL(2)-XL(10)*1713, XL(3)-XL(11)*1713;
         XL(5)-XL(9)*879, XL(6)-XL(10)*879, XL(7)-XL(11)*879;
         XR(1)-XR(9)*2090, XR(2)-XR(10)*2090, XR(3)-XR(11)*2090;
         XR(5)-XR(9)*575, XR(6)-XR(10)*575, XR(7)-XR(11)*575];
 Rr2 =   [1713-XL(4);
         879-XL(8);
         2090-XR(4);
         575-XR(8)]; 
 R2= Rl2\Rr2;
 
 Radius = (sqrt((R1(1)-R2(1))^2 + (R1(2)-R2(2))^2 + (R1(3)-R2(3))^2))/2
 fprintf("The radius of the bottle is %f mm \n", Radius)
 
 % For height
 hl1 =   [XL(1)-XL(9)*1558,  XL(2)-XL(10)*1558, XL(3)-XL(11)*1558;
         XL(5)-XL(9)*1795, XL(6)-XL(10)*1795, XL(7)-XL(11)*1795;
         XR(1)-XR(9)*1916, XR(2)-XR(10)*1916, XR(3)-XR(11)*1916;
         XR(5)-XR(9)*1308, XR(6)-XR(10)*1308, XR(7)-XR(11)*1308];
 hr1 =   [1558-XL(4);
         1795-XL(8);
         1916-XR(4);
         1308-XR(8)]; 
 h1 = hl1\hr1;
 
 hl2 =   [XL(1)-XL(9)*1413,  XL(2)-XL(10)*1413, XL(3)-XL(11)*1413;
         XL(5)-XL(9)*384, XL(6)-XL(10)*384, XL(7)-XL(11)*384;
         XR(1)-XR(9)*1875, XR(2)-XR(10)*1875, XR(3)-XR(11)*1875;
         XR(5)-XR(9)*175, XR(6)-XR(10)*175, XR(7)-XR(11)*175];
 hr2 =   [1413-XL(4);
         384-XL(8);
         1875-XR(4);
         175-XR(8)]; 
 h2= hl2\hr2;
 
 height = sqrt((h1(1)-h2(1))^2 + (h1(2)-h2(2))^2 + (h1(3)-h2(3))^2)
 fprintf("The height of the bottle is %f mm \n", height)
 
 volume = pi*Radius^2 * height * 10^-6 
 fprintf("The volume of the bottle is %f litre \n", volume)
 
 
 
         







