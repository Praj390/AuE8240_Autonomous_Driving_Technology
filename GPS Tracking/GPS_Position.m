clear all
clc
% Prajval Vaskar
%% Load files of sending and receiving times at satellites..
load 'st.mat'
load 'rt.mat'
%% Satelite position in Mm
X = [101, 52, 17, -15];
Y = [16, 21, 53, 159];
Z = [207, 302, 350, 208];
s = 300;    %Speed of GPS radio clock wave in Mm

x0 = [0 0 0 0];   % Initial condition
lb = [5 5 5 -10]; % Upper bound
ub = [20 20 20 10]; % Lower bound

% Equation formed with 4 satellite co-ordinates

fun = @(x)(((x(1)-X(1))^2+(x(2)-Y(1))^2+(x(3)-Z(1))^2)^(1/2)-s*(rt(1)-st(1)-x(4)))^2 + (((x(1)-X(2))^2+(x(2)-Y(2))^2+(x(3)-Z(2))^2)^(1/2)-s*(rt(2)-st(2)-x(4)))^2 +(((x(1)-X(3))^2+(x(2)-Y(3))^2+(x(3)-Z(3))^2)^(1/2)-s*(rt(3)-st(3)-x(4)))^2+(((x(1)-X(4))^2+(x(2)-Y(4))^2+(x(3)-Z(4))^2)^(1/2)-s*(rt(4)-st(4)-x(4)))^2;

%% Using fmincon optimization function
f = fmincon(fun,x0,[],[],[],[],lb,ub,@inequality)
% For checking the distance of the GPS from the earth centre.
R = sqrt(f(1)^2 + f(2)^2 + f(3)^2)

%% Position of GPS
fprintf('The Position of GPS unit in the unit of Mm is (X,Y,Z) = [%f,%f,%f]',f(1), f(2), f(3))
fprintf('\n')
fprintf('The clock difference between the GPS unit clock and Satellite system clock in the unit of sec is %f',f(4))
%% Function for inequality constraint.
function [c,ceq] = inequality(x)
c = [5-x(1) x(1)-20 5-x(2) x(2)-20 5-x(3) x(3)-20 -10-x(4) x(4)-10];
ceq=[];
end


