clf; clear all; clc;
TIME_STEP = 0.001;
INIT_MV = 700;
QE = 10*360/6400;
INIT_VX = INIT_MV*cosd(QE); INIT_VZ = INIT_MV*sind(QE);
g = -9.78;
vx = zeros(size(0:TIME_STEP:5,2)+1,1); vx(1:end) = INIT_VX;
vz = zeros(size(0:TIME_STEP:5,2)+1,1); vz(1) = INIT_VZ;
x = zeros(size(0:TIME_STEP:5,2)+1,1);
z = zeros(size(0:TIME_STEP:5,2)+1,1);
i = 2;
cd = 0.159;
ro = 1.2245; D = 0.155; S = pi*D^2/4; m = 43.35;
for t=0+TIME_STEP:TIME_STEP:5    
    vx(i) = vx(i-1) + (-0.5*cd*ro*S*(vx(i-1))^2)/m * TIME_STEP; x(i) = x(i-1) + vx(i) * TIME_STEP;
    vz(i) = vz(i-1) + g * TIME_STEP;  z(i) = z(i-1) + vz(i) * TIME_STEP;
    if z(i)<0
        break;
    end
    i = i + 1;
end
plot(x(1:i))
hold on
plot(z(1:i))
x(i)