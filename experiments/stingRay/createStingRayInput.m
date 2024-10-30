% create stingray mesh: wrong: use .ipynb file
clc
clear all
close all
L = 1;
n = 2;
l = L/n;
y = [];
x = [];

for i = 0:1:n
    y_temp =  (-i*l/2:l:i*l/2);
     y = [y, y_temp];

     x_temp = (sqrt(3)/2)*(i-n)*l .*ones(1,size(y_temp,2));
     x = [x, x_temp];
end

% mirror
y_mirror = [];
x_mirror = [];
for i = 0:1:n-1
    y_temp =  (-i*l/2:l:i*l/2);
     y_mirror = [y_mirror, y_temp];

     x_temp = (sqrt(3)/2)*(n-i)*l .*ones(1,size(y_temp,2));
     x_mirror = [x_mirror, x_temp];
end
% 
x_total = [x, x_mirror];
y_total = [y, y_mirror];

% x_total = [x];
% y_total = [y];

figure()
scatter(x_total,y_total);

DT=delaunayTriangulation(x_total(:),y_total(:));
figure()
triplot(DT);
axis equal;

% Extract the elements and nodes
Face_Nodes = DT.ConnectivityList;
Nodes = DT.Points;
Nodes = [Nodes, zeros(size(Nodes,1),1)];

% DT=delaunay(x_total(:),y_total(:));
% Face_Nodes = DT;



% check face normals
face_normals = faceNormal(DT);

filename = 'stingray_n2_matlab.txt';
fid = fopen(filename,'w');
if fid ~= -1
    fprintf(fid,'*shellNodes');
    fclose(fid);
end
writematrix(Nodes,filename, 'WriteMode','append')
fid = fopen(filename, 'at');
if fid ~= -1
    fprintf(fid,'*FaceNodes');
    fclose(fid);
end
writematrix(Face_Nodes,filename, 'WriteMode','append')

%% theta
T = 1;
dt = 0.01;
time_arr = 0:dt:T;
curr_time = 0;
flap_freq = 4;
for i = 1:size(time_arr,2)
    theta(i) = 5*pi/12*(-cos(2*flap_freq*pi*curr_time) + 1);
    curr_time = curr_time+dt;
end
figure()
plot(time_arr,theta)


