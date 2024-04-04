% delaunayTriangulation
h0=1;
[x,y]=meshgrid(1:h0:15,1:h0*sqrt(3)/2:15);
x(2:2:end,:)=x(2:2:end,:)+h0/2; % Shift even rows
DT=delaunayTriangulation(x(:),y(:));
triplot(DT);