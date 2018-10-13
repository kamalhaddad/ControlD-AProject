%% Phase portrait of accelerating pendulum 
% Code adapted from Dr Naseem Daher

clear all;
close all;
clc
b =0.20 ; 
L = 4 ; 
F1 = 1 ; 


f = @(t,Y) [Y(2); -9.81/2*sin(Y(1))-b/0.2*Y(2)-L*cos(Y(1))/0.2*F1];
%To generate the phase portrait, we need to compute the derivatives y_1_prime and y_2_prime at t=0 on
%a grid over the range of values for y_1 and y_2 we are interested in. We will plot the
%derivatives as a vector at each (y1, y2) which will show us the initial direction from each
%point. We will examine the solutions over the range -2 < y1 < 8, and -2 < y2 < 2 for y2,
%and create a grid of 20x20 points

y1 = linspace(-4,4,20);
y2 = y1;

% creates two matrices one for all the x-values on the grid, and one for
% all the y-values on the grid. Note that x and y are matrices of the same
%size and shape, in this case 20 rows and 20 columns

[x,y] = meshgrid(y1,y2);
size(x)
size(y)

%% Computing the vector field
%we need to generate two matrices, u and v, where u will contain the value of y1' at each x and y
%position, while v will contain the value of y2' at each x and y position of our grid.
%we preallocate the arrays so they have the right size and shape

u = zeros(size(x));
v = zeros(size(x));

% we can use a single loop over each element to compute the derivatives at
% each point (y1, y2)
t=0; % we want the derivatives at each point at t=0, i.e. the starting time

for i = 1:numel(x)
    Yprime = f(t,[x(i); y(i)]);
    u(i) = Yprime(1);
    v(i) = Yprime(2);
end


%% Now we use the quiver command to plot our vector field

%A quiver plot displays velocity vectors as arrows with components (u,v) at the points (x,y).
%For example, the first vector is defined by components u(1),v(1) and is displayed at the point x(1),y(1).
%quiver(x,y,u,v) plots vectors as arrows at the coordinates specified in each corresponding pair of elements in x and y. 
%The matrices x, y, u, and v must all be the same size and contain corresponding position and velocity components. 
%However, x and y can also be vectors, as explained in the next section. By default, the arrows are scaled to just not overlap, 
%but you can scale them to be longer or shorter if you want.


quiver(x,y,u,v,'r'); 
figure(gcf)
xlabel('y_1')
ylabel('y_2')
axis tight equal;


%% Plotting solutions on the vector field
%Let's plot a few solutions on the vector field. We will consider the solutions where
%y1(0)=0, and values of y2(0) = [0 0.5 1 1.5 2 2.5], in otherwords we start the pendulum at
%an angle of zero, with some angular velocity.
% 
 hold on
 for y20 = [ 2]
     [ts,ys] = ode45(f,[0,50],[-1.5;y20]);
     plot(ys(:,1),ys(:,2))
     plot(ys(:,1),ys(:,2))
     plot(ys(1,1),ys(1,2),'bo') % starting point
     plot(ys(end,1),ys(end,2),'ks') % ending point
 end
 for y20 = [2 0.5]
     [ts,ys] = ode45(f,[0,50],[-2;y20]);
     plot(ys(:,1),ys(:,2))
     plot(ys(:,1),ys(:,2))
     plot(ys(1,1),ys(1,2),'bo') % starting point
     plot(ys(end,1),ys(end,2),'ks') % ending point
 end
  for y20 = [0 .8]
     [ts,ys] = ode45(f,[0,50],[-1;y20]);
     plot(ys(:,1),ys(:,2))
     plot(ys(:,1),ys(:,2))
     plot(ys(1,1),ys(1,2),'bo') % starting point
     plot(ys(end,1),ys(end,2),'ks') % ending point
  end
  for y20 = [-0.8 2.5]
     [ts,ys] = ode45(f,[0,50],[-1.5;y20]);
     plot(ys(:,1),ys(:,2))
     plot(ys(:,1),ys(:,2))
     plot(ys(1,1),ys(1,2),'bo') % starting point
     plot(ys(end,1),ys(end,2),'ks') % ending point
 end
 hold off


%% What do these figures mean? 
% Answer: For starting points near the origin, and small velocities, the pendulum goes into
% a stable limit cycle. For others, the trajectory appears to fly off into y1 space. 
% Recall that y1 is an angle that has values from -pi to +pi. The y1 data in this case
% is not wrapped around to be in this range.

