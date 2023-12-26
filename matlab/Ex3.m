clear;

%% initializing
alpha = pi/6;
beta = 0.8;

A = [cos(alpha) sin(alpha);
    -sin(alpha) cos(alpha)];

A = A*beta;

H = [cos(pi/3) sin(pi/3);
    -cos(pi/3) -sin(pi/3);
     sin(pi/3) -cos(pi/3);
    -sin(pi/3) -cos(pi/3)];

h = [2;1;2;5];
P = Polyhedron(H,h);

verbose = false;

%% Plotting of the transform

if verbose
    P.plot();
    hold on;
    
    % Define the ranges for x and y
    x_range = -10:4;
    y_range = -2:8;
    
    % Create a grid of all combinations of x and y values
    [x, y] = meshgrid(x_range, y_range);
    
    % Concatenate x and y to create a 2D matrix
    matrix_2D = [x(:), y(:)];
    
    
    for i=1:size(matrix_2D,1)
        x = matrix_2D(i,1);
        y = matrix_2D(i,2);
        g = A*[x;y];
        
        direction = [g(1)-x,g(2)-y];
        
        quiver(x,y,direction(1),direction(2));
        
        scatter(g(1),g(2), 'filled','blue');
        scatter(x,y, 'filled','green');
        plot(preOmega,'alpha',0.1);
        preOmega = Polyhedron(H*A,h);
    end
end
%% Computing largest invariant Set 
omega = Polyhedron(H,h);
preOmega = Polyhedron(omega.A*A,h);
omega1 = intersect(preOmega,omega);
plot([omega,preOmega,omega1]);

omega = omega1;
preOmega = Polyhedron(omega.A*A,h);
omega1 = intersect(preOmega,omega);
plot([omega,preOmega,omega1]);
%{
while 1
    preOmega = Polyhedron(omega.A*A,h);
    omega1 = intersect(preOmega,omega);
    if omega1 == omega
        break;
    end
    
    omega = omega1;

end

plot(omega);
%}