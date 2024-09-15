Inverse Kinematics of Stewart Platform

Input:
desired_position (mm)   = [ X; Y; Z] 3x1 matrix
desired_orientation (deg)  = [rotX; rotY; totZ] 3x1 matrix
output:
6 Leg lengths  in mm (leg_lengths) 1x6 Matrix

% Define the desired position and orientation of the moving platform
desired_position = [-10; 15; 185]; % Desired position [x, y, z]
desired_orientation = [10; -15; 20]*pi/180; % Desired orientation [roll, pitch, yaw] In DEGREE

R1=130/2; %radious of the base frame
R2=125/2; %radious of the moving frame
GAMMA=20; % half angle between to adjacent legs


% Define the fixed and moving platform geometry (attachment points)
% Fixed platform attachment points (in the base frame)
B1=[R1*cosd(60-GAMMA) R1*sind(60-GAMMA) 0]'; 
B2=[R1*cosd(60+GAMMA) R1*sind(60+GAMMA) 0]'; 
B3=[-R1*cosd(GAMMA) R1*sind(GAMMA) 0]'; 
B4=[-R1*cosd(GAMMA) -R1*sind(GAMMA) 0]'; 
B5=[R1*cosd(60+GAMMA) -R1*sind(60+GAMMA) 0]'; 
B6=[R1*cosd(60-GAMMA) -R1*sind(60-GAMMA) 0]'; 

B = [B1 B2 B3 B4 B5 B6];

% Moving platform attachment points (in the moving frame)
P1=[R2*cosd(GAMMA) R2*sind(GAMMA) 0]';
P2=[-R2*sind(30-GAMMA) R2*cosd(30-GAMMA) 0]';
P3=[-R2*sind(30+GAMMA) R2*cosd(30+GAMMA) 0]';
P4=[-R2*sind(30+GAMMA) -R2*cosd(30+GAMMA) 0]';
P5=[-R2*sind(30-GAMMA) -R2*cosd(30-GAMMA) 0]';
P6=[R2*cosd(GAMMA) -R2*sind(GAMMA) 0]';

P = [P1 P2 P3 P4 P5 P6];

% Solve inverse kinematics to find the lengths of each leg
leg_lengths = inverseKinematics(desired_position, desired_orientation, B, P);

% Display the Stewart platform
[R, P_global, top_centre] = displayStewartPlatform(desired_position, desired_orientation, B, P);
Functions
function leg_lengths = inverseKinematics(desired_position, desired_orientation, B, P)
    % Convert orientation to rotation matrix
    R = Eul2rotm(desired_orientation);

    % Calculate leg lengths
    leg_lengths = zeros(6, 1);
    disp("Leg Lengths are:")
    for i = 1:6
        % Position of attachment point on moving platform in world frame
        Pi = desired_position + R * P(:, i);
        
        % Vector from base attachment point to moving platform attachment point
        L = Pi - B(:, i);
        
        % Leg length
        leg_lengths(i) = norm(L);
        disp(leg_lengths(i))
    end

end


% Utility function to convert Euler angles to rotation matrix
function R = Eul2rotm(eul)
    % Assuming Euler angles are in the order of [roll, pitch, yaw]
    R = eul2rotm([eul(1), eul(2), eul(3)],"XYZ");
end

function [R, P_global, top_centre] = displayStewartPlatform(desired_position, desired_orientation, B, P)
    % Convert orientation to rotation matrix
    R = Eul2rotm(desired_orientation);
    P_global = desired_position + R * P;

    % Plot the base attachment points
    figure;
    hold on;
    plot3(B(1,:), B(2,:), B(3,:), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    hold on
    fill3(B(1,:), B(2,:), B(3,:),'r', 'FaceAlpha', 0.2);
    text(B(1,:), B(2,:), B(3,:), {'B1', 'B2', 'B3', 'B4', 'B5', 'B6'}, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');

    % Plot the platform attachment points
    plot3(P_global(1,:), P_global(2,:), P_global(3,:), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    hold on
    fill3(P_global(1,:), P_global(2,:), P_global(3,:),'b','FaceAlpha', 0.2);
    text(P_global(1,:), P_global(2,:), P_global(3,:), {'P1', 'P2', 'P3', 'P4', 'P5', 'P6'}, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left');

    % Plot the legs
    for i = 1:6
        plot3([B(1,i), P_global(1,i)], [B(2,i), P_global(2,i)], [B(3,i), P_global(3,i)], 'k-');
    end

    %Plot Centre
    top_centre = mean(P_global,2);
    plot3(top_centre(1),top_centre(2),top_centre(3),'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    
    % Plot settings
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Stewart Platform (Inverse)');
    grid on;
    axis equal;
    view(3);
    hold off;
end
