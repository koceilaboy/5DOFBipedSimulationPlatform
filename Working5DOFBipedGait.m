clear all
clc
clf
close all

%% Simulation Parameters and Initial conditions



global x0 x1 x2 x3 x4 x5 y0 y1 y2 y3 y4 y5 h z ep xd1 xd3 xd7 xd9 DESIREDPOS
global K K_SLIP m1 m2 m3 m4 m g l b k l1 l2 l3 l1d l1u l2d l2u l3d I2 I3 I4 b1 b2 b3 T dt t x y xd yd KK
global previous_r_position previous_theta_position prev_x0 prev_y0 prev_x4 prev_y4 foot last_ep epi KP KD KI desired_hip_position desired_hip_pos Max_Step_Length

x0(1) = 0;
y0(1) = 0;
x2(1) = 0;
y2(1) = 40;
x4(1) = 0;
y4(1) = 0;



K  = [1 1 1 1 1]'* 50; % Controller gains
K_SLIP=[0;1];

m1 = 1;
m2 = 1;
m3 = 1;
m4 = 1;

m= m1+m2+m3+m4;
g=-9.8;
l=40;
Ts= [0 20]
b=1/60;
KK=2000;

l1 = 20; %cm
l2 = 20; %cm
l3 = 20; %cm

% computing radius of rotation of each joint relative to the center of mass

l1d= l1/2;
l1u= l1d;
l2d= l2/2;
l2u= l2d;
l3d= l3/2;

% Computing the moment of inertia of each joint
I2 = l1d* m2;
I3 = l2d* m3;
I4 = l3d* m4; % double check the moment of inertia of each joint

% Friction Coefficient between joints
b1= 0.2
b2= 0.2
b3= 0.2

T  = 2;
dt = 0.005;

foot = 0; % Initial Condition

%         x1  x2  x3   x4   x5   x6
if foot == 0
    x(:,1) = [0 0 0 0 0 0 0 0 0 0 0 0] * pi/180;
elseif foot ==1 
    x(:,1) = [0 0 0 0 0 0 0 0 0 0 0 0] * pi/180;
end

xd = [ 0 0 0 0 0 0 0 0 0 0 0 0 ]'*pi/180; % Set points for controller

yd= [40;0; 4*pi/180;0];



previous_r_position =40;
previous_theta_position = 0 *pi/180; % This is the angle between the ZMP and the CM point 
prev_x0 = 0;
prev_y0 = 0;
prev_x4 = 0;
prev_y4 = 0;

h=0;



%Position Controller
last_ep = 0;
epi = 0
KP = 10;
KD = 1000; 
KI = 2;


Max_Step_Length = 5; % 2*(l1+l2)*tan(5*pi/180); % More complex formula required
ANGLE = atan(5/40);

last_step = 0;

i= 1
global k 
k=1 




desired_hip_position = 43;



%% Step Planning
t= 0:dt: i*T - dt; 
while ( x2(end) ~= desired_hip_position)
    
    if foot == 0
        DIST = sqrt((x2(end) - x0(end))^2 + (y2(end) - y0(end))^2 )
        ANGLE_OFFSET = atan((x2(end) - x0(end))/(y2(end) - y0(end)))
    elseif foot == 1
        DIST = sqrt((x2(end) - x4(end))^2 + (y2(end) - y4(end))^2 )
        ANGLE_OFFSET = atan((x2(end) - x4(end))/(y2(end) - y4(end)))
    end
    
    Max_Step_Length = sqrt((l1+l2)^2 + (DIST)^2 - 2*(l1+l2)*DIST*cos(ANGLE-ANGLE_OFFSET))
    
    if Max_Step_Length >= 9.8
        Max_Step_Length = 9.8;
    elseif Max_Step_Length <= -9.8
        Max_Step_Length = -9.8; 
    end
        
    
    
    ERROR = desired_hip_position - x2(end);
    if ERROR >= Max_Step_Length
        desired_pos = x2(end) + Max_Step_Length
    elseif ERROR < Max_Step_Length
        desired_pos = x2(end) + ERROR
        last_step = 1;
    end
    
    step = walk(desired_pos, foot, i, k)
    
    

    if ERROR >= Max_Step_Length
        if step && foot == 1
            foot = 0;
            i=i+1;
            
        elseif step && foot == 0
            foot = 1;
            i=i+1;
        end
    elseif last_step && step
        break
        
    elseif step ==0
        break
    end
end

%% Plotting    
% x=x'*180/pi;
% 
% figure(1)
% plot(t,x(:,1))
% title('phi1')
% 
% 
% figure(2)
% plot(t,x(:,2))
% title('phi1 dot')
% 
% figure(3)
% plot(t,x(:,3))
% title('phi2')
% 
% figure(4)
% plot(t,x(:,4))
% title('phi2 dot')
% 
% figure(5)
% plot(t,x(:,7))
% title('phi4')
% 
% figure(6)
% plot(t,x(:,8))
% title('phi4 dot')
% 
% figure(7)
% plot(t,x(:,9))
% title('phi5')
% 
% figure(8)
% plot(t,x(:,10))
% title('phi5 dot')
% 
% figure(9)
% plot(t,x(:,11))
% title('phi6')
% 
% figure(10)
% plot(t,x(:,12))
% title('phi6 dot')
% 
% figure(11)
% plot(1:k, y(1,:))
% title('x displacement over time' )
% xlabel('time (s)')
% figure(12)
% plot(1:k, y(2,:))
% title('x velocity over time' )
% xlabel('time (s)')
% 
% figure(13)
% plot(1:k, y(3,:)*180/pi)
% title('angluar displacement over time' )
% xlabel('time (s)')
% figure(14)
% plot(1:k, y(4,:)*180/pi)
% title('angluar velocity over time' )
% xlabel('time (s)')
% 
% figure(15)
% plot(1:k, F_SLIP(1,:))
% title('Normal Force' )
% xlabel('time (s)')
% 
% figure(16)
% plot(1:k, F_SLIP(2,:))
% title('Torque applied by the ground on the foot')
% xlabel('time (s)')


%% Biped Motion generation
t= 0:dt: i*T - dt; 
% generating images in 2D
figure
for i=1:14:length(t)-1
    
    
hold off
plot3([x3(i) x4(i)], [z(i) z(i)], [y3(i) y4(i)],'ro' , [x2(i) x5(i)],[z(i) z(i)],[y2(i) y5(i)],'co',[x0(i) x1(i)],[z(i) z(i)],[y0(i) y1(i)], 'bo' ,[x0(i) x1(i)], [z(i) z(i)],[y0(i) y1(i)],'k',[x1(i) x2(i)],[z(i) z(i)],[y1(i) y2(i)],'k',[x2(i) x3(i)],[z(i) z(i)],[y2(i) y3(i)],'k', [x3(i) x4(i)],[z(i) z(i)],[y3(i) y4(i)],'k',[x2(i) x5(i)],[z(i) z(i)],[y2(i) y5(i)],  'k','LineWidth',2.2)                
title('Motion of 5DOF Bipedal Robot')


zlabel('z')
ylabel('y')
xlabel('x')
xlim([-10 50])

%  axis([-30 30 -2 60]);
 grid
 hold on
 plot3(x0([1:i]),z([1:i]),y0([1:i]),'.b','LineWidth',2)
 plot3(x1([1:i]),z([1:i]),y1([1:i]),'.b','LineWidth',2)
 plot3(x2([1:i]),z([1:i]),y2([1:i]),'.c','LineWidth',2)
 plot3(x3([1:i]),z([1:i]),y3([1:i]),'.r','LineWidth',2)
 plot3(x4([1:i]),z([1:i]),y4([1:i]),'.r','LineWidth',2)
 
 %legend('foot 1','hip and upperbody joints', 'foot 0')
 
 MM(i)=getframe(gcf);
 
 
end
drawnow;

figure()
hold on
plot(DESIREDPOS)
plot(x2)
legend('desired hip position', 'actual hip position')



%% Step Action

function f = walk(desired_pos, foot, i, k)
    global x0 x1 x2 x3 x4 x5 y0 y1 y2 y3 y4 y5 h y k KK ep xd1 xd3 xd7 xd9 DESIREDPOS
    global K K_SLIP m1 m2 m3 m4 m g l b k l1 l2 l3 l1d l1u l2d l2u l3d I2 I3 I4 b1 b2 b3 T dt t x y z xd yd
    global previous_r_position previous_theta_position prev_x0 prev_y0 prev_x4 prev_y4 foot last_ep epi KP KD KI desired_hip_position desired_hip_pos Max_Step_Length

    for k = k: 1: i*T/dt - 1
        

        %% Forward Kinematics
        if foot == 0
            z(k) = 0;
            x0(k)= prev_x0;
            y0(k)= prev_y0;
            x1(k)= x0(k) + l1 * sin(-x(1,k));
            y1(k)= y0(k) + l1 * cos(-x(1,k));
            x2(k)= x1(k) - l2*sin(x(3,k));
            y2(k)= y1(k) + l2*cos(x(3,k));
            x3(k)= x2(k) - l2*sin(x(9,k));
            y3(k)= y2(k) - l2*cos(x(9,k));

            x4(k)= x3(k) - l1*sin(-x(7,k));
            y4(k)= y3(k) - l1*cos(-x(7,k));
            prev_x4= x3(k) - l1*sin(-x(7,k));
            prev_y4= y3(k) - l1*cos(-x(7,k));

            x5(k)= x2(k) + l3*sin(x(11,k));
            y5(k)= y2(k) + l3*cos(x(11,k));

            
        elseif foot == 1       

            z(k) = 0;
            x4(k)= prev_x4;
            y4(k)= prev_y4;
            x3(k)= x4(k) + l1*sin(-x(7,k));
            y3(k)= y4(k) + l1*cos(-x(7,k));
            x2(k)= x3(k) + l2*sin(x(9,k));
            y2(k)= y3(k) + l2*cos(x(9,k));
            x1(k)= x2(k) + l2 * sin(x(3,k));
            y1(k)= y2(k) - l2 * cos(x(3,k));
            x0(k)= x1(k) - l1*sin(-x(1,k));
            y0(k)= y1(k) - l1*cos(-x(1,k)) ;
            x5(k)= x2(k) + l3*sin(x(11,k));
            y5(k)= y2(k) + l3*cos(x(11,k));  

            prev_x0 = x0(k);
            prev_y0 = y0(k);
        end

        %% Parameter computation
        c21= cos(x(3,k) - x(1,k));
        c41= cos(x(7,k) - x(1,k));
        c42= cos(x(7,k) - x(3,k));
        c51= cos(x(9,k) - x(1,k));
        c52= cos(x(9,k) - x(3,k));
        c54= cos(x(9,k) - x(7,k));
        c61= cos(x(11,k) - x(1,k));
        c62= cos(x(11,k) - x(3,k));

        s21= sin(x(3,k) - x(1,k));
        s41= sin(x(7,k) - x(1,k));
        s42= sin(x(7,k) - x(3,k));
        s51= sin(x(9,k) - x(1,k));
        s52= sin(x(9,k) - x(3,k));
        s54= sin(x(9,k) - x(7,k));
        s61= sin(x(11,k) - x(1,k));
        s62= sin(x(11,k) - x(3,k));

        s1 = sin(x(1,k));
        s2 = sin(x(3,k));
        s4 = sin(7);
        s5 = sin(x(9,k));
        s6 = sin(x(11,k));


        % M Matrix
        M(:,1)= [m2*l1d^2 + I2 + (2*m3 + m2 + m4)*l1^2;
                ((m2 + m3 + m4)*l1*l2 + m3*l1*l2d)*c21;
                -m2*l1*l1u*c41;
                -(m3*l1*l2u + m2*l1*l2^2)*c51;
                m4*l1*l3d*c61                            ];

        M(:,2)= [((m2 + m3 + m4)*l1*l2 + m3*l1*l2d)*c21;
                m3*l2^2 + I3 + (m3 + m2 + m4)*l2^2;
                -m2*l2*l1u*c42;
                -(m3*l2*l2u + m2*l2^2)*c52;
                m4*l2*l3d*c62                            ];

        M(:,3)=[-m2*l1*l1u*c41;
                -m2*l2*l1u*c42;
                m2*l1u^2 + I2;
                m2*l2*l1u*c54;
                0                                       ];

        M(:,4)= [-(m3*l1*l2u + m2*l1*l2)*c51;
                -(m3*l2*l2u + m2*l2^2)*c52;
                m2*l2*l1u*c54;
                m3*l2u^2 + I3 + m2*l2^2
                0                                       ];

        M(:,5)= [m4*l1*l3d*c61;
                m4*l2*l3d*c62
                0
                0
                I4 + m4*l3d^2                           ];

        % C Matrix

        C(:,1)= [b1 + b2;
                -b2 + ((m2 + m3 + m4)*l1*l2 + m3*l1*l2d)*s21*x(2,k);
                -m2*l1*l1u*s41*x(2,k);
                -(m2*l1*l2 + m3*l1*l2u)*s51*x(2,k)
                m4*l1*l3d*s61*x(2,k)];

        C(:,2)= [-b2 + ((m2 + m3 + m4)*l1*l2 + m3*l1*l2d)*s21*x(4,k);
                b2 + b3;
                -m2*l2*l1u*s42*x(4,k);
                -(m2*l2^2 + m3*l2*l2u)*s52*x(4,k);
                -b3 + m4*l2*l3d*s62*x(4,k)];

        C(:,3)= [m2*l1*l1u*s41*x(8,k);
                m2*l2*l1u*s42*x(8,k);
                b2;
                -b2 + m2*l2*l1u*s54*x(8,k);
                0];

        C(:,4)= [(m2*l1*l2 + m3*l1*l2u)*s51*x(10,k);
                (m2*l2^2 + m3*l2*l2u)*s52*x(10,k);
                -b2-m2*l2*l1u*s54*x(10,k);
                b2+b3
                -b3];

        C(:,5)= [-m4*l1*l3d*s61*x(12,k);
                -b3 - m4*l2*l3d*s62*x(12,k)
                0
                -b3
                2*b3];

        % G Matrix
        G = g*[(m2*l1d + (2*m3 + m4 + m2)*l1)*s1;
                (m3*l2d + (m3 + m4 + m2)*l2)*s2
                -m2*l1u*s4;
                -(m3*l2u* + m2*l2)*s5
                m4*l3d*s6];

        % D Matrix
        D = [1, 1, 0, 0, 0;
            0, -1, -1, 0, 0;
            0, 0, 0, 1, 0;
            0, 0, 0, -1, -1;
            0, 0, 1, 0, 1];

        %% PD Position Control
        DESIREDPOS(k) = desired_pos; % For plotting
        
        
        ep(k) = desired_pos - x2(k);
        epd = ep(k) - last_ep;
        epi = epi + ep(k)*dt;
        
        
        Input = (KP*ep(k) + KD*epd + KI*epi);
        
%         if Input >= 5
%             Input = 5
%         elseif Input <= -5
%             Input = -5
%         end 
        
        yd(3) = atan((Input)/y2(k));

        % if input is greater than 7 deg
        if yd(3) >= 7*pi/180
            yd(3) = 7*pi/180
        elseif yd(3) <= -7*pi/180
            yd(3) = -7*pi/180
        end    
        
        if ep(k) <= 0.14 && ep(k) >= -0.14
            f=1;
        else 
            f=0;
        end

        %% Spring Loaded Inverted Pendulum 
        if foot == 0
            x_r(k) = sqrt( (x2(k) - x0(k))^2 + (y2(k)- y0(k))^2) ;
            x_r_velocity(k) = (x_r(k) - previous_r_position)/dt;
            previous_r_position = x_r(k);

            x_theta(k)= atan((x2(k)- x0(k))/(y2(k) - y0(k)));
            x_theta_velocity(k) = (x_theta(k) - previous_theta_position)/dt;
            previous_theta_position=x_theta(k);
        elseif foot == 1
            x_r(k) = sqrt( (x2(k) - x4(k))^2 + (y2(k)- y4(k))^2) ;
            x_r_velocity(k) = (x_r(k) - previous_r_position)/dt;
            previous_r_position = x_r(k);

            x_theta(k)= atan2((x2(k)- x4(k)),(y2(k) - y4(k)));
            x_theta_velocity(k) = (x_theta(k) - previous_theta_position)/dt;
            previous_theta_position= x_theta(k);
        end

        %% ISLP Parameters  
        y(:,k)= [x_r(k); x_r_velocity(k); x_theta(k); x_theta_velocity(k)];

        G_SLIP= [ -y(1,k)*(y(4,k))^2    +    g*cos(y(3,k))   -   (KK/m)*(l-y(1,k)) +   (b/m)*y(2,k);
              (-g*sin(y(3,k)) + 2*(y(2,k)*y(4,k)))/y(1,k);
            ];
        D_SLIP = [1, 0;0, 1/(m*(y(1,k))^2)];

        %% ISLP Errors 
        e1 = y(3,k)-yd(3)


        Y(:,k)=[y(1,k)-yd(1) e1]'; % x is the state error, X the states of the system and, xd the setponts
        Y_dots = [y(2,k) y(4,k)]'; % these are the angular velocities of each joint in the robotic arm

        %% ISLP ZMP Controller 
        u(:,k)= - K_SLIP.*Y_dots - (10000*K_SLIP.*Y(:,k));
        F_SLIP(:,k)= inv(D_SLIP)*(G_SLIP+ u(:,k) );

        %% Otput states of the ISLP Model
        Y_dots = -G_SLIP + D_SLIP*F_SLIP(:,k);

        rdot(2,1)  = Y_dots(1);
        rdot(4,1)  = Y_dots(2);
        rdot(1,1)  = y(2,k) + Y_dots(1)*dt;
        rdot(3,1)  = y(4,k) + Y_dots(2)*dt;

        r = y(1,k) + rdot(1,1)*dt ;
        if r >= (l1+l1)
           r= l1+l2; 
        end
        theta = y(3,k) + rdot(3,1)*dt;  
        
        %% Computing Inverse Kinematics
    if foot == 0
        if theta >= 0
            xd1(k) = -acos ((1/(2*l1))*( r*cos(theta)  +  (1/r)*(   sqrt(-l1^4 + 2*(l1^2)*(l2^2) - (l2^4) + 2*((l1^2) + (l2^2))*(r^2) - (r^4))*sin(theta)  +  ((l1^2)-(l2^2))*cos(theta)  )));
            xd3(k) = -acos ((1/(2*l2))*( r*cos(theta)  -  (1/r)*(   sqrt(-l1^4 + 2*(l1^2)*(l2^2) - (l2^4) + 2*((l1^2) + (l2^2))*(r^2) - (r^4))*sin(theta)  +  ((l1^2)-(l2^2))*cos(theta)  )));
        elseif theta <= 0
            xd1(k) = acos ((1/(2*l1))*( r*cos(theta)  +  (1/r)*(   sqrt(-l1^4 + 2*(l1^2)*(l2^2) - (l2^4) + 2*((l1^2) + (l2^2))*(r^2) - (r^4))*sin(theta)  +  ((l1^2)-(l2^2))*cos(theta)  )));
            xd3(k) = acos ((1/(2*l2))*( r*cos(theta)  -  (1/r)*(   sqrt(-l1^4 + 2*(l1^2)*(l2^2) - (l2^4) + 2*((l1^2) + (l2^2))*(r^2) - (r^4))*sin(theta)  +  ((l1^2)-(l2^2))*cos(theta)  )));
        end

        xd9(k) =  xd3(k)-0.35;
        xd7(k) =  -xd1(k);

        if (x(1,k) <= xd1(k)+0.2) && (x(3,k) <= xd3(k)+0.2)
            xd9(k) =  xd3(k);
            xd7(k) =  -xd1(k);
        end
        

        
    elseif foot == 1
        if theta >= 0
            xd7(k) = -acos ((1/(2*l1))*( r*cos(theta)  +  (1/r)*(   sqrt(-l1^4 + 2*(l1^2)*(l2^2) - (l2^4) + 2*((l1^2) + (l2^2))*(r^2) - (r^4))*sin(theta)  +  ((l1^2)-(l2^2))*cos(theta)  )));
            xd9(k) = acos ((1/(2*l2))*( r*cos(theta)  -  (1/r)*(   sqrt(-l1^4 + 2*(l1^2)*(l2^2) - (l2^4) + 2*((l1^2) + (l2^2))*(r^2) - (r^4))*sin(theta)  +  ((l1^2)-(l2^2))*cos(theta)  )));
        elseif theta <= 0
            xd7(k) = acos ((1/(2*l1))*( r*cos(theta)  +  (1/r)*(   sqrt(-l1^4 + 2*(l1^2)*(l2^2) - (l2^4) + 2*((l1^2) + (l2^2))*(r^2) - (r^4))*sin(theta)  +  ((l1^2)-(l2^2))*cos(theta)  )));
            xd9(k) = -acos ((1/(2*l2))*( r*cos(theta)  -  (1/r)*(   sqrt(-l1^4 + 2*(l1^2)*(l2^2) - (l2^4) + 2*((l1^2) + (l2^2))*(r^2) - (r^4))*sin(theta)  +  ((l1^2)-(l2^2))*cos(theta)  )));
        end



        xd3(k) =  xd9(k)+1;
        xd1(k) =  -xd7(k);

            
        if (x(7,k) <= xd7(k)+0.2) && (x(9,k) <= xd9(k)+0.2)
            xd3(k) =  xd9(k);
            xd1(k) =  -xd7(k);
        end    
            
        
    end
        

        
    %     
    
        %% State error
        X=[x(1,k)-xd1(k) x(3,k)-xd3(k) x(7,k)-xd7(k) x(9,k)-xd9(k) x(11,k)-xd(11)]'; % x is the state error, X the states of the system and, xd the setponts
        X_dots = [x(2,k) x(4,k) x(8,k) x(10,k) x(12,k)]'; % these are the angular velocities of each joint in the robotic arm

        %% Joint Controller
        F= inv(D)*inv(M)*(C*X_dots + G - X + (M*(-K.*X_dots - (10*K.*X))))  ; % feedback linearization controller to stabilize the system at the setpoint

        %% System Dynamics
        X_dots =  inv(M)*(-C*X_dots - G) + D*F; % here we are computing the new states of the system

        %Continuous time representation
        xdot(1,1)  = x(2,k);
        xdot(2,1)  = X_dots(1);
        xdot(3,1)  = x(4,k);
        xdot(4,1)  = X_dots(2);
        xdot(7,1)  = x(8,k);
        xdot(8,1)  = X_dots(3);
        xdot(9,1)  = x(10,k);
        xdot(10,1) = X_dots(4);
        xdot(11,1) = x(12,k);
        xdot(12,1) = X_dots(5);

        %Converting Continuous time to discrete time
        x(:,k+1) =  x(:,k) + xdot(:,1)*dt;

    end
    
    previous_r_position = 0
end

