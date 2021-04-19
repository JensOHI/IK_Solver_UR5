function q = ikSolverUR5All(pos,eul,qPrevious)
%This solution is based on Kinematics of a UR5 by Rasmus Skovgaard Andersen
%http://rasmusan.blog.aau.dk/files/ur5_kinematics.pdf

%The input pos is a 3x1 vector with x-, y-, z-coordinate for the desired position of the end-effector.
%The input eul is a 3x1 vector with the desired orientation of the end-effector in euler angles (Z,Y,X).
%The input q is the previous configuration (joint angles) of the robot.


%DH parameters for the UR5 robot taken from here: https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics
a = [0 -0.425 -0.39225 0 0 0];
d = [0.089159 0 0 0.10915 0.09465 0.0823];
alpha = [pi/2 0 0 pi/2 -pi/2 0];

% Finding end effector related to base.
T06 = eye(4); 
T06(1:3,1:3) = eul2rotm(eul); 
T06(1:3,4) = pos;

P06 = T06(1:3,4);



% ------------------------------ Theta 1 ------------------------------ 
%Theta1 = 2 solutions, isn't depended on other angles.

P05 = T06 * [0;0;-d(6);1];
theta1_ = [];
theta1P = atan2(P05(2),P05(1)) + pi/2;
theta1M = theta1P;

if P05(1) ~= 0 || P05(2) ~= 0
   theta1P = theta1P + real(acos(complex(d(4)/sqrt((P05(1))^2+(P05(2))^2))));
   theta1M = theta1M - real(acos(complex(d(4)/sqrt((P05(1))^2+(P05(2))^2))));
end
theta1_ = [theta1P;theta1M];


% ------------------------------ Theta 5 ------------------------------ 
% Theta5 = 2 solutions, is depended on theta1 which also have 2 solutions.
% # of solutions = 2*2=4

theta5_ = zeros(4,1);
idx = 1;
for i = 1:length(theta1_)
    acosValue = (P06(1)*sin(theta1_(i)) - P06(2)*cos(theta1_(i)) - d(4))/d(6);
    if acosValue > 1
        error('Theta5 can not be detemined. Value inside acos is above 1 and the solution is therefore not valied.')
    end
    for sign = [1 -1]
        theta5_(idx) = sign * acos(acosValue);
        idx = idx + 1;
    end
end


% ------------------------------ Theta 6 ------------------------------ 
%Theta6 = 1 solution, is depended on theta1 and theta5 which each have 2 solutions.
% # of solutions = 1*2*2 = 4

T60 = inv(T06);

X60 = T60(1:3,1);
Y60 = T60(1:3,2);

theta6_ = zeros(4,1);
t5 = [1 3];
idx = 1;
for t1 = 1:length(theta1_)
    theta6_(idx) = calculateTheta6(X60, Y60, theta1_(t1), theta5_(t5(t1)));
    theta6_(idx+1) = calculateTheta6(X60, Y60, theta1_(t1), theta5_(t5(t1)+1));
    idx = idx + 2;
end

% ------------------------------ Theta 3 ------------------------------ 
%We want to find frame T14 (to get P14). We can do this by going from frame 1 to base,
%then go to end effector, frame 5 and at last to frame 4. We can find frame
%T01, T06, T56 and T45. We can then inverse most of these and multiply them
%together.

%Theta3 = 2 solutions, but P14 is depended on theta1, theta5 and theta6.
% # of solutions = 2*2*2*1 = 8 solutions

theta6Index = 1:4;
t6 = 1;
theta3_ = zeros(8,1);
P14_ = zeros(8,3);
T14_ = zeros(4,4,8);
idx = 1;
for t1 = 1:length(theta1_)
    for sign = [1 -1]
        [theta3, P14, T14] = calculateTheta3(T06, alpha, a, d, theta1_(t1), theta5_(t5(t1)), theta6_(t5(t1)));
        theta3_(idx) = sign * theta3;
        P14_(idx,:) = P14';
        T14_(:,:,idx) = T14;
        idx = idx + 1;
        
        [theta3, P14, T14] = calculateTheta3(T06, alpha, a, d, theta1_(t1), theta5_(t5(t1)+1), theta6_(t5(t1)+1));
        theta3_(idx) = sign * theta3;
        P14_(idx,:) = P14';
        T14_(:,:,idx) = T14;
        idx = idx + 1;
    end     
    t6 = t6 + 1;
end

%Reagrange so lists match.
rearangeIdx = [1 3 2 4 5 7 6 8];
theta3_Copy = theta3_;
P14_Copy = P14_;
T14_Copy = T14_;
for i = 1:length(rearangeIdx)
    theta3_Copy(i) = theta3_(rearangeIdx(i));
    P14_Copy(i,:) = P14_(rearangeIdx(i),:);
    T14_Copy(:,:,i) = T14_(:,:,rearangeIdx(i));
end
theta3_ = theta3_Copy;
P14_ = P14_Copy;
T14_ = T14_Copy;

% ------------------------------ Theta 2 ------------------------------ 
%Theta2 = 1 solution, depends on Theta1, Theta3, Theta5 and theta6
% # of solutions 1 * 2 * 2 * 2 * 1 = 8 

theta2_ = zeros(8,1);
idx = 1;
for t3 = 1:length(theta3_)
    P14_xz_length = norm([P14_(t3,1) P14_(t3,3)]);
    theta2 = atan2(-P14_(t3,3), -P14_(t3,1)) - asin(-a(3) * sin(theta3_(t3))/P14_xz_length);
    theta2_(idx) = theta2;
    idx = idx + 1;
end

% ------------------------------ Theta 4 ------------------------------ 
%Since we can't get the transformation matrix T43, because it requires
%theta4, we can go from frame T23 to T12 and then up to T34 using T14

% Theta4 = 1 solution, depends on Theta1, Theta2, Theta3 and Theta5
% # of solutions 1 * 1 * 2 * 2 * 2 * 1 = 8 

theta4_ = zeros(8,1);
idx = 1;
for t2t3 = 1:length(theta2_)
    T12 = DH2tform(alpha(1),a(1),d(2),theta2_(t2t3));
    T21 = inv(T12);

    T23 = DH2tform(alpha(2),a(2),d(3),theta3_(t2t3));
    T32 = inv(T23);

    T34 = T32*T21*T14_(:,:,t2t3);
    X34 = T34(1:3,1);

    theta4 = atan2(X34(2),X34(1));
    
    theta4_(idx) = theta4;
    idx = idx + 1;
end




solutions = generatePossibleSolutions(theta1_,theta2_,theta3_,theta4_,theta5_,theta6_);
solution = closetSolution(solutions, qPrevious);

q = solution';
end

function theta6 = calculateTheta6(X60, Y60, theta1, theta5)
theta6 = 0;
if sin(theta5) ~= 0
    leftNumerator = -X60(2)*sin(theta1) + Y60(2)*cos(theta1);
    rightNumerator = X60(1)*sin(theta1) - Y60(1)*cos(theta1);
    denominator = sin(theta5);
    theta6 = atan2(leftNumerator/denominator, rightNumerator/denominator);
end
end

function [P14, T14] = calculateP14(T06, alpha, a, d, theta1, theta5, theta6)

T01 = DH2tform(0, 0, d(1), theta1);
T10 = inv(T01);

T45 = DH2tform(alpha(4), a(4), d(5), theta5);
T54 = inv(T45);


T56 = DH2tform(alpha(5), a(5), d(6), theta6);
T65 = inv(T56);


T14 = T10*T06*T65*T54;
P14 = T14(1:3,4);
end

function [theta3, P14, T14] = calculateTheta3(T06, alpha, a, d, theta1, theta5, theta6)
[P14, T14] = calculateP14(T06, alpha, a, d, theta1, theta5, theta6);

P14_xz_length = norm([P14(1) P14(3)]);

%Before calculating theta3 we have to check conditions mentioned in the
%paper. The conditions are
conditions = [abs(a(2)-a(3));
              abs(a(2)+a(3))];

if P14_xz_length > conditions(1) && P14_xz_length < conditions(2)
   theta3 = acos((P14_xz_length^2 - a(2)^2 -a(3)^2)/(2*a(2)*a(3)));
else
    error('Theta3 can not be determined. Conditions are not uphold. P14_xz_length is exceding the condidtions.')
end
end

function Transform = DH2tform(alpha, a, d, theta)
Transform = eye(4); 

alpha_mi = alpha; 
a_mi = a; 
d_i = d; 
theta_i = theta; 

% Row 1 
Transform(1,1) = cos(theta_i);
Transform(1, 2) = -sin(theta_i); 
Transform(1, 3) = 0; 
Transform(1, 4) = a_mi; 

% Row 2
Transform(2, 1) = sin(theta_i)*cos(alpha_mi); 
Transform(2, 2) = cos(theta_i) *cos(alpha_mi); 
Transform(2, 3) = -sin(alpha_mi); 
Transform(2, 4) = -sin(alpha_mi)*d_i; 

% Row 3 
Transform(3, 1) = sin(theta_i)*sin(alpha_mi); 
Transform(3, 2) = cos(theta_i)*sin(alpha_mi); 
Transform(3, 3) = cos(alpha_mi); 
Transform(3, 4) = cos(alpha_mi) *d_i; 
end

function solutions = generatePossibleSolutions(theta1,theta2,theta3,theta4,theta5,theta6)

solutions = [theta1(1) theta2(1) theta3(1) theta4(1) theta5(1) theta6(1);
             theta1(1) theta2(3) theta3(3) theta4(3) theta5(2) theta6(2);
             theta1(2) theta2(5) theta3(5) theta4(5) theta5(3) theta6(3);
             theta1(2) theta2(7) theta3(7) theta4(7) theta5(4) theta6(4);
             
             theta1(1) theta2(2) theta3(2) theta4(2) theta5(1) theta6(1);
             theta1(1) theta2(4) theta3(4) theta4(4) theta5(2) theta6(2);
             theta1(2) theta2(6) theta3(6) theta4(6) theta5(3) theta6(3);
             theta1(2) theta2(8) theta3(8) theta4(8) theta5(4) theta6(4)];
end

function solution = closetSolution(solutions, q)
weights = [6 5 4 3 2 1];
bestConfigurationDistance = inf;
solution = solutions(1,:);
for i = 1:size(solutions,1)
    configurationDistance = sum(((solutions(i,:) - q') .* weights).^2);
    if configurationDistance < bestConfigurationDistance
        bestConfigurationDistance = configurationDistance;
        solution = solutions(i,:);
    end
end
end

