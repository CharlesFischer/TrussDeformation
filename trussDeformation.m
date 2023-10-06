%%%%%%%%%%%%%%%%%
% Please be more transparent about what's expected
%%%%%%%%%%%%%%%%%

%% Test Cases
clc; clear;
deformationScalingFactor = 34;
% forces = [0 0 100 -200 0 0];
% DOF = [0 0 1 1 0 0];
% nodes = [0 0; 1 1; 2 0];
% elements = [1 2 200000000000 .002; 2 3 200000000000 .002];
% forces = [0 0 20000 0 0 -25000 0 0];
% DOF = [0 0 1 0 1 1 0 0];
% nodes = [0 0; 40 0; 40 30; 0 30];
% elements = [1 2 29500000 1; 1 3 29500000 1; 2 3 29500000 1; 3 4 29500000 1];
forces = [0 0 0 0 0 0 0 0 20000 0 0 -50000 0 0];
DOF = [0 0 1 1 1 1 0 0 1 1 1 1 1 1];
nodes = [0 0; 1 0; 2 0; 3 0; 2.5 1; 1.5 1; .5 1];
elements = [1 2 29500000 1; 2 3 29500000 1; 3 4 29500000 1; 4 5 29500000 1; 5 6 29500000 1; 6 7 29500000 1; 1 7 29500000 1; 2 7 29500000 1; 2 6 29500000 1; 3 6 29500000 1; 3 5 29500000 1];

%% Inputs
% DOF = input("Input Degrees of Freedom: ");
% forces = input("Input forces acting on truss: ");
% nodes = input("Input node coordinates sequentially: ");
% elements = input("Input element connections: ");

%% Math
numDOF = size(DOF);
GSM = zeros(numDOF(2));
numE = size(elements);

%Generate angles
for a = 1:numE(1)
    %Generate Lengths of elements
    lengths(a) = sqrt((nodes(elements(a,2),1)-nodes(elements(a,1),1))^2+(nodes(elements(a,2),2)-nodes(elements(a,1),2))^2);
    %Create vector for element
    u = [nodes(elements(a,2),1)-nodes(elements(a,1),1) nodes(elements(a,2),2)-nodes(elements(a,1),2) 0];
    %x-axis vector
    v = [1 0 0];
    theta(a) = atan2(norm(cross(u,v)),dot(u,v)) *180/pi;
    %Find angles over pi
    if sign(u(2)) == -1
        theta(a) = 360 - theta(a);
    end
end

%Generate elementary matrices and identifier vectors
for i = 1:numE(1)
    %Generate elementary matrices
    K(:,:,i) = ((elements(i,3)*elements(i,4))/lengths(i))*Kgen(theta(i));
    %Generate identifier vectors
    Ku(i,:) = zeros([1 numDOF(2)]);
    Ku(i,elements(i,1)*2) = 1;
    Ku(i,elements(i,1)*2-1) = 1;
    Ku(i,elements(i,2)*2) = 1;
    Ku(i,elements(i,2)*2-1) = 1;
end

i = 1;
j = 1;

%Generate Global Stiffness Matrix
for a = 1:numE(1)
    for b = 1:numDOF(2)
        for c = 1:numDOF(2)
            if (Ku(a,b) == 1 && Ku(a,c) == 1) && Ku(a,b) ~= 0
                GSM(b,c) = GSM(b,c) + K(j,i,a);
                i = i + 1;
            end
        end
        i = 1;
        if Ku(a,b) == 1
            j = j + 1;
        end
    end
    j = 1;
end

l = 1;
m = 1;

%Derive reduced matrix
for b = 1:numDOF(2)
    for c = 1:numDOF(2)
        if (DOF(c) == 1 && DOF(b) == 1) && DOF(c) ~= 0
            Kred(l,m) = GSM(b,c);
            m = m + 1;
        end
    end
    m = 1;
    if DOF(b) == 1
        l = l + 1;
    end
end

i = 1;

%Shrink froce matrix to match reduced matrix
for a = 1:numDOF(2)
    if DOF(a) == 1
        forces2(i) = forces(a);
        i = i + 1;
    end
end

displacements = forces2/Kred;
i = 1;

%Expand displacements matrix to include all DOF
for a = 1:numDOF(2)
    if DOF(a) == 1
        displacements2(a) = displacements(i);
        i = i + 1;
    else
        displacements2(a) = 0;
    end
end

resultants = GSM*displacements2';

%Find stress
for a = 1:numE(1)
    stress(a) = (elements(a,3)/lengths(a))*trans(theta(a))*[displacements2(elements(a,1)*2-1) displacements2(elements(a,1)*2) displacements2(elements(a,2)*2-1) displacements2(elements(a,2)*2)]';
end

%Find strain
for a = 1:numE(1)
    strain(a) = stress(a)/elements(a,3);
end

%% Plot Section

i = 1;
j = 1;

%Add displacements to node coordinates
for a = 1:numDOF(2)
    if mod(a,2) == 1
        newxcoords(i) = nodes(i,1) + deformationScalingFactor*displacements2(a);
        i = i + 1;
    else
        newycoords(j) = nodes(j,2) + deformationScalingFactor*displacements2(a);
        j = j + 1;
    end
end

% Generate element vectors
for a = 1:numE(1)
    evectors(a,:,1) = [nodes(elements(a,1),1) nodes(elements(a,1),2)];
    evectors(a,:,2) = [nodes(elements(a,2),1) nodes(elements(a,2),2)];
    newevectors(a,:,1) = [newxcoords(elements(a,1)) newycoords(elements(a,1))];
    newevectors(a,:,2) = [newxcoords(elements(a,2)) newycoords(elements(a,2))];
end
i = 1;
j = 1;

%Shuffle the vectors to fit the syntax of plot()
for a = 1:numE(1)*2
    if mod(a,2) == 1
        ivector(a,:) = evectors(i,:,1);
        newivector(a,:) = newevectors(i,:,1);
        i = i + 1;
    else
        ivector(a,:) = evectors(j,:,2);
        newivector(a,:) = newevectors(j,:,2);
        j = j + 1;
    end
end
figure(1)
hold on
%Truss without load
plot(ivector(:,1), ivector(:,2))
%Truss with load
plot(newivector(:,1), newivector(:,2))

%% Elementary Matrix Function
function K = Kgen(x)
    c = cosd(x);
    s = sind(x);
    K =  [c^2 c*s -c^2 -c*s; c*s s^2 -c*s -s^2; -c^2 -c*s c^2 c*s; -c*s -s^2 c*s s^2];
end

%% Translation function
function T = trans(x)
    c = cosd(x);
    s = sind(x);
    T = [-c -s c s];
end