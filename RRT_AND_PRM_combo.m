%% 1. TO PLOT F SHAPE (MAP)

map = binaryOccupancyMap(100,80, 1);
occ = zeros(80, 100);
occ(1,:) = 1;
occ(:,1) = 1;
occ(:,100)=1;
occ(40,[1:30]) = 1;
occ(40,[70:100])=1;
setOccupancy(map, occ)
figure
show(map)
title('F shape')

% ****************************************************************************************
% ****************************************************************************************
% ****************************************************************************************
%% 2. RRT to get initial level of support points from actual points needing
% support with minimum distance function iteratively checking over entire
% tree data

mapData = occupancyMatrix(map);
startPose = [55 75 pi/4];
goalPose = [85 2 pi/2];
bounds = [[-10 110]; [-10 90]; [-pi pi]];
ss = stateSpaceDubins(bounds);
ss.MinTurningRadius = 0.2;
stateValidator = validatorOccupancyMap(ss); 
stateValidator.Map = map;
stateValidator.ValidationDistance = 0.05;
planner = plannerRRT(ss,stateValidator);
planner.MaxConnectionDistance = 0.5;
planner.MaxIterations = 3000;
rng default
[pthObj, solnInfo] = plan(planner,startPose,goalPose);
show(occupancyMap(mapData))

plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-');



% ****************************************************************************************
% ****************************************************************************************
% ****************************************************************************************



%% 3.Actual points needing support on F shape Final2D function written
% already

xi=[0 30]
yi=[40 40]
Final2D(xi,yi);
xi2=[70 100]
yi2=[40 40]
Final2D(xi2,yi2);
a=[15,15]
b=[0 25]
plot(a,b,'-o', 'LineWidth', 2, 'MarkerSize', 5);
a=[85,85]
b=[0 25]
plot(a,b,'-o', 'LineWidth', 2, 'MarkerSize', 5);





% ****************************************************************************************
% ****************************************************************************************
% ****************************************************************************************

%% 4.TO GET BRIDGE LENGTH POINTS (POINTS NEEDING SUPPPORT ON ACTUAL BASE
% MATERIAL)

xi=[0,100];
yi=[80,80];
b=10;

plot(xi,yi,'-o', 'LineWidth', 2, 'MarkerSize', 5);
hold on

% Number of lines 
no_of_lines=numel(yi)-1;

%%STEP 2: AFTER PLOTTING LINES CHECK WHETHER THEY NEEDS SUPPPORT OR NOT

%L_slope==>calculates line slope first initializing it with zeroes
L_slope=zeros(1,size(xi,2)-1);
%phi==>calculates angles first initializing it with zeroes
phi=zeros(1,size(xi,2)-1);
%L==>Length of individual first lines initializing with zeroes
L=zeros(1,size(xi,2)-1);

% Check requirement of support structure
for i=1:no_of_lines
      L_slope(i)= (yi(i+1)-yi(i))/(xi(i+1)-xi(i));
      phi(i)=atan(L_slope(i)); 

      fprintf('The slope of the line from (%d,%d) to (%d,%d) is %.3f\n',xi(i),yi(i),xi(i+1),yi(i+1),L_slope(i))

      if L_slope(i)<1
           fprintf('The line from (%d,%d) to (%d,%d) requires a support structure for 3D printing\n',xi(i),yi(i),xi(i+1),yi(i+1));

      else
           fprintf('The line from (%d,%d) to (%d,%d) can be 3D printed without a support structure\n',xi(i),yi(i),xi(i+1),yi(i+1));
      end
           L(i)= sqrt(abs((yi(i+1)-yi(i))^2+(xi(i+1)-xi(i))^2)); 
end


no_of_lines=size(xi,2)-1;
xn=[];
yn=[];
for i=1:no_of_lines
    if rem(L(i),b)==0   %if remainder is zero that means we can divide the line into equal parts
        xn=[xn,xi(i):b*cos(phi(i)):xi(i+1)];
        if phi(i)==0
            if i>1
                k=size(xn,2)-size(yn,2);
                torepeat=repmat(yn(end), 1, k);
                yn=[yn,torepeat]
            elseif i==1
                k=size(xn,2)-size(yn,2);
                torepeat=repmat(yi(i), 1, k);
                yn=[yn,torepeat]
            else    
                yn=zeros(size(xn)); 
            end    
        else
            yn=[yn,yi(i):b*sin(phi(i)):yi(i+1)];
        end
    else
        xn=[xn,xi(i):b*cos(phi(i)):xi(i+1),xi(i+1)];
        if phi(i)==0
            if i>=1
                k=size(xn,2)-size(yn,2);
                torepeat=repmat(yn(end), 1, k);
                yn=[yn,torepeat];
            else    
                yn=zeros(size(xn)); 
            end 
            
        else
            yn=[yn,yi(i):b*sin(phi(i)):yi(i+1),yi(i+1)];
        end
    end
end

% plot(xn,yn,'-o', 'LineWidth', 2, 'MarkerSize', 5);





% ****************************************************************************************
% ****************************************************************************************
% ****************************************************************************************


%% 5. ITERATING OVER TREE DATA GENERATED THROUGH RRT METHOD TO GET SECOND
%LEVEL OF MINIMUM DISTANCE FUNCTION (FIND_GOAL_PT) FUNCTION ALREADY WRITTEN

comptreex=[];
comptreey=[];
for i=1:(size(xn,2))
    startPose1 = [xn(i) 80 -pi/4];
    out_goalPose = find_goal_pt([startPose1(1) startPose1(2 )],solnInfo.TreeData);
    comptreex=[comptreex,out_goalPose(1)];
    comptreey=[comptreey,out_goalPose(2)];
    an=[startPose1(1) out_goalPose(1)];
    bn=[startPose1(2) out_goalPose(2)];
    plot(an,bn,'-o', 'LineWidth', 2, 'MarkerSize', 5);
end


plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2)





% ****************************************************************************************
% ****************************************************************************************
% ****************************************************************************************


%% 6.PRM METHOD START TO FIND SHPRTEST PATH FROM SET OF POINTS OBTAINED IN
% PREVIOUS STEP TO SOME GOAL POINT (ALL START POINTS ARE CONVERGED TO SAME
% GOAL POINT)


hold on
map = binaryOccupancyMap(100,80, 1);
occ = zeros(80, 100);
occ(1,:) = 1;
occ(:,1) = 1;
occ(:,100)=1;
occ(40,[1:30]) = 1;
occ(40,[70:100])=1;
setOccupancy(map, occ)
inflatedmap=copy(map);

% subplot(121),show(map);
% subplot(122),
show(inflatedmap);

prm=mobilerobotprm;
prm.NumNodes=200;
prm.ConnectionDistance=10;
prm.Map=inflatedmap;
show(prm);
i=0;

pathCell = cell(1, 10);
while i<11
    i=i+1;
    disp('click for start');
    start=[comptreex(i) comptreey(i)];
    disp('click for end');
    goal=[50 2];
    path1=findpath(prm,start,goal);

      while isempty(path1)
            prm.NumNodes=prm.NumNodes+20;
            update(prm);
            path1=findpath(prm,start,goal);
            show(prm);
            pause(1);
      end  
  pathCell{i}=path1;
%   update(prm)
%   cellArray{index} = eval(['array', num2str(index)]);
show(prm);
hold on
end


map = binaryOccupancyMap(100,80, 1);
occ = zeros(80, 100);
occ(1,:) = 1;
occ(:,1) = 1;
occ(:,100)=1;
occ(40,[1:30]) = 1;
occ(40,[70:100])=1;
setOccupancy(map, occ)
inflatedmap=copy(map);
plot(comptreex,comptreey,'-o', 'LineWidth', 2, 'MarkerSize', 5);

hold on; 
show(inflatedmap);
% plot(comptreex,comptreey,'-o', 'LineWidth', 2, 'MarkerSize', 5);




% ****************************************************************************************
% ****************************************************************************************
% ****************************************************************************************

%% 7.PLOTTING THE OBTAINED PATHS
for i = 1:10
    path = pathCell{i};
     x = path(:, 1);
     y = path(:, 2); 
     plot(x, y, 'Color', 'r', 'LineWidth',2);
    
end


comptreex=[];
comptreey=[];
for i=1:(size(xn,2))
    startPose1 = [xn(i) 80 -pi/4];
    out_goalPose = find_goal_pt([startPose1(1) startPose1(2 )],solnInfo.TreeData);
    comptreex=[comptreex,out_goalPose(1)];
    comptreey=[comptreey,out_goalPose(2)];
    an=[startPose1(1) out_goalPose(1)];
    bn=[startPose1(2) out_goalPose(2)];
    plot(an,bn,'Color', 'r', 'LineWidth',2);
end  
hold off; 



% ****************************************************************************************
% ****************************************************************************************
% ****************************************************************************************
