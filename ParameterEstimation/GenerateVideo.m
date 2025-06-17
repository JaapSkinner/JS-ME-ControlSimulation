% Plot 3D animation of UAV. Run directly after simulation
% Original by Junyi Chen, updated 29 Jun 23 by NJKay for generalised model.

%% 14AUG2023 - Allow user to load file if not in workspace

%If simOut is not in the workspace
if ~exist('simOut','var')

    %Clear Workspace
    clear

    %Get user to choose output file
[OutFile,OutPath] = uigetfile([pwd,'/data_results/*.mat'],'Select a results data set');

%Load File
load(fullfile(OutPath,OutFile))

end

%% Retrieve x, z, and pitch data
logsout = simOut.get('logsout');    % retrieve loggged data from Simulink output
tout = simOut.get('tout');          % retrieve time data
tstart = 60;
tend = tstart + 30;
tCutOff = tout((abs(tout-tstart)) == min(abs(tout-tstart)));                  % user specified start time for analysis
tCutOffEnd = tout((abs(tout-tend)) == min(abs(tout-tend)));             % user specified end time for analysis
loc = getsampleusingtime( logsout.get('xi').Values, tCutOff, tCutOffEnd);
att = getsampleusingtime( logsout.get('eta').Values, tCutOff, tCutOffEnd);

%Get data by axis
x = loc.Data(:,1);
y = loc.Data(:,2);
z = loc.Data(:,3);
roll = att.Data(:,1);
pitch = att.Data(:,2);
yaw = att.Data(:,3);

%Get PWM
PWM = getsampleusingtime( logsout.get('pwm').Values, tCutOff, tCutOffEnd);
pwm = squeeze(PWM.Data);

%% Create figure window
screenSize = get(0,'ScreenSize');
fig = figure('color',[1,1,1],'Position',[screenSize(3)*0.1,screenSize(4)*0.1,screenSize(3)*0.8,screenSize(4)*0.8]);
hold on
grid on
axis equal

%% Plot path
subplot(2,4,[1,2,5,6])
plot3(x,-y,-z,'color',[0.75,0.75,0.75])
hold on
plot3(x(1),-y(1),-z(1),'bo')
plot3(x(end),-y(end),-z(end),'ro')

xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
hTitle1 = title('t = 0s');
view(-30,30)

%% Plot initial UAV
% Base rotor coordinates
nRotorPts = 30;
rotorPts = [0.5*Uav.D_PROP*cos(linspace(0,2*pi,nRotorPts));0.5*Uav.D_PROP*sin(linspace(0,2*pi,nRotorPts));zeros(Uav.N_ROTORS,nRotorPts)];

% Set axis limits to fit in UAV at all times
xMax = max(x)+(0.5*Uav.D_UAV+max(Uav.D_PROP));
xMin = min(x)-(0.5*Uav.D_UAV+max(Uav.D_PROP));
yMax = max(y)+(0.5*Uav.D_UAV+max(Uav.D_PROP));
yMin = min(y)-(0.5*Uav.D_UAV+max(Uav.D_PROP));
zMin = max(z)+(0.5*Uav.D_UAV+max(Uav.D_PROP));
zMax = min(z)-(0.5*Uav.D_UAV+max(Uav.D_PROP));

%% Add rotors, in order of the Uav array

for i=1:Uav.N_ROTORS
    %Extract rotor points data for the given prop
    rotorPts_i = rotorPts([i,i+Uav.N_ROTORS,i+2*Uav.N_ROTORS],:);

    %Get arm coordinates from Uav
    uavAnim.arm(i,1).coord = [0,Uav.MotorLoc(i,1);
        0,Uav.MotorLoc(i,2);
        0,Uav.MotorLoc(i,3)];

    %Rotate rotor points based on tilt angles
    rotorPts_i = Uav.R_MOTOR_TO_BODY(:,:,i)*rotorPts_i;

    %Add arm end points offsets
    uavAnim.rotor(i,1).coord = [rotorPts_i(1,:)+uavAnim.arm(i,1).coord(1,2);
        rotorPts_i(2,:)+uavAnim.arm(i,1).coord(2,2);
        rotorPts_i(3,:)+uavAnim.arm(i,1).coord(3,2)];

    %Plot arm
    subplot(2,4,[1,2,5,6])
    armPlt(i,1) = plot3(uavAnim.arm(i,1).coord(1,:),uavAnim.arm(i,1).coord(2,:),uavAnim.arm(i,1).coord(3,:),'k','LineWidth',1.5);

    %Plot rotors, different colours for different rotations (r CCW, B CW)if
    if Uav.ROTOR_DIRECTION(i) == -1
        rotorcolour = [0,0,1];
    else
        rotorcolour = [1,0,0];
    end
    rotPlt(i,1) = plot3(uavAnim.rotor(i,1).coord(1,:),uavAnim.rotor(i,1).coord(2,:),uavAnim.rotor(i,1).coord(3,:),'Color',rotorcolour,'LineWidth',1.5);
    rotPWM(i,1) = fill3(uavAnim.rotor(i,1).coord(1,:),uavAnim.rotor(i,1).coord(2,:),uavAnim.rotor(i,1).coord(3,:),rotorcolour,'EdgeColor','none');
    rotPWM(i).FaceAlpha = pwm(i,1);
    axis equal

end

%Add arrow for direction
ArrowStick = [0,0.75*Uav.D_UAV;0,0;0,0];
ArrowHead = [0.75*Uav.D_UAV,0.65*Uav.D_UAV,0.65*Uav.D_UAV;0,0.05*Uav.D_UAV,-0.05*Uav.D_UAV;0,0,0];
ArrowStickPlt = plot3(ArrowStick(1,:),ArrowStick(2,:),ArrowStick(3,:),'k','LineWidth',2);
ArrowHeadPlt = fill3(ArrowHead(1,:),ArrowHead(2,:),ArrowHead(3,:),[0,0,0]);
grid on
box on

uavUpd = uavAnim;
set(gca,'XLim',[xMin,xMax],'YLim',[yMin,yMax],'ZLim',[min([-1,zMin]),max([1,zMax])])

%Bar plots

%Position
subplot(2,4,3)
X = categorical({'X','Y','Z'});
X = reordercats(X,{'X','Y','Z'});
Y = [x(1),y(1),z(1)];
posbar = bar(X,Y);
ylabel('Position, m')
set(gca,'YLim',[min([xMin,yMin,zMin]),max([xMax,yMax,zMax])])
grid on

%Attitude
subplot(2,4,4)
X = categorical({'\phi','\theta','\psi'});
X = reordercats(X,{'\phi','\theta','\psi'});
Y = [roll(1),pitch(1),yaw(1)];
attbar = bar(X,Y);
ylabel('Attitude, \circ')
set(gca,'YLim',[-15,15])
grid on

%PWM
subplot(2,4,7:8)
pwmbar = bar(pwm(:,1));
ylabel('PWM')
set(gca,'YLim',[0,1])
grid on

%% Set up vid
framecount = 1;

%Find which frames to extract
VidFreq = 30;            %Video freq (Hz)
Tlist = (loc.Time(1)):(1/VidFreq):(loc.Time(end));
Tlist = [Tlist,Tlist(end)+(1/VidFreq),Tlist+(2/VidFreq)];

%% Update plot data
nPts = length(x);
ang = [roll,-pitch,-yaw];
pos = [x,-y,-z];

for i=1:nPts

    %to save time, only process if going to save to video
    if ((loc.Time(i)>=Tlist(framecount))&&(loc.Time(i)<Tlist(framecount+1)))
        
        R = [cos(ang(i,3))*cos(ang(i,2)),cos(ang(i,3))*sin(ang(i,2))*sin(ang(i,1))-sin(ang(i,3))*cos(ang(i,1)),cos(ang(i,3))*sin(ang(i,2))*cos(ang(i,1))+sin(ang(i,3))*sin(ang(i,1));
            sin(ang(i,3))*cos(ang(i,2)),sin(ang(i,3))*sin(ang(i,2))*sin(ang(i,1))+cos(ang(i,3))*cos(ang(i,1)),sin(ang(i,3))*sin(ang(i,2))*cos(ang(i,1))-cos(ang(i,3))*sin(ang(i,1));
            -sin(ang(i,2)),cos(ang(i,2))*sin(ang(i,1)),cos(ang(i,2))*cos(ang(i,1))];
        for j=1:Uav.N_ROTORS
            uavUpd.arm(j).coord = R*uavAnim.arm(j).coord;
            uavUpd.rotor(j).coord = R*uavAnim.rotor(j).coord;
            
            for k=1:3
                uavUpd.arm(j).coord(k,:) = uavUpd.arm(j).coord(k,:)+pos(i,k);
                uavUpd.rotor(j).coord(k,:) = uavUpd.rotor(j).coord(k,:)+pos(i,k);
            end
            set(armPlt(j),'XData',uavUpd.arm(j).coord(1,:),...
                'YData',uavUpd.arm(j).coord(2,:),...
                'ZData',uavUpd.arm(j).coord(3,:))
            set(rotPlt(j),'XData',uavUpd.rotor(j).coord(1,:),...
                'YData',uavUpd.rotor(j).coord(2,:),...
                'ZData',uavUpd.rotor(j).coord(3,:))
            set(rotPWM(j),'XData',uavUpd.rotor(j).coord(1,:),...
                'YData',uavUpd.rotor(j).coord(2,:),...
                'ZData',uavUpd.rotor(j).coord(3,:),...
                'FaceAlpha',pwm(j,i))

        end
        ArrowStickAnim = R*ArrowStick+(pos(i,:))';
        ArrowHeadAnim = R*ArrowHead+(pos(i,:))';
        set(ArrowStickPlt,'XData',ArrowStickAnim(1,:),...
            'YData',ArrowStickAnim(2,:),...
            'ZData',ArrowStickAnim(3,:))
        set(ArrowHeadPlt,'XData',ArrowHeadAnim(1,:),...
            'YData',ArrowHeadAnim(2,:),...
            'ZData',ArrowHeadAnim(3,:))
        set(posbar,'YData',[x(i),y(i),z(i)])
        set(attbar,'YData',[roll(i),pitch(i),yaw(i)]*180/pi)
        set(pwmbar,'YData',pwm(:,i))
        set(hTitle1,'String',sprintf('T = %.1f s' ,loc.Time(i)))
        drawnow



        PlotVid(framecount) = getframe(fig);
        framecount = framecount + 1;
        if (framecount+1) > length(Tlist)
            break
        end
    end
end

disp('Please wait while the video is written')
warning('off','MATLAB:audiovideo:VideoWriter:mp4FramePadded')
[~, windName, ~] = fileparts(windFile);
VidName = Uav.uavType + "_" + windName;
video = VideoWriter(char("Results/ParameterEstimation/Videos/"+ VidName + ".avi"));
video.FrameRate = VidFreq;
open(video)
writeVideo(video, PlotVid);
close(video);
