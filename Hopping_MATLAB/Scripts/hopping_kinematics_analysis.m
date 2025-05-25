%==========================================================================
% An App to visualize the hopping robot kinematics with a joint state gui
% It also calculates the center of mass (with respect to frame 1: foot) 
% of the robot as the joint moves
%==========================================================================
%--------------------------------------------------------------------------
% Foot      = link 0 --> in contact with ground
% ankle y   = joint 1
% ankle z   = joint 2
% lower_leg = link 2
% knee      = joint 3
% upper_leg = link 3
% hip z     = joint 4
% hip y     = joint 5
% head      = link 5
%--------------------------------------------------------------------------
clear
clc
hopping_parameters;

q_stand_straight = [0 0 0 0 0]; % stand up straigt
global q_offset;
q_offset = q_stand_straight; % will change to a more intuitive config later
global q;
q = [0 0 0 0 0] + q_offset;
q_limit = [-pi/2 pi/2; -pi pi; -pi pi; -pi pi; -pi/2 pi/2]; % assume all the same

function [o_i_0, cm_1] = hopping_forward_kin(q)
    global head_pos_7 ankle_pos_1 lower_l upper_l m cm cm_frame;
    dh = [0    ankle_pos_1(3) ankle_pos_1(1)  pi/2;
          q(1) 0              0              -pi/2;
          q(2) lower_l        0               pi/2;
          q(3) 0              0              -pi/2;
          0    upper_l        0               0; % intermediate step to correct frame
          q(4) 0              0               pi/2;
          q(5) 0              0              -pi/2;
          0    head_pos_7(3)  head_pos_7(1)   0];
    
    transform_num = size(dh,1);
    frame_num = transform_num+1;
    A = zeros(4,4,transform_num);
    
    o = [0 0 0 1]'; % origin
    o_i_0 = zeros(4,frame_num);
    o_i_0(:,1) = o; % initial frame at bottom of the foot
    H = eye(4);
    cm_1 = zeros(4,1);
    M = sum(m);
    
    for i=1:transform_num
        th = dh(i,1);
        d  = dh(i,2);
        a  = dh(i,3);
        al = dh(i,4);
        A(:,:,i) = [cos(th) -sin(th)*cos(al)  sin(th)*sin(al) a*cos(th);
                    sin(th)  cos(th)*cos(al) -cos(th)*sin(al) a*sin(th);
                    0        sin(al)          cos(al)         d;
                    0        0                0               1];
    
        H = H*A(:,:,i);
        o_i_0(:,i+1) = H*o;
        
        id = find(cm_frame==i);
        if id
            cm_1 = cm_1 + m(id)/M*(H*cm(:,id));
        end
     end
end

global o_i_0 cm_1;
[o_i_0, cm_1] = hopping_forward_kin(q);

%% Joint state gui
% function joint_state_gui(q, q_limit, o_i_0)
fig = uifigure;
g = uigridlayout(fig);
g.RowHeight = {'1x','1x','1x','1x','1x'};
g.ColumnWidth = {40,'1x','1x','1x','1x'};

% UI
% try looking for ways to return some values from function back to 
% global config space
label1 = uilabel(g,'Text','theta1');
sld1 = uislider(g, "Limits", q_limit(1,:)*180/pi, "Value", 0, ...
    "ValueChangingFcn", @(src,event)updateJoint(src,event,1,g));
label2 = uilabel(g,'Text','theta2');
sld2 = uislider(g, "Limits", q_limit(2,:)*180/pi, "Value", 0, ...
    "ValueChangingFcn", @(src,event)updateJoint(src,event,2,g));
label3 = uilabel(g,'Text','theta3');
sld3 = uislider(g, "Limits", q_limit(3,:)*180/pi, "Value", 0, ...
    "ValueChangingFcn", @(src,event)updateJoint(src,event,3,g));
label4 = uilabel(g,'Text','theta4');
sld4 = uislider(g, "Limits", q_limit(4,:)*180/pi, "Value", 0, ...
    "ValueChangingFcn", @(src,event)updateJoint(src,event,4,g));
label5 = uilabel(g,'Text','theta5');
sld5 = uislider(g, "Limits", q_limit(5,:)*180/pi, "Value", 0, ...
    "ValueChangingFcn", @(src,event)updateJoint(src,event,5,g));

% Plot
global ax;
ax = uiaxes(g);
ax.XLim = [-0.4 0.4];
ax.YLim = [-0.4 0.4];
ax.ZLim = [0 0.4];

ax.Layout.Row = [2 5];
ax.Layout.Column = [3 5];
title.Layout.Row = 1;
title.Layout.Column = 4;
update_plot(g, o_i_0, cm_1);

% Callback function
function updateJoint(~, event, i, g)
    global q q_offset o_i_0 cm_1;
    val = event.Value; %deg
    q(i) = val*pi/180 + q_offset(i);
    [o_i_0, cm_1] = hopping_forward_kin(q);
    update_plot(g, o_i_0, cm_1);
end

% update plot function
function update_plot(g, o_i_0, cm_1)
    global ax
    ax.NextPlot = "replaceall";
    plot3(ax, o_i_0(1,:), o_i_0(2,:), o_i_0(3,:), ...
        "AffectAutoLimits", "off", "LineWidth",2);
    ax.XLim = [-0.4 0.4];
    ax.YLim = [-0.4 0.4];
    ax.ZLim = [0 0.4];
    ax.NextPlot = "add";
    plot3(ax, cm_1(1), cm_1(2), cm_1(3), "Marker","o");
    title = uilabel(g,'Text','Frame positions');
    title.FontSize = 16;

    ax.Layout.Row = [2 5];
    ax.Layout.Column = [3 5];
    title.Layout.Row = 1;
    title.Layout.Column = 4;
end

% Layout commands
label1.Layout.Row = 1;
label1.Layout.Column = 1;
label2.Layout.Row = 2;
label2.Layout.Column = 1;
label3.Layout.Row = 3;
label3.Layout.Column = 1;
label4.Layout.Row = 4;
label4.Layout.Column = 1;
label5.Layout.Row = 5;
label5.Layout.Column = 1;

sld1.Layout.Row = 1;
sld1.Layout.Column = 2;
sld2.Layout.Row = 2;
sld2.Layout.Column = 2;
sld3.Layout.Row = 3;
sld3.Layout.Column = 2;
sld4.Layout.Row = 4;
sld4.Layout.Column = 2;
sld5.Layout.Row = 5;
sld5.Layout.Column = 2;
