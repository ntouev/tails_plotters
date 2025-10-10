function motion_vis(out)
% 2x2 layout:
% Left column: World view (top, miniature STL moves on trajectory),
%              Attitude-only view (bottom, larger STL rotates at origin, NED axes)
% Right column: Z-angle(t) (top), Y-angle(t) (bottom)
% Rotations: drive with QUATERNIONS out.q.Data = [qs qx qy qz]
%            use Euler ZXY ONLY for display/plots.
% Both position and world axes use NED: [N, E, D], with D positive down.
% Visualization plays at 50 Hz (downsampled). Yaw (Z) is unwrapped.

%% Time & data
t    = out.t.Data(:);           % Nx1
PNED = out.p.Data;              % Nx3, NED order: [N, E, D]
Q    = out.q.Data;              % Nx4, [qs qx qy qz]
Q    = Q ./ vecnorm(Q,2,2);     % normalize rows
N    = numel(t);

% --- Optional second UAV (position-only, NED) ---
PNED2 = squeeze(out.xg.Data)';  % [N2 x 3]
N2    = size(PNED2,1);          % may differ from N

% Rotation matrices (body->world) from quaternions
Rall = zeros(3,3,N);
for i = 1:N
    Rall(:,:,i) = quat2rotm(Q(i,:));
end

% Euler ZXY ONLY for display/labels
zxy   = zeros(N,3);
for i = 1:N
    zxy(i,:) = rotm2eul(Rall(:,:,i), 'ZXY'); % [z x y] rad
end
zDeg = unwrap(zxy(:,1))*180/pi;  % unwrapped yaw (Z)
xDeg = zxy(:,2)*180/pi;          %#ok<NASGU> % (unused but computed for completeness)
yDeg = zxy(:,3)*180/pi;

%% === 50 Hz visualization timeline (UI-only) ===
fs_vis = 50; dt_vis = 1/fs_vis;              % 50 Hz -> 0.02 s
t_vis  = (t(1):dt_vis:t(end)).';             % uniform visual timeline

% Last 2 seconds of positions in the trail
Mtrail = max(1, round(2 / dt_vis));          % ~2 seconds worth of frames

% Map each t_vis to nearest original sample index
IDX = zeros(numel(t_vis),1);
j = 1;
for k = 1:numel(t_vis)
    while j < N && t(j) < t_vis(k), j = j + 1; end
    if j == 1
        IDX(k) = 1;
    elseif t(j) == t_vis(k) || j == N
        IDX(k) = j;
    else
        if (t(j) - t_vis(k)) <= (t_vis(k) - t(j-1)), IDX(k) = j; else, IDX(k) = j-1; end
    end
end
[IDX, ia] = unique(IDX, 'stable');   % keep unique frames, in order
t_vis     = t_vis(ia);

% Precompute downsampled angles for plots
zDeg_vis = zDeg(IDX);
yDeg_vis = yDeg(IDX);
xDeg_vis = xDeg(IDX);

%% UI & grid  (LEFT: 70/30, RIGHT: 50/50)
f  = uifigure('Name','UAV (Quaternion-driven, ZXY display, NED positions, 50 Hz)',...
              'Position',[60 60 1200 820],'Color','white');
f.AutoResizeChildren = 'off';

% Top-level: 1 row x 2 columns
top = uigridlayout(f,[1 2]);
top.ColumnWidth = {'1x','1x'};
top.RowHeight   = {'1x'};

% LEFT side grid: 2 rows (70% / 30%)
leftPanel = uipanel(top); leftPanel.Layout.Row = 1; leftPanel.Layout.Column = 1;
leftGrid  = uigridlayout(leftPanel,[2 1]);
leftGrid.RowHeight   = {'0.7x','0.3x'};
leftGrid.ColumnWidth = {'1x'};

% RIGHT side grid: 3 rows (equal split)
rightPanel = uipanel(top); rightPanel.Layout.Row = 1; rightPanel.Layout.Column = 2;
rightGrid  = uigridlayout(rightPanel,[3 1]);
rightGrid.RowHeight   = {'1x','1x','1x'};
rightGrid.ColumnWidth = {'1x'};

% Top-left: world view (NED)
axWorld = uiaxes(leftGrid); axWorld.Layout.Row=1; axWorld.Layout.Column=1;
hold(axWorld,'on'); grid(axWorld,'on'); view(axWorld,3);
axWorld.ZDir = 'reverse';                          % NED visual: D positive down
plot3(axWorld,PNED(:,1),PNED(:,2),PNED(:,3),':');  % N, E, D path
axis(axWorld,'equal'); daspect(axWorld,[1 1 1]);
xlabel(axWorld,'N [m]'); ylabel(axWorld,'E [m]'); zlabel(axWorld,'D [m]');
title(axWorld,'World view (NED; miniature STL moving on trajectory)');

% Bottom-left: attitude-only (NED)
axBody = uiaxes(leftGrid); axBody.Layout.Row=2; axBody.Layout.Column=1;
hold(axBody,'on'); grid(axBody,'on'); view(axBody,3);
axBody.ZDir = 'reverse';
axis(axBody,[-0.9 0.9 -0.9 0.9 -0.3 0.3]); daspect(axBody,[1 1 1]);
xlabel(axBody,'X_b (forward)'); ylabel(axBody,'Y_b (right)'); zlabel(axBody,'Z_b (down)');
title(axBody,'Attitude-only (NED body frame)');

% Top right
axY = uiaxes(rightGrid); axY.Layout.Row=1; axY.Layout.Column=1;
hold(axY,'on'); grid(axY,'on');
plot(axY, t_vis, yDeg_vis);
ylabel(axY,'Y [deg]'); xlabel(axY,'Time [s]');
xlim(axY,[t_vis(1) t_vis(end)]); title(axY,'Y vs Time (Euler ZXY, 50 Hz)');
set(axY.Children,'HitTest','off','PickableParts','none');

% Middle right
axX = uiaxes(rightGrid); axX.Layout.Row=2; axX.Layout.Column=1;
hold(axX,'on'); grid(axX,'on');
plot(axX, t_vis, xDeg_vis);
ylabel(axX,'X [deg]'); xlabel(axX,'Time [s]');
xlim(axX,[t_vis(1) t_vis(end)]); title(axX,'X vs Time (Euler ZXY, 50 Hz)');
set(axX.Children,'HitTest','off','PickableParts','none');

% Bottom right
axZ = uiaxes(rightGrid); axZ.Layout.Row=3; axZ.Layout.Column=1;
hold(axZ,'on'); grid(axZ,'on');
plot(axZ, t_vis, zDeg_vis);
ylabel(axZ,'Z [deg]'); xlabel(axZ,'Time [s]');
xlim(axZ,[t_vis(1) t_vis(end)]); title(axZ,'Z vs Time (Euler ZXY, 50 Hz)');
set(axZ.Children,'HitTest','off','PickableParts','none');

% Labels & controls (anchored near bottom-left of axBody)
lblT = uilabel(f,'Text','t = 0.00 s','FontWeight','bold','Position',[0 0 220 22]);
btn  = uibutton(f,'state','Text','▶ Play','Position',[0 0 90 30], ...
                'ValueChangedFcn',@(b,~)togglePlay(b));

% World marker (use downsampled initial frame)
hPt = plot3(axWorld,PNED(IDX(1),1),PNED(IDX(1),2),PNED(IDX(1),3),...
            'o','MarkerSize',6,'LineWidth',1,'Color',[0 0 0]);

% Black trailing dots (last ~2 seconds) in world view
hTrail = plot3(axWorld, nan, nan, nan, 'o', ...
    'MarkerSize',1, 'MarkerFaceColor','k', 'MarkerEdgeColor','k', 'LineStyle','none');

%% --- Second UAV (green dot + red full-history trail) ---
if N2 >= 1
    i0_2  = min(IDX(1), N2);
    hPt2  = plot3(axWorld, PNED2(i0_2,1), PNED2(i0_2,2), PNED2(i0_2,3), ...
                  'o', 'MarkerSize',5, 'MarkerFaceColor','g', 'MarkerEdgeColor','g', 'LineStyle','none');
    hTrail2 = plot3(axWorld, nan, nan, nan, '-', 'Color',[1 0 0], 'LineWidth',1.2);
else
    % placeholders to keep code simple
    hPt2    = plot3(axWorld, nan, nan, nan, 'o', 'MarkerSize',5, 'MarkerFaceColor','g', 'MarkerEdgeColor','g');
    hTrail2 = plot3(axWorld, nan, nan, nan, '-', 'Color',[1 0 0], 'LineWidth',1.2);
end

%% STL MODEL
stlFile = 'Cyclone2.stl';
[F0,V0] = local_read_stl(stlFile);
centroid = mean(V0,1); V0c = V0 - centroid;
box  = max(V0c) - min(V0c); unit = max(box); if unit==0, unit=1; end
Vunit = V0c / unit;

meshEulerZXY = [0 0 0];          % radians
Rb0 = eul2rotm(meshEulerZXY, 'ZXY');
Vunit_rb = (Rb0 * Vunit.').';

% Scales (compute from scene PNED extents)
rangeN = max(PNED(:,1)) - min(PNED(:,1));
rangeE = max(PNED(:,2)) - min(PNED(:,2));
rangeD = max(PNED(:,3)) - min(PNED(:,3));
sceneSize = max([rangeN, rangeE, rangeD]); if sceneSize==0, sceneSize=1; end
scale_world = max(0.1, min(0.24*sceneSize, 4.0));  % miniature
scale_body  = 0.9;                                  % large

V_world = Vunit_rb * scale_world;
V_body  = Vunit_rb * scale_body;

% Transforms and patches (lighter look)
hT_world = hgtransform('Parent',axWorld);
hT_body  = hgtransform('Parent',axBody);

patch('Parent',hT_world,'Faces',F0,'Vertices',V_world, ...
      'FaceColor',[0.35 0.65 0.95],'EdgeColor','k','FaceAlpha',0.95, ...
      'DiffuseStrength',0.8,'AmbientStrength',0.35);

patch('Parent',hT_body,'Faces',F0,'Vertices',V_body, ...
      'FaceColor',[0.35 0.65 0.95],'EdgeColor','none','FaceAlpha',1.0, ...
      'DiffuseStrength',0.8,'AmbientStrength',0.35);

% Expand world axes (NED)
pad = 0.1*sceneSize + scale_world;
xlim(axWorld,[min(PNED(:,1))-pad, max(PNED(:,1))+pad]);   % N
ylim(axWorld,[min(PNED(:,2))-pad, max(PNED(:,2))+pad]);   % E
zlim(axWorld,[min(PNED(:,3))-pad, max(PNED(:,3))+pad]);   % D

% Lighting (try; some UIAxes configs don’t support lighting fully)
try
    light(axWorld,'Style','infinite'); lighting(axWorld,'gouraud');
    light(axBody, 'Style','infinite'); lighting(axBody, 'gouraud');
catch
end

% Body axes triad (no legend)
Lax=0.7; %#ok<NASGU>
plot3(axBody,[0 0.7],[0 0],[0 0],'LineWidth',2);
plot3(axBody,[0 0],[0 0.7],[0 0],'LineWidth',2);
plot3(axBody,[0 0],[0 0],[0 0.7],'LineWidth',2);

% Cursors (use 50 Hz time)
cxY = xline(axY,  t_vis(1),'-','Cursor'); cxY.LabelVerticalAlignment='bottom';
cxX = xline(axX,  t_vis(1),'-','Cursor'); cxX.LabelVerticalAlignment='bottom';
cxZ = xline(axZ,  t_vis(1),'-','Cursor'); cxZ.LabelVerticalAlignment='bottom';

% Fixed STL alignment (ZXY): z=0, x=0, y=-pi/2   (mesh fixed rotation)
Rfix = eul2rotm([0, 0, -pi/2], 'ZXY');

%% === PERSON FIGURE AT ORIGIN (NED) ===
personHeight = 1.75;   % meters
draw_person_ned(axWorld, [0 0 0], personHeight);

%% Playback / interaction state (50 Hz)
kFrame = 1;                        % visual-frame index (1..M)
M = numel(t_vis);
curTime = t_vis(kFrame);           % visual time
isDragging=false; dragAx=[];
isPlaying=false; tmr=[];
playWasOn = false;

% Anchor controls initially and on figure resize
repositionControls();
f.SizeChangedFcn = @(~,~)repositionControls();

% Mouse & keys
axX.ButtonDownFcn       = @(~,~) startDrag(axX);
axZ.ButtonDownFcn       = @(~,~) startDrag(axZ);
axY.ButtonDownFcn       = @(~,~) startDrag(axY);
f.WindowButtonMotionFcn = @(~,~) onDrag();
f.WindowButtonUpFcn     = @(~,~) stopDrag();
f.WindowKeyPressFcn     = @(~,ev) onKey(ev);
f.CloseRequestFcn       = @(src,~) onClose(src);

% Init first frame
moveToK(1);

%% ===== nested functions =====
    function repositionControls()
        % place btn and lblT at bottom-left corner of axBody, inside figure coords
        drawnow limitrate;
        p = axBody.Position;   % [x y w h] in pixels relative to figure
        margin = 10;
        btn.Position  = [p(1)+margin, p(2)+margin, 90, 30];
        lblT.Position = [btn.Position(1)+btn.Position(3)+10, p(2)+margin+4, 220, 22];
    end

    function onKey(ev)
        switch ev.Key
            case {'rightarrow','d'}
                moveToK(min(kFrame+1, M));
            case {'leftarrow','a'}
                moveToK(max(kFrame-1, 1));
            case {'space'}
                btn.Value = ~btn.Value; togglePlay(btn);
        end
    end

    function startDrag(ax)
        isDragging=true; dragAx=ax;
        playWasOn = isPlaying;
        if isPlaying, stopPlay(); end
        cp=ax.CurrentPoint(1,1); cp=max(min(cp,t_vis(end)),t_vis(1));
        [~,k]=min(abs(t_vis - cp));
        moveToK(k);
    end

    function onDrag()
        if ~isDragging || isempty(dragAx), return; end
        cp=dragAx.CurrentPoint(1,1); cp=max(min(cp,t_vis(end)),t_vis(1));
        [~,k]=min(abs(t_vis - cp));
        moveToK(k);
    end

    function stopDrag()
        isDragging=false; dragAx=[];
        if playWasOn, startPlay(); end
    end

    function togglePlay(b)
        if b.Value, b.Text='⏸ Pause'; startPlay();
        else,       b.Text='▶ Play';  stopPlay(); end
    end

    function startPlay()
        if isPlaying, return; end
        isPlaying = true;
        if isempty(tmr) || ~isvalid(tmr)
            tmr = timer('ExecutionMode','fixedSpacing', ...
                        'Period',dt_vis, ...
                        'TimerFcn',@(~,~) stepForward(), ...
                        'StartDelay',0);
        else
            tmr.Period = dt_vis;
        end
        start(tmr);
    end

    function stopPlay()
        isPlaying = false;
        if ~isempty(tmr) && isvalid(tmr), stop(tmr); end
    end

    function stepForward()
        if kFrame >= M
            stopPlay(); btn.Value=false; btn.Text='▶ Play'; return;
        end
        moveToK(kFrame+1);
    end

    function moveToK(k)
        kFrame  = k;
        curTime = t_vis(kFrame);
        iFrame  = IDX(kFrame);                 % map to original index

        % Orientation: body->world (Rall) then fixed mesh->body (Rfix)
        Rtot = Rall(:,:,iFrame) * Rfix;

        % Update transforms (use NED position PNED)
        T1 = eye(4); T1(1:3,1:3)=Rtot; T1(1:3,4)=PNED(iFrame,:).';
        set(hT_world,'Matrix',T1);
        T2 = eye(4); T2(1:3,1:3)=Rtot; set(hT_body,'Matrix',T2);

        % Main UAV marker & time label
        set(hPt,'XData',PNED(iFrame,1),'YData',PNED(iFrame,2),'ZData',PNED(iFrame,3));

        % Update black trailing dots (last ~2 seconds of visual frames)
        kStart = max(1, kFrame - (Mtrail - 1));
        kIdx   = kStart:kFrame;
        ptsIdx = IDX(kIdx);
        Ptrail = PNED(ptsIdx,:);
        set(hTrail, 'XData', Ptrail(:,1), 'YData', Ptrail(:,2), 'ZData', Ptrail(:,3));

        % --- Second UAV (green dot + red trail) ---
        if N2 >= 1
            iFrame2 = min(iFrame, N2); % clamp
            set(hPt2,'XData',PNED2(iFrame2,1),'YData',PNED2(iFrame2,2),'ZData',PNED2(iFrame2,3));

            idx2 = IDX(1:kFrame); idx2 = idx2(idx2 <= N2);
            if ~isempty(idx2)
                P2trail = PNED2(idx2,:);
                set(hTrail2, 'XData', P2trail(:,1), 'YData', P2trail(:,2), 'ZData', P2trail(:,3));
            else
                set(hTrail2, 'XData', nan, 'YData', nan, 'ZData', nan);
            end
        end

        lblT.Text = sprintf('t = %.3f s (50 Hz vis)', curTime);

        % Sync cursors (visual time)
        cxY.Value = curTime; cxX.Value = curTime; cxZ.Value = curTime; 

        drawnow limitrate
    end

    function onClose(src)
        stopPlay();
        if ~isempty(tmr) && isvalid(tmr), delete(tmr); end
        delete(src);
    end
end

%% ===== helper: STL read =====
function [F,V] = local_read_stl(stlFile)
    try
        tr = stlread(stlFile);
        F = tr.ConnectivityList;
        V = tr.Points;
    catch
        [F,V] = stlread(stlFile); %#ok<STLREAD>
    end
end

%% ===== helper: draw a simple person at origin in NED =====
function draw_person_ned(ax, originNED, height)
% originNED = [N E D]; height in meters
if nargin < 3, height = 1.75; end
N0 = originNED(1); E0 = originNED(2); D0 = originNED(3);

% Proportions
rHead   = 0.12*height/1.75;
legL    = 0.52*height;      % feet->hip
torsoL  = 0.38*height;      % hip->neck
neckL   = 0.02*height;
shouldW = 0.26*height;
hipW    = 0.18*height;

% Key depths (D is positive DOWN; up is negative)
D_foot   = D0;                                % feet at origin
D_hip    = D_foot - legL;
D_neck   = D_hip  - torsoL;
D_head_c = D_neck - neckL - rHead;

% X,Y for symmetry around origin N0,E0
N_left  = N0 - shouldW/2;
N_right = N0 + shouldW/2;
N_hipL  = N0 - hipW/2;
N_hipR  = N0 + hipW/2;

% Colors
colLine = [0.2 0.2 0.2];

hold(ax,'on');

% Legs (hip to feet)
plot3(ax,[N_hipL N0],[E0 E0],[D_hip D_foot],'LineWidth',2,'Color',colLine);
plot3(ax,[N_hipR N0],[E0 E0],[D_hip D_foot],'LineWidth',2,'Color',colLine);

% Torso (hip to neck)
plot3(ax,[N0 N0],[E0 E0],[D_hip D_neck],'LineWidth',3,'Color',colLine);

% Arms (shoulder line)
D_sh = D_hip - 0.7*torsoL;
plot3(ax,[N_left N_right],[E0 E0],[D_sh D_sh],'LineWidth',2,'Color',colLine);

% Head (sphere)
[XS,YS,ZS] = sphere(20);
XS = N0 + rHead*XS;
YS = E0 + rHead*YS;
ZS = D_head_c + rHead*ZS;     % centered at head depth
surf(ax,XS,YS,ZS,'FaceColor',[0.6 0.6 0.6],'EdgeColor','none','FaceAlpha',1);
end
