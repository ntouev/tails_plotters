function traj_vis_anim(varargin)

    homeDir = getenv('HOME');
    addpath(genpath(fullfile(homeDir,'tails_plotters/')));
    
    %% Parse inputs
    if nargin == 3
        ref        = struct();
        actual     = varargin{1};
        title_name = varargin{2};
        fs_vis     = varargin{3};
    elseif nargin == 4
        ref        = varargin{1};
        actual     = varargin{2};
        title_name = varargin{3};
        fs_vis     = varargin{4};
    else
        error('Usage: traj_vis_anim(actual, name)  OR traj_vis_anim(ref, actual, name)');
    end
    
    tActual = actual.t;              % Nx1
    pActual = actual.p;              % Nx3, NED [N E D]
    qActual = actual.q;              % Nx4, [qs qx qy qz]
    ASActual = actual.AS;
    qActual = qActual ./ vecnorm(qActual,2,2); % normalize quaternion
    
    hasRef = ~isempty(ref);
    if hasRef && isfield(ref,'p') && isfield(ref,'q') && ~isempty(ref.p) && ~isempty(ref.q)
        pRef = ref.p;
        qRef = ref.q;
        qRef = qRef ./ vecnorm(qRef,2,2);
    else
        hasRef = false;
        pRef   = nan(0,3);
        qRef   = nan(0,4);
    end
    
    N = numel(tActual);
    
    %% Rotations (actual only; ref Euler not used anywhere)
    Rall = zeros(3,3,N);
    for i = 1:N
        Rall(:,:,i) = quat2rotm(qActual(i,:));
    end
    
    zxy = zeros(N,3);
    for i = 1:N
        zxy(i,:) = rotm2eul(Rall(:,:,i), 'ZXY');
    end
    
    %% Visualization timeline
    dt_vis = 1/fs_vis;
    t_vis  = (tActual(1):dt_vis:tActual(end)).';
    IDX = zeros(numel(t_vis),1);
    j = 1;
    for k = 1:numel(t_vis)
        while j < N && tActual(j) < t_vis(k), j = j + 1; end
        if j == 1
            IDX(k) = 1;
        elseif tActual(j) == t_vis(k) || j == N
            IDX(k) = j;
        else
            if (tActual(j) - t_vis(k)) <= (t_vis(k) - tActual(j-1)), IDX(k) = j;
            else, IDX(k) = j-1; end
        end
    end
    [IDX, ia] = unique(IDX,'stable');
    t_vis     = t_vis(ia);
    
    % ~2 s trail
    Mtrail = max(1, round(2 / dt_vis));
    
    %% UI & layout
    f = uifigure('Name', sprintf('%s — %.1f Hz', title_name, fs_vis), ...
                 'Position', [60 60 1200 820], 'Color','white');
    
    
    % Top-level grid: main content + fixed-height controls row
    top = uigridlayout(f,[2 1]);
    top.RowHeight   = {'1x', 70};      % bottom row reserved for controls
    top.ColumnWidth = {'1x'};
    
    % Main panel & grid (2 rows x 2 cols)
    leftPanel = uipanel(top); leftPanel.Layout.Row = 1; leftPanel.Layout.Column = 1;
    leftGrid  = uigridlayout(leftPanel,[2 2]);
    leftGrid.RowHeight     = {'0.7x','0.3x'};
    leftGrid.ColumnWidth = {'0.5x','0.5x'};
    leftGrid.Padding       = [10 10 10 10];
    leftGrid.RowSpacing    = 10;
    leftGrid.ColumnSpacing = 10;
    
    % World view (span 2 columns on top row)
    axWorld = uiaxes(leftGrid); 
    axWorld.Layout.Row = 1; 
    axWorld.Layout.Column = [1 2];
    hold(axWorld,'on'); grid(axWorld,'on'); view(axWorld,3);
    axWorld.ZDir = 'reverse'; 
    axWorld.YDir = 'reverse';
    axis(axWorld,'equal'); daspect(axWorld,[1 1 1]);
    xlabel(axWorld,'N [m]'); ylabel(axWorld,'E [m]'); zlabel(axWorld,'D [m]');
    
    % Trajectory paths
    plot3(axWorld, pActual(:,1), pActual(:,2), pActual(:,3), ':', 'Color', [0.9 0.4 0]);
    if hasRef && ~isempty(pRef)
        plot3(axWorld, pRef(:,1), pRef(:,2), pRef(:,3), '.', 'MarkerSize', 1, 'Color', [0 0.447 0.741]);
    end
    
    % Body view (bottom-left) centered
    bodyPanel = uipanel(leftGrid);
    bodyPanel.Layout.Row    = 2;
    bodyPanel.Layout.Column = 1;
    
    % 3-column grid → center item
    bodyGrid = uigridlayout(bodyPanel,[1 3]);
    bodyGrid.ColumnWidth   = {'1x','fit','1x'};
    bodyGrid.RowHeight     = {'1x'};
    bodyGrid.Padding       = [0 0 0 0];
    bodyGrid.ColumnSpacing = 0;
    bodyGrid.RowSpacing    = 0;
    
    axBody = uiaxes(bodyGrid);
    axBody.Layout.Row    = 1;
    axBody.Layout.Column = 2;
    
    hold(axBody,'on'); grid(axBody,'on'); view(axBody,3);
    axBody.ZDir = 'reverse';
    axBody.YDir = 'reverse';
    axis(axBody,[-0.9 0.9 -0.9 0.9 -0.3 0.3]);
    daspect(axBody,[1 1 1]);
    xlabel(axBody,'N'); ylabel(axBody,'E'); zlabel(axBody,'D');
    
    % Signals panel (bottom-right)
    rightPanel = uipanel(leftGrid, 'Title','Signals', 'BorderType','line'); 
    rightPanel.Layout.Row = 2; 
    rightPanel.Layout.Column = 2;
    rightPanel.Scrollable = 'on';   % useful if content overflows
    
    % 8 rows x 4 columns grid for variables
    readGrid = uigridlayout(rightPanel,[8 4]);
    readGrid.RowHeight   = repmat({'1x'},1,8);
    readGrid.ColumnWidth = repmat({'1x'},1,4);
    readGrid.Padding     = [10 10 10 10];
    readGrid.RowSpacing  = 6;
    readGrid.ColumnSpacing = 10;
    
    % Example items (keep or replace later)
    lblTime = uilabel(readGrid, 'Text','t = 0.000 s', 'FontSize',12, 'FontWeight','bold');
    lblTime.Layout.Row = 1; 
    lblTime.Layout.Column = 1;   % spans all columns
    
    lblVel  = uilabel(readGrid, 'Text','AS = 0.00 m/s', 'FontSize',12, 'FontWeight','bold');
    lblVel.Layout.Row = 2; 
    lblVel.Layout.Column = 1; 
    
    lblPhi  = uilabel(readGrid, 'Text','ϕ = 0.0°', 'FontSize',12, 'FontWeight','bold');
    lblPhi.Layout.Row = 3; 
    lblPhi.Layout.Column = 1;
    
    lblTheta  = uilabel(readGrid, 'Text','θ = 0.0°', 'FontSize',12, 'FontWeight','bold');
    lblTheta.Layout.Row = 4; 
    lblTheta.Layout.Column = 1;
    
    lblPsi  = uilabel(readGrid, 'Text','ψ = 0.0°', 'FontSize',12, 'FontWeight','bold');
    lblPsi.Layout.Row = 5; 
    lblPsi.Layout.Column = 1;
    
    
    % ---- Controls row (bottom) ----
    controls = uigridlayout(top,[1 2]); 
    controls.Layout.Row = 2; 
    controls.ColumnWidth = {100, '1x'};     % buttons fixed, slider expands
    controls.Padding = [10 10 10 10];
    controls.ColumnSpacing = 10;
    
    btn  = uibutton(controls,'state','Text','▶ Play', ...
                    'ValueChangedFcn',@(b,~)togglePlay(b));
    btn.Layout.Row = 1; btn.Layout.Column = 1;
    
    sldTime = uislider(controls, ...
        'Limits',[t_vis(1) t_vis(end)], 'Value', t_vis(1), ...
        'MajorTicks',[], 'MinorTicks',[], ...
        'ValueChangingFcn', @(s,ev) onSliderChanging(ev.Value), ...
        'ValueChangedFcn',  @(s,~)  onSliderChanged(s.Value));
    sldTime.Layout.Row = 1; 
    sldTime.Layout.Column = 2;
    
    % Markers
    hPt  = plot3(axWorld, pActual(IDX(1),1), pActual(IDX(1),2), pActual(IDX(1),3), ...
                 'o','MarkerSize',6,'LineWidth',1,'Color',[0 0 0]);
    
    hTrail = plot3(axWorld, nan, nan, nan, 'o', 'MarkerSize',1, ...
                   'MarkerFaceColor','k', 'MarkerEdgeColor','k', 'LineStyle','none');
    
    if hasRef && ~isempty(pRef)
        i0_2  = min(IDX(1), size(pRef,1));
        hPt2  = plot3(axWorld, pRef(i0_2,1), pRef(i0_2,2), pRef(i0_2,3), ...
                      'o', 'MarkerSize',5, 'MarkerFaceColor','none', ...
                      'MarkerEdgeColor','b', 'LineStyle','none');
    else
        hPt2  = plot3(axWorld, nan, nan, nan, 'o', 'MarkerSize',5, ...
                      'MarkerFaceColor','b', 'MarkerEdgeColor','b');
    end
    
    %% STL model
    stlFile = 'Cyclone2.stl';
    [F0,V0] = local_read_stl(stlFile);
    centroid = mean(V0,1); V0c = V0 - centroid;
    box  = max(V0c) - min(V0c); unit = max(box); if unit==0, unit=1; end
    Vunit = V0c / unit;
    
    Rb0 = eul2rotm([0 0 0], 'ZXY');
    Vunit_rb = (Rb0 * Vunit.').';
    
    % Scales from PNED extents
    rangeN = max(pActual(:,1)) - min(pActual(:,1));
    rangeE = max(pActual(:,2)) - min(pActual(:,2));
    rangeD = max(pActual(:,3)) - min(pActual(:,3));
    sceneSize = max([rangeN, rangeE, rangeD]); if sceneSize==0, sceneSize=1; end
    scale_world = max(0.1, min(0.08*sceneSize, 30.0));
    scale_body  = 0.9;
    
    V_world = Vunit_rb * scale_world;
    V_body  = Vunit_rb * scale_body;
    
    % Transforms and patches
    hT_world = hgtransform('Parent',axWorld);
    hT_body  = hgtransform('Parent',axBody);
    
    patch('Parent',hT_world,'Faces',F0,'Vertices',V_world, ...
          'FaceColor',[0.8500 0.3250 0.0980], 'EdgeColor','none', ...
          'FaceAlpha',1, 'FaceLighting','flat', 'AmbientStrength',0.8);
    
    patch('Parent',hT_body,'Faces',F0,'Vertices',V_body, ...
          'FaceColor',[0.35 0.65 0.95],'EdgeColor','none','FaceAlpha',1.0);
    
    % Axes limits (ensure origin visible)
    pad = 0.1*sceneSize + scale_world;
    xmin = min(pActual(:,1)); xmax = max(pActual(:,1));
    ymin = min(pActual(:,2)); ymax = max(pActual(:,2));
    zmin = min(pActual(:,3)); zmax = max(pActual(:,3));
    xlim(axWorld,[min(xmin,0)-pad, max(xmax,0)+pad]);
    ylim(axWorld,[min(ymin,0)-pad, max(ymax,0)+pad]);
    zlim(axWorld,[min(zmin,0)-pad, max(zmax,0)+pad]);
    
    % Lighting
    try
        light(axWorld,'Style','infinite'); lighting(axWorld,'gouraud');
        light(axBody, 'Style','infinite'); lighting(axBody, 'gouraud');
    catch, end
    
    % Body axes triad
    plot3(axBody,[0 0.7],[0 0],[0 0],'LineWidth',2,'LineStyle',':');
    plot3(axBody,[0 0],[0 0.7],[0 0],'LineWidth',2,'LineStyle',':');
    plot3(axBody,[0 0],[0 0],[0 0.7],'LineWidth',2,'LineStyle',':');
    axisLen = 0.6;
    line('Parent',hT_body,'XData',[0 0],'YData',[0 0],'ZData',[0 -axisLen], ...
         'LineWidth',2,'Color',[0 0.4470 0.7410]);
    line('Parent',hT_body,'XData',[0 0],'YData',[0 axisLen],'ZData',[0 0], ...
         'LineWidth',2,'Color',[0.8500 0.3250 0.0980]);
    line('Parent',hT_body,'XData',[0 axisLen],'YData',[0 0],'ZData',[0 0], ...
         'LineWidth',2,'Color',[0.9290 0.6940 0.1250]);
    
    % Fixed STL alignment
    Rfix = eul2rotm([0, 0, -pi/2], 'ZXY');
    
    % Person at NED origin
    draw_person_ned(axWorld, [0 0 0], 1.75);
    
    %% Playback state & init
    kFrame = 1;
    M = numel(t_vis);
    curTime = t_vis(kFrame);
    isPlaying = false; tmr = [];
    isScrubbing = false;
    isProgrammatic = false;
    
    moveToK(1);

%% Nested functions
    function togglePlay(b)
        if b.Value, b.Text='⏸ Pause'; startPlay();
        else,       b.Text='▶ Play';  stopPlay(); end
    end

    function startPlay()
        if isPlaying, return; end
        isPlaying = true;
        if isempty(tmr) || ~isvalid(tmr)
            tmr = timer('ExecutionMode','fixedSpacing','Period',dt_vis, ...
                        'TimerFcn',@(~,~) stepForward(),'StartDelay',0);
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

    function onSliderChanging(val)
        if isProgrammatic, return; end
        isScrubbing = true;
        if isPlaying
            stopPlay(); btn.Value=false; btn.Text='▶ Play';
        end
        moveToTime(val);
    end

    function onSliderChanged(val)
        if isProgrammatic, return; end
        moveToTime(val);
        isScrubbing = false;
    end

    function moveToTime(tv)
        tv = max(min(tv, t_vis(end)), t_vis(1));
        [~, k] = min(abs(t_vis - tv));
        moveToK(k);
    end

    function moveToK(k)
        kFrame  = k;
        curTime = t_vis(kFrame);
        iFrame  = IDX(kFrame);

        % Orientation & transforms
        Rtot = Rall(:,:,iFrame) * Rfix;
        T1 = eye(4); T1(1:3,1:3)=Rtot; T1(1:3,4)=pActual(iFrame,:).';
        set(hT_world,'Matrix',T1);
        T2 = eye(4); T2(1:3,1:3)=Rtot; set(hT_body,'Matrix',T2);

        % Markers
        set(hPt,'XData',pActual(iFrame,1),'YData',pActual(iFrame,2),'ZData',pActual(iFrame,3));

        % Trail
        kStart = max(1, kFrame - (Mtrail - 1));
        ptsIdx = IDX(kStart:kFrame);
        Ptrail = pActual(ptsIdx,:);
        set(hTrail,'XData',Ptrail(:,1),'YData',Ptrail(:,2),'ZData',Ptrail(:,3));

        % Ref marker
        if hasRef && ~isempty(pRef)
            iFrame2 = min(iFrame, size(pRef,1));
            set(hPt2,'XData',pRef(iFrame2,1),'YData',pRef(iFrame2,2),'ZData',pRef(iFrame2,3));
        end

        % Update readouts
        lblTime.Text = sprintf('t = %.3f s', curTime);
        lblVel.Text  = sprintf('AS = %.2f m/s', ASActual(iFrame));
        lblPhi.Text = sprintf('ϕ = %.1f°', rad2deg(zxy(iFrame, 2)));
        lblTheta.Text = sprintf('θ = %.1f°', rad2deg(zxy(iFrame, 3)));
        lblPsi.Text = sprintf('ψ = %.1f°', rad2deg(zxy(iFrame, 1)));

        % Keep slider in sync (without re-triggering)
        if ~isScrubbing
            isProgrammatic = true;
            sldTime.Value = curTime;
            isProgrammatic = false;
        end

        drawnow limitrate
    end
end % end main function

%% Helpers
function [F,V] = local_read_stl(stlFile)
    try
        tr = stlread(stlFile);
        F = tr.ConnectivityList; V = tr.Points;
    catch
        [F,V] = stlread(stlFile);
    end
end

function draw_person_ned(ax, originNED, height)
    if nargin < 3, height = 1.75; end
    N0 = originNED(1); E0 = originNED(2); D0 = originNED(3);
    
    rHead   = 0.12*height/1.75;
    legL    = 0.52*height;
    torsoL  = 0.38*height;
    neckL   = 0.02*height;
    shouldW = 0.26*height;
    hipW    = 0.18*height;
    
    D_foot   = D0;
    D_hip    = D_foot - legL;
    D_neck   = D_hip  - torsoL;
    D_head_c = D_neck - neckL - rHead;
    
    N_left  = N0 - shouldW/2; N_right = N0 + shouldW/2;
    N_hipL  = N0 - hipW/2;    N_hipR  = N0 + hipW/2;
    
    colLine = [0.2 0.2 0.2];
    hold(ax,'on');
    
    plot3(ax,[N_hipL N0],[E0 E0],[D_hip D_foot],'LineWidth',2,'Color',colLine);
    plot3(ax,[N_hipR N0],[E0 E0],[D_hip D_foot],'LineWidth',2,'Color',colLine);
    plot3(ax,[N0 N0],[E0 E0],[D_hip D_neck],'LineWidth',3,'Color',colLine);
    D_sh = D_hip - 0.7*torsoL;
    plot3(ax,[N_left N_right],[E0 E0],[D_sh D_sh],'LineWidth',2,'Color',colLine);
    
    [XS,YS,ZS] = sphere(20);
    XS = N0 + rHead*XS;
    YS = E0 + rHead*YS;
    ZS = D_head_c + rHead*ZS;
    surf(ax,XS,YS,ZS,'FaceColor',[0.6 0.6 0.6],'EdgeColor','none','FaceAlpha',1);
end
