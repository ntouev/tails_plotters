function f = traj_vis_static(varargin)

    if nargin == 3
        ref        = varargin{1};
        actual     = varargin{2};
        title_name = varargin{3};
    else
        error('Usage: traj_vis_static(ref, actual, name)');
    end

    f = figure('Name', title_name);
    plot3(ref.p(:,1), ref.p(:,2), ref.p(:,3));
    hold on;
    plot3(actual.p(:,1), actual.p(:,2), actual.p(:,3));
    
    All_Data = [actual.p(:); ref.p(:)];
   
    mins = min(All_Data);
    maxs = max(All_Data);
    
    xlim([mins maxs]);
    ylim([mins maxs]);
    zlim([mins maxs]);
    
    xlim([mins maxs]);
    ylim([mins maxs]);
    zlim([mins maxs]);
    
    xlabel('N [m]');
    ylabel('E[m]');
    zlabel('D[m]');

    set(gca,'ZDir','reverse');
    set(gca,'YDir','reverse');

    axis vis3d;
    grid on;
    rotate3d on;

end