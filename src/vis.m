function vis(ac_data, fs_vis)
    
    ref = struct(); 
    actual = struct();
    
    [tq, q, qref] = get_quat(ac_data);

    p = [ac_data.ROTORCRAFT_FP.north_alt ac_data.ROTORCRAFT_FP.east_alt -ac_data.ROTORCRAFT_FP.up_alt];
         
    t_vis  = (tq(1) : 1/fs_vis : tq(end))';
    q = interp1(tq, q, t_vis, 'linear', 'extrap');
    qref = interp1(tq, qref, t_vis, 'linear', 'extrap');
    p = interp1(ac_data.ROTORCRAFT_FP.timestamp, p, t_vis, "linear", "extrap");

    N = length(t_vis);

    ref.t = t_vis;
    ref.p = zeros(N,3);
    ref.q = qref;
   
    actual.t = t_vis;
    actual.p = p;
    actual.q = q;

    traj_vis(ref, actual, fs_vis);
end