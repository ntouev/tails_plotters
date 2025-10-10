function vis(ac_data, fs_vis)
    % add tranges

    ref = struct(); 
    actual = struct();
    
    quat = double([ac_data.AHRS_REF_QUAT.body_qi ac_data.AHRS_REF_QUAT.body_qx ac_data.AHRS_REF_QUAT.body_qy ac_data.AHRS_REF_QUAT.body_qz]);
    refquat = double([ac_data.AHRS_REF_QUAT.ref_qi ac_data.AHRS_REF_QUAT.ref_qx ac_data.AHRS_REF_QUAT.ref_qy ac_data.AHRS_REF_QUAT.ref_qz]);
    [refquat_t,irefquat_t,~] = unique(ac_data.AHRS_REF_QUAT.timestamp);
    quat = quat(irefquat_t,:);
    refquat = refquat(irefquat_t,:);

    pos = [ac_data.ROTORCRAFT_FP.north_alt ac_data.ROTORCRAFT_FP.east_alt -ac_data.ROTORCRAFT_FP.up_alt];
         
    t_vis  = (ac_data.AHRS_REF_QUAT.timestamp(1) : 1/fs_vis : ac_data.AHRS_REF_QUAT.timestamp(end))';
    quat = interp1(ac_data.AHRS_REF_QUAT.timestamp, quat, t_vis, 'linear', 'extrap');
    pos = interp1(ac_data.ROTORCRAFT_FP.timestamp, pos, t_vis, "linear", "extrap");

    N = length(t_vis);

    ref.t = t_vis;
    ref.p = zeros(N,3);
    ref.q = [ones(N,1) zeros(N,1)];
   
    actual.t = t_vis;
    actual.p = pos;
    actual.q = quat;

    motion_vis(ref, actual, fs_vis);
end