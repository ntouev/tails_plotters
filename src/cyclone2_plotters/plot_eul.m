function plot_eul(ac_data, order)

    % Plot the Euler angles
    msg = ac_data.AHRS_REF_QUAT;

    quat = double([msg.body_qi msg.body_qx msg.body_qy msg.body_qz]);
    refquat = double([msg.ref_qi msg.ref_qx msg.ref_qy msg.ref_qz]);
    [refquat_t,irefquat_t,~] = unique(msg.timestamp);
    quat = quat(irefquat_t,:);
    refquat = refquat(irefquat_t,:);

    if strcmp(order,'ZXY')
        [psi, phi, theta] = quat2angle(quat,order);
        [refpsi, refphi, reftheta] = quat2angle(refquat,order);
    elseif strcmp(order,'ZYX')
        [psi, theta, phi] = quat2angle(quat,order);
        [refpsi, reftheta, refphi] = quat2angle(refquat,order);
    else
        disp('Rotation order not available')
    end

    tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

    ax1 = nexttile;
    hold on; zoom on;
    h1 = plot(refquat_t, rad2deg(refphi), LineWidth=1.5);
    h2 = plot(refquat_t, rad2deg(phi), LineWidth=1.5);
    xlabel('Time [s]');
    ylabel('\phi [deg]');
    title('\phi');
    grid on;

    ax2 = nexttile;
    hold on; zoom on;
    h3 = plot(refquat_t, rad2deg(reftheta), LineWidth=1.5);
    h4 = plot(refquat_t, rad2deg(theta), LineWidth=1.5);
    xlabel('Time [s]');
    ylabel('\theta [deg]');
    title('\theta');
    grid on;

    ax3 = nexttile;
    hold on; zoom on;
    h5 = plot(refquat_t, rad2deg(refpsi), LineWidth=1.5);
    h6 = plot(refquat_t, rad2deg(psi), LineWidth=1.5);
    xlabel('Time [s]');
    ylabel('\psi [deg]');
    title('\psi');
    grid on;

    % flight modes
    mode_values = ac_data.ROTORCRAFT_RADIO_CONTROL.mode;
    mode_timestamps = ac_data.ROTORCRAFT_RADIO_CONTROL.timestamp;
    draw_mode_transitions(mode_values, mode_timestamps, {ax1, ax2, ax3});
    legend(ax1, [h1, h2], {'\phi ref', '\phi'});
    legend(ax2, [h3, h4], {'\theta ref', '\theta'});
    legend(ax3, [h5, h6], {'\psi ref', '\psi'});

    linkaxes([ax1,ax2,ax3],'x');

end