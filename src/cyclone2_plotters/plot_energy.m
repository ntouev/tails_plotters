function plot_energy(ac_data)
    
    current = [single(ac_data.SERIAL_ACT_T4_IN.motor_1_current_int)/100, ...
               single(ac_data.SERIAL_ACT_T4_IN.motor_2_current_int)/100];
    voltage = [single(ac_data.SERIAL_ACT_T4_IN.motor_1_voltage_int)/100, ...
               single(ac_data.SERIAL_ACT_T4_IN.motor_2_voltage_int)/100];
    power = voltage.*current;

    tot_motor_current = current(:,1) + current(:,2);
    avg_motor_voltage = (voltage(:,1) + voltage(:,2))/2;

    tiledlayout(2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

    ax1 = nexttile;
    hold on; grid on; zoom on;
    h1 = plot(ac_data.SERIAL_ACT_T4_IN.timestamp, power(:,1), LineWidth=1.5);
    h2 = plot(ac_data.SERIAL_ACT_T4_IN.timestamp, power(:,2), LineWidth=1.5);
    xlabel('Time [s]');
    ylabel('');
    title('Motor power');

    ax2 = nexttile;
    hold on; grid on; zoom on;
    yyaxis left;
    h3 = plot(ac_data.SERIAL_ACT_T4_IN.timestamp, avg_motor_voltage, LineWidth=1.5);
    ylabel('Average voltage [V]');
    yyaxis right;
    h4 = plot(ac_data.SERIAL_ACT_T4_IN.timestamp, tot_motor_current, LineWidth=1.5);
    xlabel('Time [s]');
    ylabel('Total current [A]');
    title('Both motors');

    % flight modes
    mode_values = ac_data.ROTORCRAFT_RADIO_CONTROL.mode;
    mode_timestamps = ac_data.ROTORCRAFT_RADIO_CONTROL.timestamp;
    draw_mode_transitions(mode_values, mode_timestamps, {ax1, ax2});
    legend(ax1, [h3, h4], {'Power 1', 'Power 2'});

    linkaxes([ax1,ax2],'x');
end