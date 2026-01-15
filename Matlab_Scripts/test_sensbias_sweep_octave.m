addpath(pwd);

dt = 0.00001;
N  = floor(1/dt)+1;
t  = (0:N-1)*dt;
X_SAFE = 20;

ctrl(1).Kp = 274.7391; ctrl(1).Kd = 0.0461; ctrl(1).Kd2 = 0;        ctrl(1).Ki = 334.0568; ctrl(1).Ki2 = 0; ctrl(1).z = 0; ctrl(1).p = 0;
ctrl(2).Kp = 0;        ctrl(2).Kd = 0;      ctrl(2).Kd2 = 0;        ctrl(2).Ki = 0;        ctrl(2).Ki2 = 0; ctrl(2).z = 0; ctrl(2).p = 0;
bb_up = 46.5457; bb_low = -45.4420;
ctrl(3).Kp = 274.9566; ctrl(3).Kd = 0.0263; ctrl(3).Kd2 = 0;        ctrl(3).Ki = 293.1913; ctrl(3).Ki2 = 0; ctrl(3).z = 0; ctrl(3).p = 0;
ctrl(4).Kp = 143.5032; ctrl(4).Kd = 0.0264; ctrl(4).Kd2 = 0;        ctrl(4).Ki = 422.5903; ctrl(4).Ki2 = 0; ctrl(4).z = 0; ctrl(4).p = 0;
ctrl(5).Kp = 163.4738; ctrl(5).Kd = 0.0257; ctrl(5).Kd2 = 2.1e-6;   ctrl(5).Ki = 423.3579; ctrl(5).Ki2 = 321.3681; ctrl(5).z = 0; ctrl(5).p = 0;
ctrl(6).Kp = 272.2565; ctrl(6).Kd = 0.0306; ctrl(6).Kd2 = 0;        ctrl(6).Ki = 421.5556; ctrl(6).Ki2 = 0; ctrl(6).z = 0; ctrl(6).p = 0;
ctrl(7).Kp = 125.3641; ctrl(7).Kd = 0;      ctrl(7).Kd2 = 0;        ctrl(7).Ki = 0;        ctrl(7).Ki2 = 0; ctrl(7).z = 5.3716; ctrl(7).p = 47.6495;

amps = [-0.02 -0.05 -0.1 -0.2 -0.5 -1 -2 -5];

fprintf('=== EXP A: sensor-bias escape sweep (t0=0.1 s, |x|<20 safe) ===\n');
for id = 0:6
    fprintf('ID %d: ', id);
    for A = amps
        a_s = attack_lib('bias', t, N, dt, struct('A', A, 't0', 0.1));
        if id == 1
            [t_info,x,u,err,ctrl_state,u_amp_lim_state,xm] = ...
                run_attack_sim(1, 0,0,0,0,0,0,0, zeros(1,N), a_s, 'n', bb_up, bb_low);
        else
            [t_info,x,u,err,ctrl_state,u_amp_lim_state,xm] = ...
                run_attack_sim(id, ctrl(id+1).Kp, ctrl(id+1).Kd, ctrl(id+1).Kd2, ...
                               ctrl(id+1).Ki, ctrl(id+1).Ki2, ctrl(id+1).z, ctrl(id+1).p, ...
                               zeros(1,N), a_s, 'n', 150, -150);
        end
        xmax = max(abs(x));
        esc_idx = find(abs(x) > X_SAFE | ~isfinite(x), 1);
        if isempty(esc_idx), esc_str = 'ok'; else, esc_str = sprintf('ESC@%.3f', t(esc_idx)); end
        fprintf(' A=%.2f:x=%.2f%s ', A, xmax, esc_str);
    end
    fprintf('\n');
end
fprintf('DONE.\n');
