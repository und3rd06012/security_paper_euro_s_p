addpath(pwd);

dt = 0.00001;
N  = floor(1/dt)+1;
t  = (0:N-1)*dt;
X_SAFE = 20;

fprintf('=== spot 1: ID6 lead-integrator, sensor bias -5 @0.1s ===\n');
a_s = attack_lib('bias', t, N, dt, struct('A', -5, 't0', 0.1));
[t_info,x,u,err,ctrl_state,u_amp_lim_state,xm] = ...
    run_attack_sim(6, 125.3641, 0, 0, 0, 0, 5.3716, 47.6495, zeros(1,N), a_s, 'n', 150, -150);
xmax = max(abs(x)); esc_idx = find(abs(x) > X_SAFE | ~isfinite(x), 1);
if isempty(esc_idx), esc_str = 'safe'; else, esc_str = sprintf('ESC @ %.3f s', t(esc_idx)); end
fprintf('   xmax = %.2f, %s, IAE = %.4e\n', xmax, esc_str, t_info(2));

fprintf('=== spot 2: ID3 static PID, ref sine 500 Hz, defence D2 (filter+limiter) ===\n');
a_r = attack_lib('sine', t, N, dt, struct('A', 0.25, 'f', 500, 't0', 0));
[t_info,x,u,err,ctrl_state,u_amp_lim_state,xm] = ...
    run_attack_sim_def(3, 143.5032, 0.0264, 0, 422.5903, 0, 0, 0, a_r, zeros(1,N), 'y', 'y', 'n');
xmax = max(abs(x)); esc_idx = find(abs(x) > X_SAFE | ~isfinite(x), 1);
if isempty(esc_idx), esc_str = 'safe'; else, esc_str = sprintf('ESC @ %.3f s', t(esc_idx)); end
fprintf('   xmax = %.2f, %s, IAE = %.4e, Eu = %.1f, K = %.4e\n', xmax, esc_str, t_info(2), t_info(4), t_info(7));

fprintf('=== spot 3: ID0 switching PID, ref sine 500 Hz, defence D3 (filter+limiter+dwell) ===\n');
a_r = attack_lib('sine', t, N, dt, struct('A', 0.25, 'f', 500, 't0', 0));
[t_info,x,u,err,ctrl_state,u_amp_lim_state,xm] = ...
    run_attack_sim_def(0, 274.7391, 0.0461, 0, 334.0568, 0, 0, 0, a_r, zeros(1,N), 'y', 'y', 'y');
sw = sum(diff(ctrl_state) ~= 0);
xmax = max(abs(x)); esc_idx = find(abs(x) > X_SAFE | ~isfinite(x), 1);
if isempty(esc_idx), esc_str = 'safe'; else, esc_str = sprintf('ESC @ %.3f s', t(esc_idx)); end
fprintf('   xmax = %.2f, %s, IAE = %.4e, Eu = %.1f, K = %.4e, sw = %d\n', xmax, esc_str, t_info(2), t_info(4), t_info(7), sw);

fprintf('DONE.\n');
