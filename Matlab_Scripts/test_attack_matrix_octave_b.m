addpath(pwd);

dt = 0.00001;
N  = floor(1/dt)+1;
t  = (0:N-1)*dt;
X_SAFE = 20;

ctrl(5).Kp = 163.4738; ctrl(5).Kd = 0.0257; ctrl(5).Kd2 = 2.1e-6;   ctrl(5).Ki = 423.3579; ctrl(5).Ki2 = 321.3681; ctrl(5).z = 0; ctrl(5).p = 0;
ctrl(6).Kp = 272.2565; ctrl(6).Kd = 0.0306; ctrl(6).Kd2 = 0;        ctrl(6).Ki = 421.5556; ctrl(6).Ki2 = 0; ctrl(6).z = 0; ctrl(6).p = 0;
ctrl(7).Kp = 125.3641; ctrl(7).Kd = 0;      ctrl(7).Kd2 = 0;        ctrl(7).Ki = 0;        ctrl(7).Ki2 = 0; ctrl(7).z = 5.3716; ctrl(7).p = 47.6495;

atk(1).name = 'clean';           atk(1).chan = 'none'; atk(1).type = 'none';
atk(2).name = 'ref sine 500Hz';  atk(2).chan = 'ref';  atk(2).type = 'sine'; atk(2).A = 0.25; atk(2).f = 500; atk(2).t0 = 0;
atk(3).name = 'sens sine 100Hz'; atk(3).chan = 'sens'; atk(3).type = 'sine'; atk(3).A = 0.05; atk(3).f = 100; atk(3).t0 = 0;

fprintf('============================================================\n');
fprintf(' ATTACK MATRIX (IDs 4-6)  (no limiter, no filter, |x|<20 safe)\n');
fprintf('============================================================\n');
fprintf('%-2s | %-16s | %-6s | %-6s | %-10s | %-10s | %-10s\n', ...
        'ID','attack','xmax','Tesc','IAE','Eu','K_uIAE');

for id = 4:6
    for k = 1:3
        a_r = zeros(1,N); a_s = zeros(1,N);
        if ~strcmp(atk(k).chan,'none')
            prm.A = atk(k).A; prm.t0 = atk(k).t0; prm.phi = 0;
            if isfield(atk(k),'f'), prm.f = atk(k).f; end
            a = attack_lib(atk(k).type, t, N, dt, prm);
            if strcmp(atk(k).chan,'ref'), a_r = a; else, a_s = a; end
        end

        [t_info,x,u,err,ctrl_state,u_amp_lim_state,xm] = ...
            run_attack_sim(id, ctrl(id+1).Kp, ctrl(id+1).Kd, ctrl(id+1).Kd2, ...
                           ctrl(id+1).Ki, ctrl(id+1).Ki2, ctrl(id+1).z, ctrl(id+1).p, ...
                           a_r, a_s, 'n', 150, -150);

        xmax = max(abs(x));
        esc_idx = find(abs(x) > X_SAFE | ~isfinite(x), 1);
        if isempty(esc_idx), esc_str = 'safe'; else, esc_str = sprintf('%.3f', t(esc_idx)); end

        fprintf('%-2d | %-16s | %6.2f | %-6s | %.4e | %9.1f | %.4e\n', ...
                id, atk(k).name, xmax, esc_str, t_info(2), t_info(4), t_info(7));
    end
    fprintf('----\n');
end
fprintf('DONE.\n');
