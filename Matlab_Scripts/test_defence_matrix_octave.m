addpath(pwd);

dt = 0.00001;
N  = floor(1/dt)+1;
t  = (0:N-1)*dt;
X_SAFE = 20;

ctrl(1).Kp = 274.7391; ctrl(1).Kd = 0.0461; ctrl(1).Kd2 = 0;        ctrl(1).Ki = 334.0568; ctrl(1).Ki2 = 0; ctrl(1).z = 0; ctrl(1).p = 0;
ctrl(2).Kp = 0;        ctrl(2).Kd = 0;      ctrl(2).Kd2 = 0;        ctrl(2).Ki = 0;        ctrl(2).Ki2 = 0; ctrl(2).z = 0; ctrl(2).p = 0;
ctrl(3).Kp = 274.9566; ctrl(3).Kd = 0.0263; ctrl(3).Kd2 = 0;        ctrl(3).Ki = 293.1913; ctrl(3).Ki2 = 0; ctrl(3).z = 0; ctrl(3).p = 0;
ctrl(4).Kp = 143.5032; ctrl(4).Kd = 0.0264; ctrl(4).Kd2 = 0;        ctrl(4).Ki = 422.5903; ctrl(4).Ki2 = 0; ctrl(4).z = 0; ctrl(4).p = 0;
ctrl(5).Kp = 163.4738; ctrl(5).Kd = 0.0257; ctrl(5).Kd2 = 2.1e-6;   ctrl(5).Ki = 423.3579; ctrl(5).Ki2 = 321.3681; ctrl(5).z = 0; ctrl(5).p = 0;
ctrl(6).Kp = 272.2565; ctrl(6).Kd = 0.0306; ctrl(6).Kd2 = 0;        ctrl(6).Ki = 421.5556; ctrl(6).Ki2 = 0; ctrl(6).z = 0; ctrl(6).p = 0;
ctrl(7).Kp = 125.3641; ctrl(7).Kd = 0;      ctrl(7).Kd2 = 0;        ctrl(7).Ki = 0;        ctrl(7).Ki2 = 0; ctrl(7).z = 5.3716; ctrl(7).p = 47.6495;

atk(1).name = 'ref bias +0.5';   atk(1).chan = 'ref';  atk(1).type = 'bias'; atk(1).A = 0.5;  atk(1).t0 = 0.1;
atk(2).name = 'ref sine 500Hz';  atk(2).chan = 'ref';  atk(2).type = 'sine'; atk(2).A = 0.25; atk(2).f = 500; atk(2).t0 = 0;
atk(3).name = 'sens sine 100Hz'; atk(3).chan = 'sens'; atk(3).type = 'sine'; atk(3).A = 0.05; atk(3).f = 100; atk(3).t0 = 0;

def(1).name = 'D1 filter only';             def(1).filter = 'y'; def(1).lim = 'n'; def(1).dwell = 'n';
def(2).name = 'D2 filter + limiter';        def(2).filter = 'y'; def(2).lim = 'y'; def(2).dwell = 'n';
def(3).name = 'D3 filter + limiter + dwell';def(3).filter = 'y'; def(3).lim = 'y'; def(3).dwell = 'y';

fprintf('=== EXP B: defence stack matrix ===\n');
for d = 1:3
    fprintf('\n--- %s ---\n', def(d).name);
    for id = 0:6
        for k = 1:3
            a_r = zeros(1,N); a_s = zeros(1,N);
            if ~strcmp(atk(k).chan,'none')
                prm.A = atk(k).A; prm.t0 = atk(k).t0; prm.phi = 0;
                if isfield(atk(k),'f'), prm.f = atk(k).f; end
                a = attack_lib(atk(k).type, t, N, dt, prm);
                if strcmp(atk(k).chan,'ref'), a_r = a; else, a_s = a; end
            end
            [t_info,x,u,err,ctrl_state,u_amp_lim_state,xm] = ...
                run_attack_sim_def(id, ctrl(id+1).Kp, ctrl(id+1).Kd, ctrl(id+1).Kd2, ...
                                   ctrl(id+1).Ki, ctrl(id+1).Ki2, ctrl(id+1).z, ctrl(id+1).p, ...
                                   a_r, a_s, def(d).filter, def(d).lim, def(d).dwell);
            xmax = max(abs(x));
            esc_idx = find(abs(x) > X_SAFE | ~isfinite(x), 1);
            if isempty(esc_idx), esc_str = 'safe'; else, esc_str = sprintf('ESC@%.3f', t(esc_idx)); end
            if id == 1
                sw = sum(diff(u(1:N-1)) ~= 0);
            elseif id == 0 || id == 2
                sw = sum(diff(ctrl_state) ~= 0);
            else
                sw = NaN;
            end
            if isnan(sw), sw_str = 'n/a'; else, sw_str = sprintf('%d', sw); end
            fprintf('%d | %-15s | x=%.2f %-8s | IAE=%.4e | Eu=%9.1f | K=%.4e | sw=%s\n', ...
                    id, atk(k).name, xmax, esc_str, t_info(2), t_info(4), t_info(7), sw_str);
        end
    end
end
fprintf('DONE.\n');
