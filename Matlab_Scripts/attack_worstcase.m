clc; close all; clear all;

ctrl_type = 3;
channel   = 'ref';
objective = 'xmax';

Kp = 143.5032;  Kd = 0.0264;  Kd2 = 2.1e-6;  Ki = 422.5903;  Ki2 = 321.3681;
z  = 5.3716;    p  = 47.6495;

u_amp_lim     = 'n';
u_amp_lim_up  = 150;
u_amp_lim_low = -150;

grid_A  = [0.02 0.05 0.1 0.2 0.5 1.0];
grid_f  = [0.5 1 5 10 50 100 500 1000 5000];
grid_t0 = [0 0.1 0.25 0.5];

t_start = 0;  t_stop = 1;  dt = 0.00001;
N = floor((t_stop - t_start)/dt) + 1;
t = t_start + (0:N-1)*dt;

best = -inf;
best_prm = struct('A',0,'f',0,'t0',0);
count = 0;

for A = grid_A
    for f = grid_f
        for t0 = grid_t0
            count = count + 1;
            prm.A = A;  prm.f = f;  prm.t0 = t0;  prm.phi = 0;
            a = attack_lib('sine', t, N, dt, prm);

            if strcmp(channel, 'ref')
                a_r = a;  a_s = zeros(1, N);
            else
                a_r = zeros(1, N);  a_s = a;
            end

            t_info = run_attack_sim(ctrl_type, Kp, Kd, Kd2, Ki, Ki2, z, p, ...
                                    a_r, a_s, u_amp_lim, u_amp_lim_up, u_amp_lim_low);

            switch objective
                case 'Eu',    val = t_info(4);
                case 'IAE',   val = t_info(2);
                case 'KuIAE', val = t_info(7);
                case 'xmax'

                    val = t_info(2);
            end

            if val > best
                best = val;
                best_prm = prm;
            end
        end
    end
end

clc;
fprintf('--------------------------------------------------\n');
fprintf('  Worst-case attack search (grid)\n');
fprintf('--------------------------------------------------\n');
fprintf('  Controller ID : %d\n', ctrl_type);
fprintf('  Channel       : %s\n', channel);
fprintf('  Objective     : %s\n', objective);
fprintf('  Candidates    : %d\n', count);
fprintf('--------------------------------------------------\n');
fprintf('  Worst %s = %.6f\n', objective, best);
fprintf('  at A = %.3f, f = %.1f Hz, t0 = %.2f s\n', best_prm.A, best_prm.f, best_prm.t0);
fprintf('--------------------------------------------------\n');
