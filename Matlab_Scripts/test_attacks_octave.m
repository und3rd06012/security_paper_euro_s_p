addpath(pwd);

dt = 0.00001;
N  = floor((1-0)/dt)+1;
t  = (0:N-1)*dt;

Kp0 = 274.7391; Kd0 = 0.0461;  Ki0 = 334.0568;
Kp3 = 143.5032; Kd3 = 0.0264;  Ki3 = 422.5903;
bb_up = 46.5457; bb_low = -45.4420;
Kd2 = 2.1e-6; Ki2 = 321.3681; z = 5.3716; p = 47.6495;

fprintf('============================================================\n');
fprintf(' 1. CLEAN VALIDATION (paper Scenario 1)\n');
fprintf('============================================================\n');

ti = run_attack_sim(3, Kp3, Kd3, Kd2, Ki3, Ki2, z, p, zeros(1,N), zeros(1,N), 'n', 150, -150);
fprintf(' ID3 static PID : Exit=%.4e IAE=%.4e Eu=%.2f KuIAE=%.4e | paper: 1.109e-4 1.305e-3 1534.79 0.07035\n', ti(1), ti(2), ti(4), ti(7));

ti = run_attack_sim(0, Kp0, Kd0, Kd2, Ki0, Ki2, z, p, zeros(1,N), zeros(1,N), 'n', 150, -150);
fprintf(' ID0 switch PID : Exit=%.4e IAE=%.4e Eu=%.2f KuIAE=%.4e | paper: 1.434e-4 6.354e-4 1535.13 0.03203\n', ti(1), ti(2), ti(4), ti(7));

ti = run_attack_sim(1, 0, 0, 0, 0, 0, 0, 0, zeros(1,N), zeros(1,N), 'n', bb_up, bb_low);
fprintf(' ID1 bang-bang  : Exit=%.4e IAE=%.4e Eu=%.2f KuIAE=%.4e | paper: 3.307e-4 0.3077 2113.92 14.14\n', ti(1), ti(2), ti(4), ti(7));

fprintf('============================================================\n');
fprintf(' 2. REFERENCE-CHANNEL ATTACKS\n');
fprintf('============================================================\n');

a_r = attack_lib('bias', t, N, dt, struct('A', 0.5, 't0', 0.1));
ti = run_attack_sim(3, Kp3, Kd3, Kd2, Ki3, Ki2, z, p, a_r, zeros(1,N), 'n', 150, -150);
fprintf(' ID3 + ref bias +0.5      : IAE=%.4e Eu=%.1f KuIAE=%.4e\n', ti(2), ti(4), ti(7));

a_r = attack_lib('sine', t, N, dt, struct('A', 0.25, 'f', 500, 't0', 0));
ti = run_attack_sim(3, Kp3, Kd3, Kd2, Ki3, Ki2, z, p, a_r, zeros(1,N), 'n', 150, -150);
fprintf(' ID3 + ref sine 500Hz     : IAE=%.4e Eu=%.1f KuIAE=%.4e\n', ti(2), ti(4), ti(7));

a_r = attack_lib('sine', t, N, dt, struct('A', 0.25, 'f', 500, 't0', 0));
ti = run_attack_sim(1, 0, 0, 0, 0, 0, 0, 0, a_r, zeros(1,N), 'n', bb_up, bb_low);
fprintf(' ID1 + ref sine 500Hz     : IAE=%.4e Eu=%.1f KuIAE=%.4e\n', ti(2), ti(4), ti(7));

fprintf('============================================================\n');
fprintf(' 3. SENSOR-CHANNEL ATTACKS (ID3 via cloop_stpid_attack_func)\n');
fprintf('============================================================\n');

a_s = attack_lib('bias', t, N, dt, struct('A', -0.05, 't0', 0.1));
ti = run_attack_sim(3, Kp3, Kd3, Kd2, Ki3, Ki2, z, p, zeros(1,N), a_s, 'n', 150, -150);
fprintf(' ID3 + sens bias -0.05    : IAE=%.4e Eu=%.1f KuIAE=%.4e\n', ti(2), ti(4), ti(7));

a_s = attack_lib('sine', t, N, dt, struct('A', 0.05, 'f', 100, 't0', 0));
ti = run_attack_sim(3, Kp3, Kd3, Kd2, Ki3, Ki2, z, p, zeros(1,N), a_s, 'n', 150, -150);
fprintf(' ID3 + sens sine 100Hz    : IAE=%.4e Eu=%.1f KuIAE=%.4e\n', ti(2), ti(4), ti(7));

fprintf('============================================================\n');
fprintf(' 4. SMALL WORST-CASE GRID (ID3, objective Eu)\n');
fprintf('============================================================\n');

grid_A  = [0.25];
grid_f  = [50 500];
grid_t0 = [0.1];
best = -inf;
for A = grid_A
    for f = grid_f
        for t0 = grid_t0
            a_r = attack_lib('sine', t, N, dt, struct('A', A, 'f', f, 't0', t0));
            ti = run_attack_sim(3, Kp3, Kd3, Kd2, Ki3, Ki2, z, p, a_r, zeros(1,N), 'n', 150, -150);
            fprintf('   candidate A=%.2f f=%4.0fHz t0=%.1fs -> Eu=%.1f KuIAE=%.4e\n', A, f, t0, ti(4), ti(7));
            if ti(4) > best
                best = ti(4); best_prm = [A f t0];
            end
        end
    end
end
fprintf('   worst Eu = %.1f at A=%.2f f=%.0fHz t0=%.1fs\n', best, best_prm(1), best_prm(2), best_prm(3));

fprintf('============================================================\n');
fprintf(' DONE.\n');
fprintf('============================================================\n');
