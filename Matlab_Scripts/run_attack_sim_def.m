function [t_info, x, u, err, ctrl_state, u_amp_lim_state, xm] = run_attack_sim_def(ctrl_type, Kp, Kd, Kd2, Ki, Ki2, z, p, a_r, a_s, use_filter, use_lim, use_dwell)

t_start = 0;  t_stop = 1;  dt = 0.00001;
Tr_const = 5; Tr_orbit = 3; dd = 2; x0 = 0.1;
rem_nonl_plant = 'y';
perm_ctrl = 'n';  perm_state = 1;
add_noise = 'n';
trip_hyst_d = 0.01;

N = floor((t_stop - t_start)/dt) + 1;
[t, r, dot_r] = orbit_func(Tr_const, Tr_orbit, t_start, t_stop, dt, 100, 10);

r = r + a_r;

if (use_filter == 'y')
    r = attack_filter_ref(r, N, 50);
end

dot_r = zeros(1, N);
for i = 1:1:N-1
    dot_r(i) = (r(i+1) - r(i))/dt;
end
min_u = dot_r - r.^dd;

nn = zeros(1, N);
xm = zeros(1, N);

if (use_lim == 'y'), u_amp_lim = 'y'; else, u_amp_lim = 'n'; end
if (use_dwell == 'y'), trip_hyst_en = 'y'; else, trip_hyst_en = 'n'; end
u_amp_lim_up  = 150;  u_amp_lim_low  = -150;
bb_up  = 46.5457;    bb_low = -45.4420;

if (ctrl_type == 0)
    [x,dot_x,err,u,u_amp_lim_state,ctrl_state,ctrl_state_on,pid_off_const,S,xm] = ...
        cloop_pid_attack_func(Kp,Kd,Ki,N,dt,dd,r,dot_r,x0,rem_nonl_plant,u_amp_lim,u_amp_lim_up,u_amp_lim_low,perm_ctrl,perm_state,trip_hyst_en,trip_hyst_d,add_noise,nn,a_s);

elseif (ctrl_type == 1)
    if (use_lim == 'y')
        bb_a_up = u_amp_lim_up;  bb_a_low = u_amp_lim_low;
    else
        bb_a_up = bb_up;         bb_a_low = bb_low;
    end
    [x,dot_x,err,u,u_amp_lim_state,no_bb_swtc,cum_no_bb_swtc,S,xm] = ...
        cloop_bb_attack_func(N,dt,dd,r,dot_r,x0,bb_a_up,bb_a_low,add_noise,nn,a_s);
    ctrl_state = zeros(1,N); ctrl_state_on = 1;

elseif (ctrl_type == 2)
    [x,dot_x,err,u,opt_u,pid_ctrl_sig,u_amp_lim_state,ctrl_state,ctrl_state_on,pid_off_const,S,xm] = ...
        cloop_optu_attack_func(Kp,Kd,Ki,N,dt,dd,r,dot_r,x0,rem_nonl_plant,u_amp_lim,u_amp_lim_up,u_amp_lim_low,perm_ctrl,perm_state,trip_hyst_en,trip_hyst_d,add_noise,nn,a_s);

elseif (ctrl_type == 3)
    [x,dot_x,err,u,u_amp_lim_state,ctrl_state,ctrl_state_on,pid_off_const,S,xm] = ...
        cloop_stpid_attack_func(Kp,Kd,Ki,N,dt,dd,r,dot_r,x0,rem_nonl_plant,u_amp_lim,u_amp_lim_up,u_amp_lim_low,add_noise,nn,a_s);

elseif (ctrl_type == 4)
    [x,dot_x,err,u,u_amp_lim_state,ctrl_state,ctrl_state_on,pid_off_const,S,xm] = ...
        cloop_stpi2d2_attack_func(Kp,Kd,Kd2,Ki,Ki2,N,dt,dd,r,dot_r,x0,rem_nonl_plant,u_amp_lim,u_amp_lim_up,u_amp_lim_low,add_noise,nn,a_s);

elseif (ctrl_type == 5)
    [Kp_tv,Kd_tv,Ki_tv,x,dot_x,err,u,u_amp_lim_state,ctrl_state,ctrl_state_on,pid_off_const,S,xm] = ...
        cloop_tvpid_attack_func(Kp,Kd,Ki,N,dt,dd,r,dot_r,x0,rem_nonl_plant,u_amp_lim,u_amp_lim_up,u_amp_lim_low,trip_hyst_en,trip_hyst_d,add_noise,nn,a_s);

elseif (ctrl_type == 6)
    [x,dot_x,err,u,u_amp_lim_state,ctrl_state,ctrl_state_on,S,xm] = ...
        cloop_clow_attack_func(Kp,z,p,N,dt,dd,r,dot_r,x0,rem_nonl_plant,u_amp_lim,u_amp_lim_up,u_amp_lim_low,add_noise,nn,a_s);
    pid_off_const = 1;

else
    error('Unknown ctrl_type %d', ctrl_type);
end

tot_err     = sum(abs(err))*dt;
tot_err_ise = (sum(err.^2)*dt)^0.5;
tot_u_en    = sum(u.^2)*dt;
tot_min_u_en= sum(min_u.^2)*dt;
ave_u_p     = tot_u_en/N;
fin_err     = abs(err(end));
K_uiae      = sum(abs(err.*u))*dt/(t_stop - t_start);
K_uise      = (sum((err.*u).^2)*dt)^0.5/(t_stop - t_start);
ctrl_util   = 100*sum(ctrl_state)/N;
lim_util    = 100*sum(abs(u_amp_lim_state))/N;

t_info = [fin_err, tot_err, tot_err_ise, tot_u_en, ave_u_p, tot_min_u_en, K_uiae, K_uise, ctrl_util, lim_util];
end
