function a = attack_lib(attack_type, t, N, dt, prm)

a = zeros(1, N);

switch lower(attack_type)
    case 'none'

    case 'bias'
        a = prm.A * double(t >= prm.t0);

    case 'ramp'
        a = prm.k * max(0, t - prm.t0);

    case 'sine'
        if ~isfield(prm, 'phi'), prm.phi = 0; end
        a = prm.A * sin(2*pi*prm.f*t + prm.phi);

    case 'pulse'
        a = prm.A * double(t >= prm.t0 & t < prm.t0 + prm.w);

    case 'stealthy'
        if ~isfield(prm, 'phi'), prm.phi = 0; end
        a = prm.A * sin(2*pi*prm.f*t + prm.phi);

    otherwise
        error('Unknown attack type: %s', attack_type);
end
end

function r_a = attack_replay_ref(r, tau, dt)

d = max(1, round(tau/dt));
r_a = r;
r_a(d+1:end) = r(1:end-d);
r_a(1:d)     = r(1);
end

function r_a = attack_freeze_ref(r, t0, dt)

i0 = max(1, round(t0/dt) + 1);
r_a = r;
r_a(i0:end) = r(i0);
end
