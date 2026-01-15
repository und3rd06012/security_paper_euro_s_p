function rf = attack_filter_ref(r, N, filter_order)

rf = r;
for pass = 1:(filter_order + 1)
    rf(1:N-1) = (rf(2:N) + rf(1:N-1)) / 2;
    rf(N) = rf(N-1);
end
d = floor(filter_order / 2);
rf(d+1:N) = rf(1:N-d);
rf(1:d) = rf(1:d) - rf(d+1);
end
