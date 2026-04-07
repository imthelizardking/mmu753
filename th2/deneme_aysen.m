N = 1000;
sigma_v2 = 1;
a = [1, -0.1, -0.8];
b = 1;  %pay

impz(b, a, 50); % 1. soru a secenegi
figure
impz(a, b, 50); % 1. soru b secenegi

v = sqrt(sigma_v2) .* randn(N+100, 1);
u_full = filter(b, a, v);
u = u_full(101:end);

emp_mean = mean(u);
emp_var = var(u);
[emp_acf, lags] = xcorr(u, 2, 'biased');

fprintf('Stat:            Teorik | Kestirim\n');
fprintf('-----------------------------------------\n');
fprintf('Mean:            0.0000 | %.4f\n', emp_mean);
fprintf('Var r(0):        3.7    | %.4f\n', emp_var);
fprintf('r(1):            1.8500 | %.4f\n', emp_acf(lags == 1));
fprintf('r(2):            3.1500 | %.4f\n', emp_acf(lags == 2));