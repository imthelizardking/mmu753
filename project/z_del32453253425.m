% Try it with your own values! (e.g., w_low = 50 rad/s, w_high = 500 rad/s)
w_low  = 50;  
w_high = 500; 

% Call the parametric filter function
H = make_bandpass(w_low, w_high);

% Display the resulting transfer function in the command window
disp('Your Parametric Transfer Function:')
bode(H);
grid on;

% Define the parametric function at the bottom of your script
function H_bp = make_bandpass(w_L, w_H)
    % 1. Calculate the parametric coefficients
    num = [ (w_H - w_L) , 0 ];          % Numerator: (w_H - w_L)*s
    den = [ 1 , (w_H - w_L) , (w_L*w_H) ]; % Denominator: s^2 + (w_H - w_L)*s + (w_L*w_H)
    
    % 2. Generate the transfer function object
    H_bp = tf(num, den);
end