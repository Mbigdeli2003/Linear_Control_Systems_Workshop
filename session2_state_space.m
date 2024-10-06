%%Session 2 state space
clc
clear 
close all
%% Realization
% define the system
% Define the numerator and denominator of the transfer function
num = [5 3 7 1];
den = [1 4 6 3 8];
tf(num,den)
%% Create the state-space system using controllable canonical form
[A, B, C, D] = tf2ss(num, den);

% Display the state-space matrices
A
B
C
D

% To check the step response
sys = ss(A, B, C, D)
sys_ccf = ss(A, B, C, D)
figure
step(sys)



%% Create a state-space system from the transfer function
sys_tf = tf(num, den);

% Convert to observable canonical form
sys_obs = canon(sys_tf, 'companion');

% Display the state-space matrices
[A_obs, B_obs, C_obs, D_obs] = ssdata(sys_obs);

A_obs
B_obs
C_obs
D_obs
%% Diagonal form
% Create a state-space system
sys = ss(A, B, C, D);

% Diagonalize if possible (Jordan canonical form)
[T, J] = jordan(A);

% Display diagonalized matrices
J
T
% Check step response
sys_diag = ss(J, T\B, C*T, D);
figure
step(sys_diag)

%% TF from SS
% Define state-space matrices A, B, C, D
clc
clear 
close all

A = [-4 -6 -3 -8;
      1  0  0  0;
      0  1  0  0;
      0  0  1  0];

B = [1;
     0;
     0;
     0];

C = [5 3 7 2];

D = [0];

% Convert state-space system to transfer function
[num, den] = ss2tf(A, B, C, D);

% Display the numerator and denominator of the transfer function
disp('Numerator coefficients:');
disp(num);
disp('Denominator coefficients:');
disp(den);

% Create and display the transfer function
sys_tf = tf(num, den);
disp('Transfer Function:');
sys_tf


%% Simulink
clc
clear
close all
% Define the numerator and denominator of the transfer function
num = [5 3 7 2]; % Coefficients of the numerator
den = [1 4 6 3 8]; % Coefficients of the denominator

% Convert to state-space form
[A, B, C, D] = tf2ss(num, den);

% Open Simulink
simulink

% Rename the model
model = 'StateSpaceModel_1';  % Choose a unique name
new_system(model);           % Create a new system
open_system(model);          % Open the system

% Add blocks to the system
add_block('simulink/Continuous/State-Space', [model '/State-Space_1'], 'A', mat2str(A), 'B', mat2str(B), 'C', mat2str(C), 'D', mat2str(D));
add_block('simulink/Sources/Step', [model '/Step']);
add_block('simulink/Sinks/Scope', [model '/Scope']);

% Connect the blocks
add_line(model, 'Step/1', 'State-Space_1/1');
add_line(model, 'State-Space_1/1', 'Scope/1');

% Set simulation parameters and run
set_param(model, 'StopTime', '10');
sim(model);  % Run the simulation


%% toolbox
% Define the state-space system
sys = ss(A, B, C, D);

% Open LTI viewer to see responses and characteristics
ltiview(sys);


%% eigen valuse and trace
eig(A)
trace(A)

%% vandermond
% first solution

clc
clear 
close all
% Define a state-space matrix A
A = [-4 -6 -3 -8;
      1  0  0  0;
      0  1  0  0;
      0  0  1  0];

B = [1;
     0;
     0;
     0];

C = [5 3 7 2];

D = [0];

% Create the state-space system
sys = ss(A, B, C, D);

% Step response of the original system
figure;
step(sys);
title('Original State-Space System Step Response');

% Compute the eigenvalues (diagonal matrix Lambda) and eigenvectors (Vandermonde matrix V)
[V, Lambda] = eig(A);

% Display the Vandermonde matrix (eigenvectors)
disp('Vandermonde Matrix (Eigenvectors):');
disp(V);

% Display the diagonal matrix of eigenvalues
disp('Diagonal Matrix of Eigenvalues (Lambda):');
disp(Lambda);


% Transform the system into modal (diagonal) form using the Vandermonde matrix V
A_modal = inv(V) * A * V;
B_modal = inv(V) * B;
C_modal = C * V;
D_modal = D;  % D remains unchanged

% Display the transformed state-space matrices in modal form
disp('A matrix in Modal Form:');
disp(A_modal);
disp('B matrix in Modal Form:');
disp(B_modal);
disp('C matrix in Modal Form:');
disp(C_modal);

% Create the state-space system in modal form
sys_modal = ss(A_modal, B_modal, C_modal, D_modal);

% Step response of the diagonalized (modal) system
figure;
step(sys_modal);
title('Modal State-Space System Step Response');


%% second solution avnder

clc
clear 
close all
% Define the state matrix A
A = [0 1 0;
     0 0 1;
    -6 -11 -6];

% Compute the eigenvalues of A
eigenvalues = eig(A);

% Display the eigenvalues
disp('Eigenvalues of A:');
disp(eigenvalues);
% Construct the Vandermonde matrix V
lambda = eigenvalues; % Eigenvalues
V = [1 1 1;
     lambda(1) lambda(2) lambda(3);
     lambda(1)^2 lambda(2)^2 lambda(3)^2];

% Display the Vandermonde matrix
disp('Vandermonde Matrix V:');
disp(V);
% Compute the inverse of V
V_inv = inv(V);

% Transform A into a diagonal matrix
A_modal = V_inv * A * V;

% Transform B and C
B_modal = V_inv * B;
C_modal = C * V;

% D remains the same
D_modal = D;

% Display the diagonalized A matrix and transformed B and C
disp('Diagonalized A matrix (A_modal):');
disp(A_modal);

disp('Transformed B matrix (B_modal):');
disp(B_modal);

disp('Transformed C matrix (C_modal):');
disp(C_modal);

% Create the modal state-space system
sys_modal = ss(A_modal, B_modal, C_modal, D_modal);

% Original state-space system
sys_original = ss(A, B, C, D);

% Compare the step responses
figure;
step(sys_original, 'r', sys_modal, 'b--');
legend('Original System', 'Modal System');
title('Comparison of Step Responses');
xlabel('Time (seconds)');
ylabel('Amplitude');
grid on;

%% with syntax of vander
clc
clear 
close all
A_1=[0 1 0;
     0 0 1;
    -6 -11 -6];
v_1=vander(eig(A_1))

%% STM and solving State space Eq
clc 
clear
close all
% Define the state-space matrix A
A = [-4 -6 -3 -8;
      1  0  0  0;
      0  1  0  0;
      0  0  1  0];
syms x
% Define a specific time t
t =  0;  % Example time at t = 2 seconds

% Compute the state transition matrix at time t
Phi_t = expm(A * t);

% Display the state transition matrix
disp('State Transition Matrix at time t = s:');
disp(Phi_t);

%% calculating phi in paramteric way
clc 
clear
close all
syms t omega

% Define the state-space matrix A (for a simple harmonic oscillator)
A = [0 1; -omega^2 0];

% Compute the state transition matrix Phi(t) = exp(A*t)
Phi_t = expm(A * t);

% Display the parametric state transition matrix
disp('State Transition Matrix Phi(t):');
disp(Phi_t);
% Define numerical values for omega and time
omega_val = 2;  % Example angular frequency
t_values = 0:0.1:10;  % Time from 0 to 10 seconds

% Preallocate for storing numerical state transition matrices
Phi_t_num = zeros(2, 2, length(t_values));

% Compute Phi(t) for each time step
for i = 1:length(t_values)
    t_val = t_values(i);
    Phi_t_num(:,:,i) = double(subs(Phi_t, {t, omega}, {t_val, omega_val}));
end

% Plot the elements of Phi(t) over time
figure;
subplot(2,1,1);
plot(t_values, squeeze(Phi_t_num(1,1,:)), 'r', 'DisplayName', 'Phi_{11}(t)');
hold on;
plot(t_values, squeeze(Phi_t_num(1,2,:)), 'b', 'DisplayName', 'Phi_{12}(t)');
xlabel('Time (seconds)');
ylabel('Phi(t) Elements');
title('State Transition Matrix Elements');
legend;

subplot(2,1,2);
plot(t_values, squeeze(Phi_t_num(2,1,:)), 'g', 'DisplayName', 'Phi_{21}(t)');
hold on;
plot(t_values, squeeze(Phi_t_num(2,2,:)), 'm', 'DisplayName', 'Phi_{22}(t)');
xlabel('Time (seconds)');
ylabel('Phi(t) Elements');
legend;

%% Cayley-Hamilton Theorem

% Define matrix A
A = [-1 0; 2 1];

% Symbolic variable for time t
syms t;

% Step 1: Compute the characteristic polynomial of A
char_poly = charpoly(A);

% Step 2: Extract coefficients from the characteristic polynomial
% The characteristic polynomial is of the form λ^n + c_{n-1} λ^{n-1} + ... + c_0
% MATLAB returns coefficients starting with the highest power term
coefficients = fliplr(char_poly);  % Flip to get coefficients in ascending powers

% Display the characteristic polynomial coefficients
disp('Characteristic Polynomial Coefficients:');
disp(coefficients);

% Step 3: Apply Cayley-Hamilton theorem
% For the matrix A, use the fact that A satisfies its own characteristic equation
% The characteristic polynomial is λ^2 - 1 = 0, so A^2 = I

% Calculate the matrix exponential e^(At) using the Cayley-Hamilton theorem
% Start by finding the identity matrix of the same size as A
I = eye(size(A));

% Since A^2 = I for this matrix, we can express higher powers of A
% Phi(t) = c0*I + c1*A + c2*(A^2) + ... based on the characteristic polynomial

% Initialize Phi(t) as the zero matrix
Phi_t = coefficients(1)*I;  % The constant term c_0

% Add the linear term (coefficient of A)
Phi_t = Phi_t + coefficients(2)*A*t;

% Add the quadratic term (since the characteristic polynomial is degree 2)
Phi_t = Phi_t + coefficients(3)*A^2*t^2 / 2;  % Divide by 2 due to factorial(2)

% Display the state transition matrix
disp('State Transition Matrix Phi(t) (using Cayley-Hamilton):');
disp(Phi_t);

% Optional: Evaluate Phi(t) for a specific value of t (e.g., t = 1)
Phi_t_at_1 = double(subs(Phi_t, t, 1));
disp('State Transition Matrix Phi(t) at t = 1:');
disp(Phi_t_at_1);

%% contralabity and observability 

clc
clear 
close all
% Define state-space matrices A, B, C, D
A = [1 2; 3 4];
B = [1; 0];
C = [1 0];
D = 0;

% Create a state-space system
sys = ss(A, B, C, D);

%% Controllability

% Compute the controllability matrix
controllability_matrix = ctrb(A, B);

% Check the rank of the controllability matrix
controllability_rank = rank(controllability_matrix);

% Determine if the system is controllable
n_states = size(A, 1);  % Number of states (size of A)
if controllability_rank == n_states
    disp('The system is controllable.');
else
    disp('The system is not controllable.');
end

%% Observability

% Compute the observability matrix
observability_matrix = obsv(A, C);

% Check the rank of the observability matrix
observability_rank = rank(observability_matrix);

% Determine if the system is observable
if observability_rank == n_states
    disp('The system is observable.');
else
    disp('The system is not observable.');
end

