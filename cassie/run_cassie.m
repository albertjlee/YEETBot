% Function to run Cassie Simulator
% This simulator is released for use by students of ME 193B / 292B.
% All other uses are not authorized.
% Copyright: Hybrid Robotics [hybrid-robotics.berkeley.edu], 2019.
function run_cassie()
    %% Initial Setup
    close all ;

    % Add various paths
    startup;

    % Load Cassie model and set Initial configuration
    model = load('cassie_model.mat') ; model = model.model ;

    % Initial configuration
    x0 = getInitialState(model);

    % Perform any one-time setup and get student / control parameters
    [ctrl, student_data] = student_setup(x0, model);

    % ODE options
    time_inter = [0 2] ;
    odeopts = odeset('Events', @falldetect);
    externalForce_fun = @ExternalForce ;
    x0 = [x0; zeros(ctrl.N_integrators, 1)] ;


    %% Simulation 
    disp('Simulating...') ;
    tic

    if(ctrl.ode_type == 0)
        [t_vec, x_vec] = ode15s( @cassie_eom, time_inter, x0, odeopts, model, ctrl, externalForce_fun) ;
    else
        [t_vec, x_vec] =  ode45( @cassie_eom, time_inter, x0, odeopts, model, ctrl, externalForce_fun) ;
    end

    toc
    disp(['Simulated for ' num2str(t_vec(end)), 's'])

    %% Animation
    stateData = getVisualizerState(x_vec, model);
    vis = CassieVisualizer(t_vec, stateData);
end