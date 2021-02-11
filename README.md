# YEETBot

The initial drop height can be configured on line 35 of getInitialState.m, which is located in the other_functions folder. The line looks like x0(3) = "SOME NUMBER"

NOTE: THIS IS THE HEIGHT OF THE COM OF CASSIE. 

Our controller is stable up to x0(3) = 1.9. Anything above will result in a simulation where CASSIE is unable to land in a balanced manner.
Our control scheme is in the studentController.m file, and the entire simulation can be run by running the run_cassie.m file.
Please note that the simulation could take a long while. We've seen simulations take up to ~1100 seconds.
