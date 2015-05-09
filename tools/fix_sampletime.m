% This checks the sample time of the running model and corrects it if necessary

function fix_sampletime()
	% Grab the correct sample time from the global workspace,
	% produced by daq_params
	correctedSampleTime = evalin('base', 'correctedSampleTime');

	% This is used to communicate with a SLRT target
	tg = slrt;

	% If the sampletime has already been corrected, quit before doing anything
	if tg.sampleTime == correctedSampleTime
		disp('Sample time is correct.')
		return
	end

	% Diagnostic for the user
	disp('Sample time needs to be changed. Sending stop command.')

	% The model must be stopped to change the sample time.
	% To stop it, we should set gui_exit_cmd then wait until
	% the model stop has actually occurred
	tg.setparam('/gui_exit_cmd', 1);

	disp('Waiting for the model to stop')
	% We have to poll the target's status to detect when the model is stopped
	while ~strcmp(tg.Status, 'stopped')
		pause(.05) % This wait is arbitrary.
	end
	disp('Model stopped. Resetting stop command')

	% We need to flip gui_exit_cmd back or the model won't restart successfully
	tg.setparam('/gui_exit_cmd', 0);

	% Adjust the sample time and re-start the model
	disp('Changing the sample time')
	tg.sampleTime = correctedSampleTime;
	disp('Re-starting the model')
	tg.start;
	disp('fix_sampletime done.')
end
