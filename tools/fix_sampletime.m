% This checks the sample time of the running model and corrects it if necessary

function fix_sampletime()
	% Grab the correct sample time from the global workspace,
	% produced by daq_params
	correctedSampleTime = evalin('base', 'correctedSampleTime');

	% This is used to communicate with a SLRT target
	tg = slrt;

	% Check if the correction needs to be made at all
	if tg.sampleTime ~= correctedSampleTime
		% The model must be stopped to change the sample time, so stop,
		% change, and restart it.
		tg.stop;
		tg.sampleTime = correctedSampleTime;
		tg.start;
	end
end
