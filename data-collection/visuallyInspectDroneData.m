% Manually look at the images, time-domain plots, and spectrums for the drone data.
%
% This function is used for preliminary visual inspection of all the data, just so
% we can get a feel for what the data looks like and see if the prop frequencies
% show up in the spectrum.

function visuallyInspectDroneData(folder, rangebins, fileExtension)
    arguments
        folder (1,1) string
        rangebins (1,:) {mustBeInteger} = 118:124
        fileExtension (1,1) string = "hdf5"
    end

    close

    NANOSEC_TO_SEC = 1e-9;
    IMG_NUM = 1;
    HPF_CUTOFF = 25;

    clims = 'auto';

    imageAx = axes();

    frequencySeen = table("", "",'VariableNames', {'filename', 'frequencySeen'});

    % Find all h5 files in the folder
    h5files = cellfun(@string, {dir(folder + filesep + "*." + fileExtension).name});

    % For each file in folder
    for h5file = h5files
        % Load data
        [h5data, h5meta] = loadh5(folder + filesep + h5file);

        % Only look at the first image from each file
        data = squeeze(h5data.data.data(IMG_NUM,:,:));
        timestamps = h5data.data.timestamps(IMG_NUM,:) * NANOSEC_TO_SEC;

        % Compute sampling frequency
        fs = computeSamplingFrequency(timestamps);

        % Extract rangebins that potentially contain a drone/propeller
        droneSignals = data(rangebins,:);

        % High-pass filter the data
        % filteredDroneSignals = hpf(droneSignals, fs, HPF_CUTOFF);

        % Compute the spectrum of the filtered data
        [droneSpectrumMag, f] = computeSpectrum(droneSignals, fs);

        % Plot everything
        makePlots

        % Prompt the user whether they saw the prop frequency or want to change the color limits
        isReadyForNextImage = false;

        while ~isReadyForNextImage
            response = input("[y] frequency seen\n[n] frequency not seen\n[m] frequency maybe seen\n[p] frequencies partially seen\n[c] change color limits\n[r] change range bins\nChoice: ", 's');

            if strcmpi(response, 'c')
                isClimInputValid = false;

                while ~isClimInputValid
                    climInput = input("set color limits (enter two numbers with a space between, or auto): ", 's');
                    clims = str2num(climInput);
                    if size(clims) == [1,2]
                        isClimInputValid = true;
                        clim(imageAx, clims);
                    elseif strcmpi(climInput, 'auto')
                        isClimInputValid = true;
                        clim(imageAx, 'auto');
                        clims = 'auto';
                    end
                end
	    elseif strcmpi(response, 'r')
	        isRangebinInputValid = false;

		while ~isRangebinInputValid
		    rangebinInput = input("set lower and upper range bin (space-delimited): ", 's');
		    rangebinLimits = str2num(rangebinInput);
		    if size(rangebinLimits) == [1,2]
		        isRangebinInputValid = true;
		        rangebins = rangebinLimits(1):rangebinLimits(2);

			% Recompute and redo the plots with the new rangebins
			droneSignals = data(rangebins,:);
			[droneSpectrumMag, f] = computeSpectrum(droneSignals, fs);
		        makePlots
		    end
		end
            elseif strcmpi(response, 'y')
                freqSeenResponse = "yes"; 
                isReadyForNextImage = true;
            elseif strcmpi(response, 'm')
                freqSeenResponse = "maybe"; 
                isReadyForNextImage = true;
            elseif strcmpi(response, 'n')
                freqSeenResponse = "no"; 
                isReadyForNextImage = true;
            elseif strcmpi(response, 'p')
                freqSeenResponse = "partial"; 
                isReadyForNextImage = true;
            else
                disp("Incorrect input. Try again :)")
            end
        end

        % Save whether the prop frequencies
        frequencySeen.filename(end+1) = h5file;
        frequencySeen.frequencySeen(end) = freqSeenResponse;

        clf;
    end

    % Remove the first empty row from the table
    frequencySeen(1,:) = [];
    writetable(frequencySeen, folder + filesep + "visual-inspection-results.csv");


    function makePlots
        tlayout = tiledlayout(2,5);
        tlayout.TileIndexing = "columnmajor";

        % Plot the image
        imageAx = nexttile([2, 2]);
        if strcmpi(clims, 'auto')
            imagesc(data);
        else
            imagesc(data, clims);
        end
        xlabel('Pulse number')
        ylabel('Range bin')
        cbar = colorbar;
        cbar.Label.String = "Amplitude [V]";

        % Plot the time-domain signals
        nexttile([1,2]);
        plot(timestamps, droneSignals.', 'LineWidth', 1);
        xlabel('Time [s]')
        ylabel('Amplitude [V]')

        % Plot the spectrums
        nexttile([1,2]);
        plot(f, droneSpectrumMag, 'LineWidth', 1.5);
        xlabel('Frequency [Hz]')
        legendObj = legend(string(rangebins), 'Location', 'eastoutside');
        legendObj.NumColumns = 2;
        legendObj.Box = "off";
        title(legendObj,"Range bin");
        legendObj.AutoUpdate = "off";
        xlim([50,fs/2])

        % Plot ground truth lines for the motor frequencies
        hold on
        prop_freqs = [
            h5data.parameters.prop_frequency.back_left.avg(IMG_NUM),
            h5data.parameters.prop_frequency.back_right.avg(IMG_NUM),
            h5data.parameters.prop_frequency.front_left.avg(IMG_NUM),
            h5data.parameters.prop_frequency.front_right.avg(IMG_NUM),
        ];
        for prop_freq = prop_freqs.'
            if prop_freq > 0
                lineObj = xline(prop_freq, 'k--', 'Alpha', 0.3, 'LineWidth', 2);
            end
        end

        % Annotate plot with parameters and ground truth
        textbox = annotation("textbox", [0.78, 0.1, 0.2, 0.8]);
        textbox.EdgeColor = "none";
        textbox.FitBoxToText = "off";

        annotationStr = [
            "\bf{Drone info:}\rm",
            "prop size = " + string(h5data.parameters.prop_size),
            "# blades = " + string(h5data.parameters.n_blades),
            "",
            "\bf{Experiment info:}\rm"
            "fill factor = " + string(h5data.parameters.fill_factor),
            "motor config = " + string(h5data.parameters.motor_configuration),
            "tilt = " + string(h5data.parameters.tilt) + "^\circ",
            "",
            "\bf{Throttle:}\rm"
            "back left = " + string(fillmissing(h5data.parameters.throttle.back_left, 'Constant', 0)),
            "back right = " + string(fillmissing(h5data.parameters.throttle.back_right, 'Constant', 0)),
            "front left = " + string(fillmissing(h5data.parameters.throttle.front_left, 'Constant', 0)),
            "front right = " + string(fillmissing(h5data.parameters.throttle.front_right, 'Constant', 0)),
            "",
            "\bf{Prop frequency:}\rm"
            "back left = " + string(h5data.parameters.prop_frequency.back_left.avg(IMG_NUM)) + " [Hz]",
            "back right = " + string(h5data.parameters.prop_frequency.back_right.avg(IMG_NUM)) + " [Hz]",
            "front left = " + string(h5data.parameters.prop_frequency.front_left.avg(IMG_NUM)) + " [Hz]",
            "front right = " + string(h5data.parameters.prop_frequency.front_right.avg(IMG_NUM)) + " [Hz]",
        ];

        textbox.String = annotationStr;
    end

end

function fs = computeSamplingFrequency(timestamps)
    Ts = mean(diff(timestamps));
    fs = 1/Ts;
end

function filtered = hpf(signal, fs, cutoff, order)
    arguments
        signal double 
        fs (1,1) double
        cutoff (1,1) double = 50
        order (1,1) {mustBeInteger} = 1
    end

    normalizedCutoff = cutoff / (fs/2);

    [b,a] = butter(order, normalizedCutoff, "high");
    filtered = filtfilt(b, a, signal);
end

function [magnitude, f] = computeSpectrum(signal, fs)
    arguments
        signal double
        fs (1,1) double
    end

    nSamples = width(signal);

    magnitude = abs(fft(signal, [], 2)).^2;
    magnitude = magnitude(:,1:nSamples/2);

    f = linspace(0, fs/2, nSamples/2);
end
