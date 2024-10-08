

estimateMaxAngleAndFrequency('audio1.wav', 180)
estimateMaxAngleAndFrequency('audio2.wav', 180)
estimateMaxAngleAndFrequency('audio3.wav', 180)
estimateMaxAngleAndFrequency('audio4.wav', 180)
estimateMaxAngleAndFrequency('audio5.wav', 180)
estimateMaxAngleAndFrequency('audio6.wav', 180) % good for -30
estimateMaxAngleAndFrequency('audio7.wav', 180)
estimateMaxAngleAndFrequency('audio8.wav', 180) % good for -30






function [audioLength, maxTime, angleEstimation, dominantFrequency, maxSPL, propagationRange] = estimateMaxAngleAndFrequency(audioFile, totalRotation)
    % Function to estimate the time of maximal sound, corresponding angle, and frequency
    % Also outputs the maximal amplitude in SPL and estimated propagation range
    % Input:
    %  - audioFile: Path to the audio file (string)
    %  - totalRotation: Total rotation in degrees (e.g., 180)
    % Output:
    %  - audioLength: Length of the audio file in seconds
    %  - maxTime: Time of the maximal sound in seconds
    %  - angleEstimation: Estimated angle of the maximal sound in degrees
    %  - dominantFrequency: Estimated dominant frequency in Hz
    %  - maxSPL: Maximum amplitude in dB SPL
    %  - propagationRange: Estimated range in meters where sound can be heard

    % Load the audio file recorded from your phone
    [audioData, fs] = audioread(audioFile);

    % Convert to mono if the recording is stereo
    if size(audioData, 2) > 1
        audioData = mean(audioData, 2); % Take the average of both channels
    end

    % Calculate the envelope of the audio signal to estimate amplitude changes
    audio_envelope = abs(hilbert(audioData));

    % Find the time index where the amplitude is maximal
    [~, max_idx] = max(audio_envelope);

    % Convert index to time
    maxTime = max_idx / fs; % Time in seconds

    % Total recording time (audio length) in seconds
    audioLength = length(audioData) / fs;

    % Estimate the angle based on the time of maximal sound
    angleEstimation = (maxTime / audioLength) * totalRotation;

    % Perform FFT to find the dominant frequency
    N = length(audioData); % Number of samples
    f = (0:N-1)*(fs/N); % Frequency vector
    audio_fft = abs(fft(audioData)); % Magnitude of FFT
    
    % Find the index of the dominant frequency
    [~, dominantFreqIdx] = max(audio_fft(1:N/2)); % Use only the first half of FFT (positive frequencies)
    dominantFrequency = f(dominantFreqIdx); % Dominant frequency in Hz

    % Calculate maximum SPL in dB SPL
    p_ref = 20e-6; % Reference sound pressure level in Pa
    sensitivity = 10^(-40/20); % Assuming -40 dBV/Pa sensitivity
    max_amplitude_pa = max(audio_envelope) * sensitivity; % Convert to Pa using sensitivity
    maxSPL = 20 * log10(max_amplitude_pa / p_ref);

    % Estimate the propagation range where the sound could be heard at 0 dB SPL
    propagationRange = 10^((maxSPL - 0) / 20);

    % Output the time of the maximal sound, corresponding angle, frequency, and SPL
    disp(['Audio file length: ', num2str(audioLength), ' seconds']);
    disp(['The time of the maximal sound is at: ', num2str(maxTime), ' seconds']);
    disp(['The estimated angle of the acoustic beam is: ', num2str(angleEstimation - 90), ' degrees']);
    disp(['The dominant frequency of the audio is: ', num2str(dominantFrequency), ' Hz']);
    disp(['The maximal SPL is: ', num2str(maxSPL), ' dB SPL']);
    disp(['The estimated propagation range is: ', num2str(propagationRange), ' meters']);

    % Plot the original signal and its envelope
    t = (0:length(audioData)-1) / fs; % Time vector
    figure;
    subplot(2, 1, 1);
    plot(t, audioData);
    xlabel('Time (s)');
    ylabel('Amplitude');
    title('Recorded Audio Signal');
    
    subplot(2, 1, 2);
    plot(t, audio_envelope, 'r');
    xlabel('Time (s)');
    ylabel('Amplitude Envelope');
    title('Audio Signal Envelope (Maximal Sound Estimation)');
end


