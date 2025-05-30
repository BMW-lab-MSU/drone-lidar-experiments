import numpy as np
from scipy.signal import find_peaks
from scipy.signal import butter, filtfilt
import matplotlib.pyplot as plt
import libf0
import h5py
import csv

#Function to detect the fundamental Frequency from the fft that has aliased frequencies
def detect_fundamental_frequency(fft_magnitude, sampling_freq):
    """
    Detect the fundamental frequency from the FFT magnitude by calculating the association between the peaks, and the nyquist frequency.
    If the peaks are aliased, it will return the peak that which when adding the nyquist frequency to it and subtracting the previous peak, will give the fundamental frequency.
    This function assumes that the input FFT magnitude is already computed and contains the positive frequencies only.

    Parameters:
    - fft_magnitude (numpy.ndarray): The magnitude of the FFT.
    - sampling_freq (float): The sampling frequency of the data.

    Returns:
    - float: The detected fundamental frequency.
    """
    
    # Calculate the nyquist frequency
    nyquist_freq = sampling_freq / 2
    
    # Apply a low-pass Butterworth filter to the best frequency row
    cutoff = 350  # Cutoff frequencies for smoothing
    normalCutoff = cutoff / nyquist_freq
    order = 4  # Order of the filter
    b, a = butter(order, normalCutoff, btype='lowpass', analog=False,)
    fft_magnitude = filtfilt(b, a, fft_magnitude)
    
    # Find peaks in the FFT magnitude
    peaks, properties = find_peaks(fft_magnitude, prominence=0.8, height=0.1* np.max(fft_magnitude))
    heights = properties['peak_heights']
    peak_frequencies = peaks * (nyquist_freq / len(fft_magnitude))
    margin = 10  # Margin to consider for peak detection in Hz
    
    # Order the peak frequencies by heights of the peaks
    sorted_indices = np.argsort(heights)[::-1]  # Sort indices by heights in descending order
    peak_frequencies = peak_frequencies[sorted_indices]
    
    normal_harmonics = []
    aliased_harmonics = []
    
    if len(peak_frequencies) == 1:
        return peak_frequencies[0]  # Return the single peak frequency if only one peak is found
    elif len(peak_frequencies) > 1:
        # Iterate through the peaks to find the fundamental frequency
        for i in range(len(peak_frequencies)):
            for j in range(len(peak_frequencies)):
                P1 = peak_frequencies[i]
                P2 = peak_frequencies[j]
                P2_imaginary = nyquist_freq + (nyquist_freq - P2)
                P1_predicted = P2_imaginary/2
                if abs(P1 - P2*2) < margin:
                    normal_harmonics.append(P2)
                if abs(P1 - P1_predicted) < margin:
                    aliased_harmonics.append(P2_imaginary - P1)
        print(f"Normal harmonics: {normal_harmonics}, Aliased harmonics: {aliased_harmonics}")
        if len(normal_harmonics) > 0 or len(aliased_harmonics) > 0:
            # Determine which set of harmonics is more relevant to determine the fundamental frequency
            if len(normal_harmonics) > len(aliased_harmonics):
                # If there are more normal harmonics, return the first one
                return normal_harmonics[0]
            elif len(aliased_harmonics) > len(normal_harmonics):
                # If there are more aliased harmonics, return the first one
                return aliased_harmonics[0]
        return peak_frequencies[0]  # No valid fundamental frequency found
    return 0


# Function to design a high-pass Butterworth filter and apply it
def highpass_filter(data, fs, cutoff=10, order=4):
    """
    Apply a high-pass Butterworth filter to the input data.

    Parameters:
    - data (numpy.ndarray): The input data to be filtered.
    - fs (float): The sampling freq of the data.
    - cutoff (float, optional): The cutoff freq of the filter in Hz. Defaults to 1.
    - order (int, optional): The order of the filter. Defaults to 1.

    Returns:
    - numpy.ndarray: The filtered data.
    """
    nyquistRate = 0.5 * fs
    normalCutoff = cutoff / nyquistRate
    b, a = butter(order, normalCutoff, btype='high', analog=False)
    filteredData = filtfilt(b, a, data)
    return filteredData

# Folder path
folderPath = '//blackmore.msu.montana.edu/ece-bmw-lab/drone-lidar/field-test-data/combined/'

# List of file paths
visualFilePaths = [
    folderPath + 'visual-inspection-results.csv',
]

total_detections = 0  # Increment this for each detection in your loop
within_confidence_count = 0
percentage_within_confidence = 0

# List to store best mean standard differences
average_best_mean_std_diff_percent = []
average_best_mean_std_diff = []
num_zeros = 0

# Define the output CSV file path
output_csv_path = folderPath + 'freq_analysis_results.csv'

# Open the output CSV file for writing
with open(output_csv_path, mode='w', newline='') as output_file:
    csv_writer = csv.writer(output_file)
    # Write the header row
    csv_writer.writerow([ 'File',
                          'Propeller',
                          'Confidence Interval Low', 
                          'Confidence Interval High',
                          'Best Mean Std Diff (%)', 
                          'Average', 
                          'f0 Detected freq', 
                          'fft Detected freq', 
                          'Peaks Detected freq difference', 
                          'Peaks Detected freq 1st value',
                          'My Detected freq'
                         ])
    
    # Process each file
    for visualFile in visualFilePaths:
        with open(visualFile, mode='r') as file:
            csvReader = csv.DictReader(file)
            for row in csvReader:
                fileName = row['filename']
                freqSeen = row['frequencySeen'] # Yes or No
                
                # If the freqSeen is 'yes', then calculate the statistics from the raw data
                if freqSeen == 'yes':# or freqSeen == 'maybe' or freqSeen == 'no':
                    # Open the h5 file
                    h5File = h5py.File(folderPath + fileName, 'r')
                    print(f"Processing file: {fileName}")
                    # Get the data from the h5 file
                    h5Data = h5File['data']
                    h5Image = h5Data['data']
                    capture_times = h5Data['capture_time']
                    timestamps = np.array(h5Data['timestamps'])
                    # Get the parameters from the h5 file
                    h5Params = h5File['parameters']
                    lens_tube_extension = h5Params['lens_tube_extension']
                    prop_freq = h5Params['prop_frequency']
                    prop_freq_back_left = prop_freq['back_left']
                    prop_freq_back_right = prop_freq['back_right']
                    prop_freq_front_left = prop_freq['front_left']
                    prop_freq_front_right = prop_freq['front_right']
                    
                    bl_avg = np.array(prop_freq_back_left['avg'])
                    bl_std = np.array(prop_freq_back_left['std_dev'])
                    br_avg = np.array(prop_freq_back_right['avg'])
                    br_std = np.array(prop_freq_back_right['std_dev'])
                    fl_avg = np.array(prop_freq_front_left['avg'])
                    fl_std = np.array(prop_freq_front_left['std_dev'])
                    fr_avg = np.array(prop_freq_front_right['avg'])
                    fr_std = np.array(prop_freq_front_right['std_dev'])
                    
                    # Initialize variables to track the strongest freq across all rows for the current image
                    strongest_freq_index = None
                    best_image_index = None
                    max_magnitude = 0
                    best_freq_row = None
                    iteration = 0
                    # Initialize variables to track the detected frequencies
                    detected_freq_f0 = None
                    detected_freq_fft = None
                    detected_freq_peaks_1 = None
                    detected_freq_peaks_diff = None
                    # Iterate through all images in the first index of h5Image
                    for image_index in range(h5Image.shape[0]):

                        # Initialize variables to track the strongest freq in the current image
                        # strongest_freq_index = None
                        # max_magnitude = 0
                        
                        # Iterate through the rows in the current image
                        row_index = 0
                        best_freq_row_index = 0
                        for row in h5Image[image_index, :, :]:
                            row_index += 1
                            
                            # Calculate the sampling freq
                            current_index = image_index
                            time_differences = np.diff(timestamps[current_index])
                            time_differences_in_seconds = time_differences / 1e9
                            average_sampling_period = np.mean(time_differences_in_seconds)
                            sampling_freq = 1 / average_sampling_period
                            
                            row = highpass_filter(row, sampling_freq, cutoff=100)
                            
                            # Calculate the FFT of the row
                            fft_result = np.fft.fft(row)
                            
                            # Get the magnitude of the FFT result (ignore DC and negative frequencies)
                            fft_magnitude = np.abs(fft_result[0:len(fft_result)//2]) # np.concatenate((np.zeros(3), np.abs(fft_result[3:len(fft_result)//2])))
                            
                            # Find the strongest freq in the current row
                            row_max_magnitude = np.max(fft_magnitude)
                            row_strongest_freq_index = np.argmax(fft_magnitude)
                            
                            # Update the global strongest freq if the current row's is stronger
                            if row_max_magnitude > max_magnitude:
                                max_magnitude = row_max_magnitude
                                strongest_freq_index = row_strongest_freq_index
                                best_image_index = image_index  # Store the index of the image with the strongest freq
                                best_freq_row = fft_magnitude  # Store the row with the strongest freq
                                best_time_row = row  # Store the row with the strongest freq
                                best_freq_row_index = row_index  # Store the row number with the strongest freq
                                # iteration += 1
                    
                    # Calculate the freq range from the capture_times
                    current_index = best_image_index
                    best_image = h5Image[best_image_index, :, :]
                    time_differences = np.diff(timestamps[current_index]) 
                    nyquist_freq = sampling_freq / 2
                    scaling_factor = nyquist_freq / len(best_freq_row)

                    # Convert time differences to seconds
                    time_differences_in_seconds = time_differences / 1e9
                    # Calculate the average sampling period (in seconds)
                    average_sampling_period = np.mean(time_differences_in_seconds)
                    # Calculate the sampling freq (in Hz)
                    sampling_freq = 1 / average_sampling_period

                    # --------------------------------------------------------------
                    # Calculate the detected freq using four different methods
                    # --------------------------------------------------------------
                    
                    # libf0 
                    detected_freq_f0 = libf0.yin(best_time_row, Fs=sampling_freq, H=2048)[0][0]
                    
                    # Index
                    detected_freq_fft = strongest_freq_index * scaling_factor
                    
                    # Findpeaks
                    peaks = find_peaks(best_freq_row, prominence=0.8)[0] * scaling_factor
                    detected_freq_peaks_1 = np.float32(peaks[0]) if len(peaks) > 0 else None
                    detected_freq_peaks_2 = np.float32(peaks[1]) if len(peaks) > 1 else None
                    if len(peaks) > 1:
                        peak_differences = np.diff(peaks)  # Calculate differences between consecutive peaks
                        detected_freq_peaks_diff = np.float32(max(set(peak_differences), key=list(peak_differences).count))  # Find the mode difference
                    else:
                        detected_freq_peaks_diff = detected_freq_peaks_1

                    # My method
                    detected_freq_myfft = detect_fundamental_frequency(best_freq_row, sampling_freq)
                    
                    # Plot the results
                    plt.figure(num=1, figsize=(12, 6))
                    plt.plot(range(len(best_freq_row)) * scaling_factor, best_freq_row, label='Row Data')
                    plt.title(f"FFT Magnitude for Image {best_image_index}, Range Bin {best_freq_row_index}")
                    plt.xlabel('freq Bin')
                    plt.ylabel('Magnitude')
                
                    # Smooth the best frequency row to reduce noise
                    nyquistRate = 0.5 * sampling_freq
                    order = 4  # Order of the filter
                    
                    plt.axvline(detected_freq_fft, color='r', linestyle='-', label='fft')
                    plt.axvline(detected_freq_f0, color='g', linestyle='--', label='f0')
                    plt.axvline(detected_freq_peaks_diff, color='b', linestyle='--', label='detected peaks differences')
                    plt.axvline(detected_freq_peaks_1, color='orange', linestyle='--', label='detected peaks 1st value')
                    plt.axvline(detected_freq_peaks_2, color='purple', linestyle='--', label='detected peaks 2nd value')
                    plt.axvline(detected_freq_myfft, color='cyan', linestyle='--', label='My Detected Freq')
                    plt.axvline(prop_freq_front_right['avg'][current_index], color='black', linestyle='--', label='Front Right Avg')
                    plt.axvline(prop_freq_front_left['avg'][current_index], color='black', linestyle='--', label='Front Left Avg')
                    plt.axvline(prop_freq_back_right['avg'][current_index], color='black', linestyle='--', label='Back Right Avg')
                    plt.axvline(prop_freq_back_left['avg'][current_index], color='black', linestyle='--', label='Back Left Avg')
                    plt.legend()
                    plt.grid()
                    plt.show()
                    
                    input("Press Enter to continue...")
                    
                    # Map propeller names to their corresponding indices in avgs
                    avgs = (
                        bl_avg[current_index],
                        br_avg[current_index],
                        fl_avg[current_index],
                        fr_avg[current_index],
                    )
                    propeller_indices = {
                        "Back Left": 0,
                        "Back Right": 1,
                        "Front Left": 2,
                        "Front Right": 3,
                    }
                    
                    analysis_freq = detected_freq_myfft
                    
                    # Calculate the mean standard difference for each propeller
                    differences = {
                        "Back Left": abs(analysis_freq - bl_avg[current_index]),
                        "Back Right": abs(analysis_freq - br_avg[current_index]),
                        "Front Left": abs(analysis_freq - fl_avg[current_index]),
                        "Front Right": abs(analysis_freq - fr_avg[current_index]),
                    }
                    best_mean_std_diff = min(differences.values())
                    best_propeller = min(differences, key=differences.get)

                    
                    # Calculate confidence intervals
                    bl_conf_interval = (bl_avg[current_index] - 3.00 * bl_std[current_index], bl_avg[current_index] + 3.00 * bl_std[current_index])
                    br_conf_interval = (br_avg[current_index] - 3.00 * br_std[current_index], br_avg[current_index] + 3.00 * br_std[current_index])
                    fl_conf_interval = (fl_avg[current_index] - 3.00 * fl_std[current_index], fl_avg[current_index] + 3.00 * fl_std[current_index])
                    fr_conf_interval = (fr_avg[current_index] - 3.00 * fr_std[current_index], fr_avg[current_index] + 3.00 * fr_std[current_index])
                    
                    # Check if detected freq is within confidence intervals
                    within_confidence = {
                        "Back Left": bl_conf_interval[0] <= analysis_freq <= bl_conf_interval[1],
                        "Back Right": br_conf_interval[0] <= analysis_freq <= br_conf_interval[1],
                        "Front Left": fl_conf_interval[0] <= analysis_freq <= fl_conf_interval[1],
                        "Front Right": fr_conf_interval[0] <= analysis_freq <= fr_conf_interval[1],
                    }
                    tf = [
                        bl_conf_interval[0] <= analysis_freq <= bl_conf_interval[1],
                        br_conf_interval[0] <= analysis_freq <= br_conf_interval[1],
                        fl_conf_interval[0] <= analysis_freq <= fl_conf_interval[1],
                        fr_conf_interval[0] <= analysis_freq <= fr_conf_interval[1],
                    ]
                    
                    # Count the percentage of times the detected freq is within any confidence interval
                    total_detections = total_detections + 1
                    
                    # increment the count of within confidence intervals by 1 if any of the propellers are within the confidence interval
                    if any(within_confidence.values()):
                        within_confidence_count = within_confidence_count + 1
                    percentage_within_confidence = (within_confidence_count / total_detections) * 100
                    
                    # standardize the mean standard difference by the average freq of the propeller
                    average = avgs[propeller_indices[best_propeller]]
                    standardized_difference = best_mean_std_diff / average * 100

                    # Append the best mean standard difference to the list
                    average_best_mean_std_diff_percent.append(standardized_difference)
                    average_best_mean_std_diff.append(best_mean_std_diff)
                    
                    if detected_freq_f0 <= 5:
                        num_zeros += 1
                    
                    # Retrieve the confidence interval for the best propeller
                    relevant_conf_interval = {
                        "Back Left": bl_conf_interval,
                        "Back Right": br_conf_interval,
                        "Front Left": fl_conf_interval,
                        "Front Right": fr_conf_interval,
                    }[best_propeller]

                    # Print the standardized difference, detected freq, and relevant confidence interval
                    # print(
                    #     f"Best Mean Std Diff: {standardized_difference:.2f}% "
                    #     f"Propeller: {best_propeller} "
                    #     f"Detected Freq: {detected_freq:.2f} Hz "
                    #     f"Int: {relevant_conf_interval} "
                    #     f"File: {fileName} "
                    # )
                    # Write the results to the CSV file
                    csv_writer.writerow([
                        fileName,
                        best_propeller,
                        relevant_conf_interval[0],
                        relevant_conf_interval[1],
                        f"{standardized_difference}",
                        f"{average}",
                        f"{detected_freq_f0}",
                        f"{detected_freq_fft}",
                        f"{detected_freq_peaks_1}",
                        f"{detected_freq_peaks_diff}",
                        f"{detected_freq_myfft}",
                    ])
                
print(f"Percentage of times within confidence interval: {percentage_within_confidence:.2f}%")

# Calculate and print the average of the best mean standard differences
average_best_mean_std_diff_percent = np.mean(average_best_mean_std_diff_percent)
print(f"Average Best Mean Standard Difference: {average_best_mean_std_diff_percent:.2f}%")
# Calculate and print the average of the best mean standard differences
average_best_mean_std_diff = np.mean(average_best_mean_std_diff)
print(f"Average Best Mean Standard Difference: {average_best_mean_std_diff:.2f}") 
print(f"Number of zeros predicted: {num_zeros}")
# # Wait for user input to continue
# input("Press Enter to continue...")
