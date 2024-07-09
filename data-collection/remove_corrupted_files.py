import argparse
import pandas as pd
import os
import glob
import sys

def main(spreadsheet, data_folder):
    
    experiment_params = pd.read_excel(spreadsheet, dtype=object)

    h5files_spreadsheet = set(experiment_params["filename"][experiment_params["filename"].notna()] + ".hdf5")

    # find all h5 files in the data folder
    h5files_data_folder = set([file.split(os.sep)[-1] for file in glob.glob(data_folder + os.sep + "*.hdf5")])

    # get a list of the h5 files in the folder that aren't in the spreadsheet;
    # these are the files that contain bad data or were somehow a duplicate
    h5files_corrupted = h5files_data_folder.difference(h5files_spreadsheet)
    print(h5files_corrupted)
    print(len(h5files_corrupted))

    # move all the corrupted/duplicate files
    os.makedirs(data_folder + os.sep + "corrupted", exist_ok=True)
    for h5file in h5files_corrupted:
        # print(h5file)
        os.rename(data_folder + os.sep + h5file, data_folder + os.sep + "corrupted" + os.sep + h5file)
        
        name = h5file.split('.')[0]
        try:
            os.rename(data_folder + os.sep + name + "time-domain.png", data_folder + os.sep + "corrupted" + os.sep + name + "time-domain.png")
        except:
            pass
        try:
            os.rename(data_folder + os.sep + name + "frequency-domain.png", data_folder + os.sep + "corrupted" + os.sep + name + "frequency-domain.png")
        except:
            pass
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "spreadsheet",
        type=str,
        help="Path to the experiment parameters spreadsheet",
    )
    parser.add_argument(
        "data_folder",
        type=str,
        help="Path to the data folder",
    )

    args = parser.parse_args()

    sys.exit(
        main(args.spreadsheet, args.data_folder)
    )
