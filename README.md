# ICP Point Cloud Registration

Description Placeholder

## Inputs

The East and West PLY files for the ScannerTop3D linescanner.


## Outputs

The East and West PLY files merged together.


## Arguments and Flags
* **Positional Arguments:**
    * **Directory containing east and west passes of PCD to be merged:** 'data_dir'
* **Required Arguments:**
    * **Filename of East PCD:** '-e', '--east_pcd'
    * **FIlename of West PCD:** '-w', '--west_pcd'
* **Optional Arguments:**
    * **Movement range threshold for ICP:** '-t', '--theshold', default = 10.0

