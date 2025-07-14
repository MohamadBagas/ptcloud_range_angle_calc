# **LiDAR Range and Angle of Incidence Calculator**

This command-line tool processes .las or .laz point cloud files to compute and add two new attributes for each point: **Range** (the distance from the sensor) and **AngleOfIncidence** (the angle between the laser beam and the surface normal).

It's optimized for performance on large datasets and can handle various input scenarios, including point clouds with separate trajectory files or those with embedded scanner origin data.

\<br\>

## **Setup Guide**

Follow these steps to set up the required environment and install the necessary packages. It is highly recommended to use a virtual environment to avoid conflicts with other Python projects.

### **1\. Install a Python Environment Manager (Recommended)**

If you don't have one, we recommend installing **Miniconda**, a minimal installer for the Conda environment manager.

* [Miniconda Installation Guide](https://docs.conda.io/projects/miniconda/en/latest/)

### **2\. Create and Activate a New Environment**

Open your terminal or Anaconda Prompt and run the following commands. This will create a new, isolated environment named lidar\_env with Python 3.9.

\# Create the new environment  
conda create \--name lidar\_env python=3.9

\# Activate the environment (you must do this every time you open a new terminal)  
conda activate lidar\_env

### **3\. Install Required Libraries**

With your lidar\_env environment active, install all the necessary libraries using pip:

pip install numpy pandas laspy\[lazrs\] open3d scipy tqdm

You are now ready to run the script.

\<br\>

## **How to Run**

Save the script as process\_lidar\_final.py and run it from your command prompt or terminal where the lidar\_env is active.

### **Scenario 1: Processing a Single File with a Trajectory**

This is the most common use case. Provide the path to your LAS/LAZ file and the corresponding trajectory file.

python process\_lidar\_final.py \-i "C:/data/scan\_01.laz" \-t "C:/data/trajectory.xyz" \-o "C:/data/output/scan\_01\_processed.laz"

\<hr\>

### **Scenario 2: Processing a Whole Folder with a Trajectory**

The script can process all .las and .laz files within a specified folder.

python process\_lidar\_final.py \-i "C:/data/all\_scans/" \-t "C:/data/trajectory.xyz" \-o "C:/data/output/"

\<hr\>

### **Scenario 3: Processing Files Without a Trajectory**

If your files already contain x\_origin, y\_origin, and z\_origin data, simply omit the \-t trajectory argument.

\# For a single file  
python process\_lidar\_final.py \-i "C:/data/scan\_with\_origin.las" \-o "C:/data/output/scan\_with\_origin\_processed.las"

\# For a whole folder  
python process\_lidar\_final.py \-i "C:/data/scans\_with\_origin/" \-o "C:/data/output/"

\<hr\>

### **Scenario 4: Tuning for Performance**

For very large files or on computers with less RAM, you can adjust the processing parameters.

* \--chunk\_size: Reduce to lower memory usage.  
* \-k: Reduce to speed up the normal estimation calculation.

python process\_lidar\_final.py \-i "C:/data/huge\_scan.laz" \-t "C:/data/trajectory.xyz" \-o "C:/data/output/huge\_scan\_processed.laz" \--chunk\_size 100000 \-k 15

\<br\>

## **Command-Line Arguments**

| Argument | Description | Required? |
| :---- | :---- | :---- |
| **\-i, \--input\_path** | Path to the input .las/.laz file or a folder containing them. | **Yes** |
| **\-o, \--output\_path** | Path to the output file or folder. Defaults to an output subfolder in the input directory. | No |
| **\-t, \--trajectory** | Path to the .xyz trajectory file. If omitted, the script will look for origin data within the LAS file. | No |
| **\-k, \--k\_neighbors** | The number of neighbors used for normal estimation. Default is 30\. | No |
| **\--chunk\_size** | The number of points to process per chunk to manage memory. Default is 500,000. | No |

