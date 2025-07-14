import numpy as np
import pandas as pd
import laspy
from pathlib import Path
from tqdm import tqdm
import open3d as o3d
import copy
import argparse
import sys

def load_trajectory_xyz(path):
    """
    Loads and prepares trajectory data from an XYZ file,
    automatically detecting if a header row is present.
    """
    try:
        # --- START: HEADER DETECTION LOGIC ---
        # Peek at the first line to see if it contains text
        with open(path, 'r') as f:
            first_line = f.readline().strip()
        
        # Check if the first item in the line is non-numeric (a strong sign of a header)
        try:
            float(first_line.split()[0])
            header_setting = None  # It's a number, so no header
        except (ValueError, IndexError):
            header_setting = 0  # It's text (like 'time'), so use the first row as the header
        # --- END: HEADER DETECTION LOGIC ---

        # Load the CSV with the determined header setting
        df = pd.read_csv(path, sep='\s+', header=header_setting, engine='python')
        
        # If no header was found, assign column names manually
        if header_setting is None:
            num_cols = len(df.columns)
            if num_cols >= 12:
                df.columns = ['x', 'y', 'z', 'time', 'gps_time', 'pitch', 'roll', 'yaw', 'rot.w', 'rot.x', 'rot.y', 'rot.z'][:num_cols]
            elif num_cols >= 8:
                df.columns = ['x', 'y', 'z', 'time', 'gps_time', 'pitch', 'roll', 'yaw']
            else:
                raise ValueError(f"Trajectory file has an unexpected number of columns: {num_cols}")

        # Standardize column names (e.g., 'gpstime' becomes 'gps_time')
        df.columns = df.columns.str.lower().str.replace('gpstime', 'gps_time')
        
        # Continue with processing as before
        for col in ['gps_time', 'x', 'y', 'z']:
            df[col] = pd.to_numeric(df[col], errors='coerce')
        df = df.dropna(subset=['gps_time', 'x', 'y', 'z'])
        
        # Ensure data is sorted by time for merge_asof
        return df.sort_values('gps_time')
    except Exception as e:
        print(f"❌ Error loading trajectory file {path}: {e}")
        sys.exit(1)

def _estimate_open3d_normals(points_chunk, k=30):
    """
    Estimates normals for a chunk of points using Open3D.
    """
    if len(points_chunk) == 0:
        return np.empty((0, 3), dtype=np.float64)
        
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_chunk)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=k))
    return np.asarray(pcd.normals)

def compute_angle_of_incidence(points, normals, origins):
    """
    Computes the angle of incidence using vectorized numpy operations.
    """
    vec_to_origin = origins - points
    vec_to_origin_norm = vec_to_origin / (np.linalg.norm(vec_to_origin, axis=1)[:, None] + 1e-9)
    dot_product = np.einsum('ij,ij->i', normals, vec_to_origin_norm)
    angles_rad = np.arccos(np.clip(np.abs(dot_product), -1.0, 1.0))
    return np.degrees(angles_rad)

def process_las_file(las_path, output_path, k, chunk_size, filtered_traj_df=None):
    """
    Processes a single LAS/LAZ file using the efficient merge_asof method or internal origins.
    """
    print(f"\nProcessing {las_path.name}...")
    
    with laspy.open(las_path) as reader:
        output_header = copy.deepcopy(reader.header)

        # Add new dimensions to the header
        new_dims = [
            laspy.ExtraBytesParams(name="Range", type=np.float32),
            laspy.ExtraBytesParams(name="AngleOfIncidence", type=np.float32),
            laspy.ExtraBytesParams(name="traj_X", type=np.float32),
            laspy.ExtraBytesParams(name="traj_Y", type=np.float32),
            laspy.ExtraBytesParams(name="traj_Z", type=np.float32),
        ]
        existing_dims = set(output_header.point_format.dimension_names)
        final_new_dims = [dim for dim in new_dims if dim.name not in existing_dims]
        if final_new_dims:
            output_header.add_extra_dims(final_new_dims)

        with laspy.open(output_path, mode="w", header=output_header) as writer:
            progress_total = int(np.ceil(reader.header.point_count / chunk_size))
            for points_chunk in tqdm(reader.chunk_iterator(chunk_size), desc=f"Processing chunks", total=progress_total):
                points = np.vstack((points_chunk.x, points_chunk.y, points_chunk.z)).T
                
                # Determine scanner origins based on mode
                if filtered_traj_df is not None:
                    # Trajectory Mode: Use high-speed merge_asof
                    chunk_df = pd.DataFrame({
                        'gps_time': points_chunk.gps_time,
                        'original_index': np.arange(len(points_chunk))
                    })
                    chunk_df = chunk_df.sort_values('gps_time')

                    merged_df = pd.merge_asof(
                        chunk_df,
                        filtered_traj_df[['gps_time', 'x', 'y', 'z']],
                        on='gps_time',
                        direction='nearest'
                    )
                    merged_df = merged_df.sort_values('original_index')
                    scanner_origins = merged_df[['x', 'y', 'z']].values
                else:
                    # Internal Origin Mode
                    if not all(d in points_chunk.point_format.dimension_names for d in ['x_origin', 'y_origin', 'z_origin']):
                         print(f"⚠️ SKIPPING {las_path.name}: Lacks x_origin/y_origin/z_origin fields and no trajectory was provided.")
                         break
                    scanner_origins = np.vstack((points_chunk.x_origin, points_chunk.y_origin, points_chunk.z_origin)).T

                normals = _estimate_open3d_normals(points, k=k)
                angles = compute_angle_of_incidence(points, normals, scanner_origins)
                ranges = np.linalg.norm(points - scanner_origins, axis=1)
                
                new_chunk = laspy.ScaleAwarePointRecord.zeros(
                    len(points_chunk),
                    point_format=writer.header.point_format,
                    scales=writer.header.scales,
                    offsets=writer.header.offsets
                )
                
                for dim_name in points_chunk.point_format.dimension_names:
                    new_chunk[dim_name] = points_chunk[dim_name]
                
                new_chunk.Range = ranges.astype(np.float32)
                new_chunk.AngleOfIncidence = angles.astype(np.float32)
                new_chunk.traj_X = scanner_origins[:, 0].astype(np.float32)
                new_chunk.traj_Y = scanner_origins[:, 1].astype(np.float32)
                new_chunk.traj_Z = scanner_origins[:, 2].astype(np.float32)
                
                writer.write_points(new_chunk)

    print(f"✅ Finished: {output_path.name}")

# === Main Script ===
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process LiDAR point clouds to add range and angle of incidence.")
    parser.add_argument("-i", "--input_path", type=str, required=True, help="Path to a single .las/.laz file or a folder containing them.")
    parser.add_argument("-o", "--output_path", type=str, help="Path to the output file or folder. Defaults to an 'output' subfolder.")
    parser.add_argument("-t", "--trajectory", type=str, help="Optional: Path to the .xyz trajectory file.")
    
    parser.add_argument("-k", "--k_neighbors", type=int, default=30, help="Number of neighbors for normal estimation (default: 30).")
    parser.add_argument("--chunk_size", type=int, default=500_000, help="Number of points to read from file at a time (default: 500,000).")
    
    args = parser.parse_args()

    input_path = Path(args.input_path)
    
    if input_path.is_dir():
        scan_files = sorted(list(input_path.glob("*.las")) + list(input_path.glob("*.laz")))
    elif input_path.is_file():
        scan_files = [input_path]
    else:
        print(f"❌ Input path does not exist: {input_path}")
        sys.exit(1)
    
    if not scan_files:
        print(f"❌ No .las or .laz files found in {input_path}")
        sys.exit(1)

    if args.output_path:
        output_path = Path(args.output_path)
    else:
        output_path = (input_path.parent if input_path.is_file() else input_path) / "output"
    
    output_path.mkdir(exist_ok=True)

    if args.trajectory:
        print("Loading trajectory file...")
        full_traj_df = load_trajectory_xyz(Path(args.trajectory))
        
        for las_file in scan_files:
            print(f"\nOptimizing trajectory for {las_file.name}...")
            with laspy.open(las_file) as reader:
                if 'gps_time' not in reader.header.point_format.dimension_names:
                    print(f"❌ SKIPPING {las_file.name}: 'gps_time' is missing.")
                    continue
                las_times = reader.read().gps_time
                min_time, max_time = las_times.min(), las_times.max()

            time_buffer = 10.0
            filtered_traj_df = full_traj_df[
                (full_traj_df['gps_time'] >= min_time - time_buffer) &
                (full_traj_df['gps_time'] <= max_time + time_buffer)
            ]
            
            if filtered_traj_df.empty:
                print(f"⚠️ SKIPPING {las_file.name}: No matching trajectory data found.")
                continue
            
            print(f"Filtered trajectory to {len(filtered_traj_df)} points for processing.")
            
            final_output_path = output_path / f"{las_file.stem}_processed.laz"
            process_las_file(
                las_path=las_file,
                output_path=final_output_path,
                filtered_traj_df=filtered_traj_df,
                k=args.k_neighbors,
                chunk_size=args.chunk_size
            )
    else:
        # Logic for files without a trajectory
        print("No trajectory file provided. Relying on internal x_origin/y_origin/z_origin fields.")
        for las_file in scan_files:
            final_output_path = output_path / f"{las_file.stem}_processed.laz"
            process_las_file(
                las_path=las_file,
                output_path=final_output_path,
                k=args.k_neighbors,
                chunk_size=args.chunk_size,
                filtered_traj_df=None # Explicitly pass None
            )