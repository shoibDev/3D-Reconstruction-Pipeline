import open3d as o3d
import sys

def show_point_cloud(ply_path):
    """Load and visualize a PLY point cloud file."""
    # Load the point cloud
    pcd = o3d.io.read_point_cloud(ply_path)
    
    # Create a visualization window
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    
    # Add the point cloud to the visualizer
    vis.add_geometry(pcd)
    
    # Set default camera view (optional)
    opt = vis.get_render_option()
    opt.background_color = [0.1, 0.1, 0.1]  # Dark background
    opt.point_size = 1.0
    
    # Run the visualizer
    vis.run()
    vis.destroy_window()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python showcloud.py path/to/pointcloud.ply")
        sys.exit(1)
        
    ply_path = sys.argv[1]
    show_point_cloud(ply_path)
