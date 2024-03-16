import open3d as o3d
import numpy as np

def create_pyramid_camera_wireframe(height: float = 1.0, side: float = 1.0) -> o3d.geometry.LineSet:
    """
    Create a wireframe of a square pyramid representing a camera.
    Parameters
    ----------
    height : float
        The height of the pyramid. Default is 1.0.
    side : float
        The side length of the square base. Default is 1.0.
    Returns
    -------
    line_set : open3d.geometry.LineSet
        The wireframe of the pyramid.
    """

    # Points defining the pyramid vertices (1 point at the tip and 4 at the base square vertices)
    points = [
        [0, 0, 0],
        [side / 2, side / 2, -height],
        [-side / 2, side / 2, -height],
        [-side / 2, -side / 2, -height],
        [side / 2, -side / 2, -height],
    ]

    # Lines defining the pyramid edges (4 lines from the tip to the base and 4 lines connecting the base vertices)
    lines = [
        [0, 1], [0, 2], [0, 3], [0, 4], 
        [1, 2], [2, 3], [3, 4], [4, 1]
    ]

    colors = [[0, 0, 1]] * len(lines) # Blue color for the edges

    # Draw line set
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines),
    )
    line_set.colors = o3d.utility.Vector3dVector(colors)

    return line_set


def animate_camera_around_object(object: o3d.geometry.PointCloud, frames: int = 200) -> None:
    """
    Animate a camera moving around a given object. The motion the camera follows the boundary of 1 out of 6 surfaces of the object bounding box.
    Parameters
    ----------
    object : open3d.geometry.PointCloud
        The point cloud representing the object to be observed by the camera.
    frames : int
        The number of frames for the camera animation. Default is 200.
    """

    # Get the object bounding box
    bbox = object.get_axis_aligned_bounding_box()
    bbox.color = (1, 0, 0)  # Red color for the object bounding box

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window()
    vis.add_geometry(object)
    vis.add_geometry(bbox)

    min_bound = bbox.get_min_bound()
    max_bound = bbox.get_max_bound()
    
    # Define the key points for the camera path (4 top corners of the bounding box of the object)
    # Note that the object orientation is currently not aligned with the world axis
    # Currently, x-axis: left to right right, y-axis: bottom to top, z-axis: back to front
    key_points = [
        np.array([max_bound[0], max_bound[1], max_bound[2]]),  # Top front right
        np.array([min_bound[0], max_bound[1], max_bound[2]]),  # Top front left
        np.array([min_bound[0], max_bound[1], min_bound[2]]),  # Top back left
        np.array([max_bound[0], max_bound[1], min_bound[2]]),  # Top back right
        np.array([max_bound[0], max_bound[1], max_bound[2]]),  # Back to start, top front right
    ]
    
    # Equal frames per segment (4 segments in total)
    frames_per_segment = frames // (len(key_points) - 1)
    frame = 0

    # To have a path tracing line while the camera moves
    path_points = []
    path_lines = []

    def move_camera(vis):
        nonlocal frame
        if frame >= frames:
            print("Animation end")
            return
        
        segment_index = frame // frames_per_segment
        segment_progress = (frame % frames_per_segment) / frames_per_segment
        
        start_pos = key_points[segment_index]
        end_pos = key_points[segment_index + 1]
        # Linear interpolation between the start and end key points
        camera_pos = start_pos + (end_pos - start_pos) * segment_progress
        
        # We aim to make the camera always looking at the center of the object bounding box
        camera_lookat = bbox.get_center()

        # Translate the camera to the calculated position
        camera = create_pyramid_camera_wireframe(height=0.1, side=0.1)
        camera.translate(camera_pos, relative=False)

        # Get the tip of the camera wireframe pyramid
        camera_tip = camera.points[0]

        # Project the vectors onto the xz-plane, find the angle to rotate so that camera_tip, camera_pos, and camera_lookat are aligned
        v1 = camera_pos - camera_tip
        v2 = camera_lookat - camera_tip
        v1[1] = v2[1] = 0 

        # Calculate the angle between v1 and v2 using Cosine Rule
        angle = np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))

        # Calculate the cross product of v1 and v2
        cross_product = np.cross(v1, v2)

        # If the y-component of the cross product is negative, make the angle negative
        if cross_product[1] < 0:
            angle = -angle

        # Rotate the camera along its y-axis by the calculated angle and 45 degrees around the x-axis to point the camera downwards
        rotation = o3d.geometry.get_rotation_matrix_from_axis_angle((-np.pi / 4, angle, 0))
        camera.rotate(rotation, center=camera_pos)

        # Trace path
        if frame > 0:
            path_lines.append([frame - 1, frame])
        path_points.append(camera.points[0])  # Tip of the pyramid
        path_line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(path_points),
            lines=o3d.utility.Vector2iVector(path_lines),
        )
        path_line_set.colors = o3d.utility.Vector3dVector([[0, 1, 0]] * len(path_lines)) # Green color for the path
        
        # Update the visualizer
        vis.clear_geometries()
        vis.add_geometry(object)
        vis.add_geometry(bbox)
        vis.add_geometry(camera)
        vis.add_geometry(path_line_set)
        frame += 1

    vis.register_key_callback(262, move_camera)  # Right arrow key to increment frame
    vis.run()
    vis.destroy_window()


def main():
    # Taken from Open3D examples
    sample_ply_data = o3d.data.DemoCropPointCloud()
    pcd = o3d.io.read_point_cloud(sample_ply_data.point_cloud_path)
    vol = o3d.visualization.read_selection_polygon_volume(sample_ply_data.cropped_json_path)
    chair = vol.crop_point_cloud(pcd)
    # Flip the pointclouds, otherwise they will be upside down.
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    chair.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

    # Animate camera moving around the chair
    animate_camera_around_object(chair)

if __name__ == "__main__":
    main()