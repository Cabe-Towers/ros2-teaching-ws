import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='amr_assignment_pkg',
            executable='global_reference_frame',
            name='assignment_node'),
        launch_ros.actions.Node(
            package='amr_assignment_pkg',
            executable='depth_camera_node',
            name='assignment_node'),
  ])

# def generate_launch_description():
#     return launch.LaunchDescription([
#         launch_ros.actions.Node(
#             package='amr_assignment_pkg',
#             executable='depth_camera_node',
#             name='assignment_node'),
#   ])