import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value=True),
        launch_ros.actions.Node(
            package='amr_assignment_pkg',
            executable='global_reference_frame',
            name='grf_node'),
        launch_ros.actions.Node(
            package='amr_assignment_pkg',
            executable='depth_camera_node',
            name='depth_node'),
        launch_ros.actions.Node(
            package='amr_assignment_pkg',
            executable='state_node',
            name='state_node'),
  ])