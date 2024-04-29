from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher()

    sl.declare_arg('x', '0')
    sl.declare_arg('y', '0')
    sl.declare_arg('z', '0.1')
    sl.declare_arg('yaw', '0')
    sl.declare_arg('pitch', '3.14')
    sl.declare_arg('roll', '0')
    sl.declare_arg('frame_id', 'right_gripper')
    sl.declare_arg('child_frame_id', 'left_gripper_desired')

    sl.include(package = 'baxter_simple_sim',
               launch_file = 'sim_launch.py',
               launch_arguments = {'lab':'puppet'})

    sl.node(package = 'tf2_ros',
            executable = 'static_transform_publisher',
            arguments = [sl.arg('x'), sl.arg('y'), sl.arg('z'),
                        sl.arg('yaw'), sl.arg('pitch'), sl.arg('roll'),
                        sl.arg('frame_id'), sl.arg('child_frame_id')])

    return sl.launch_description()
