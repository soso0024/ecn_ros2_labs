from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher()

    sl.declare_arg('name', 'right_e0')

    sl.node(package = 'slider_publisher',
            executable = 'slider_publisher',
            arguments = [sl.find('move_joint', 'single_joint.yaml')])
    
    sl.node(package = 'move_joint',
            executable = 'move_joint',
            arguments = [sl.find('move_joint', 'single_joint.yaml')],
            parameters = {'joint_name': sl.arg('name')},
            remappings = {'joint_setpoint': 'setpoint'})

    return sl.launch_description()
