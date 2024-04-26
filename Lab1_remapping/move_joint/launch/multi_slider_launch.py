from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher()

    for joint in ("left_e0", "right_e0"):
        with sl.group(ns = joint):
            sl.include(package = 'move_joint',
                       executable = 'slider_launch.py',
                       launch_arguments = {'name': joint})

    return sl.launch_description()
