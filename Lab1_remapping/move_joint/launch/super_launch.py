from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher()

    sl.include('baxter_simple_sim', 'sim_launch.py')

    for joint in ("left_e0", "right_e0"):
        with sl.group(ns = joint):
            sl.include(package = 'move_joint',
                       launch_file = 'slider_launch.py',
                       launch_arguments = {'name': joint})

    return sl.launch_description()
