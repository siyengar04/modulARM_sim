import subprocess
from pinocchio.visualize import GepettoVisualizer
import sys
import pinocchio as pin
from os.path import dirname, join, abspath
import time

# import simulate_motion #cirular, don't use

urdf_path = "./robot/robot_description.urdf"
mesh_dir = "./robot/meshes"
model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_path, mesh_dir)
subprocess.Popen(["pkill", "-f", "gepetto-gui"])
time.sleep(1)


viz = GepettoVisualizer(model, collision_model, visual_model)


class Viewer:
    # from functions from Pinocchio documentation
    # @staticmethod
    def launchViewer():
        subprocess.Popen(["gepetto-gui"])
        time.sleep(5)
        """Launch Gepetto visualizer and return visualizer object"""
        try:
            viz.initViewer()
            # viz.loadViewerModel("pinocchio")

        except ImportError as err:
            print(
                "Error while initializing the viewer. It seems you should install gepetto-viewer"
            )
            print(err)
            sys.exit(0)
        try:
            viz.loadViewerModel("pinocchio")
        except AttributeError as err:
            print(
                "Error while loading the viewer model. It seems you should start gepetto-viewer"
            )
            print(err)
            sys.exit(0)

    # @staticmethod
    def displayMotion(viz, path_data, dt=0.05):
        """Display motion plan in visualizer

        Args:
            viz: GepettoVisualizer object
            path_data: numpy array of configurations (Nx DOF)
            dt: time between frames in seconds
        """
        if viz is None:
            print("Visualizer not available")
            return

        import time

        for i, config in enumerate(path_data):
            try:
                viz.display(config)
                print(f"Frame {i+1}/{len(path_data)}")
                time.sleep(dt)
            except Exception as e:
                print(f"Error displaying frame {i}: {e}")
                break


if __name__ == "__main__":
    Viewer.launchViewer()
