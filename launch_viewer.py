import subprocess
from pinocchio.visualize import GepettoVisualizer
import sys
import pinocchio as pin
from os.path import dirname, join, abspath


class Viewer:
    @staticmethod
    def launchViewer(model, collision_model, visual_model):
        """Launch Gepetto visualizer and return visualizer object"""
        try:
            subprocess.Popen(["gepetto-gui"])
            viz = GepettoVisualizer(model, collision_model, visual_model)
            viz.initViewer()
            viz.loadViewerModel("pinocchio")
            return viz
        except ImportError as err:
            print(
                "Error while initializing the viewer. It seems you should install gepetto-viewer"
            )
            print(err)
            return None
        except AttributeError as err:
            print(
                "Error while loading the viewer model. It seems you should start gepetto-viewer"
            )
            print(err)
            return None

    @staticmethod
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
