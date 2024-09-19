# Projection Error Calculation and Visualization

This Jupyter Notebook is designed to compute and visualize the projection error of 3D points projected from a camera onto display devices, such as virtual displays used in augmented reality (AR) or virtual reality (VR) headsets. It calculates how 3D points from the real world map to 2D image planes in a camera and how they align when projected onto the displays. The notebook also plots projection error as a function of depth, helping to analyze the performance of the projection model.

## Features

- **3D Projection and Back-projection**: Back-projects 2D points from a camera's image plane into the 3D world coordinate system using depth information.
- **3D to 2D Transformation**: Transforms 3D points into left and right display coordinate systems, projecting them back to 2D.
- **Projection Error Calculation**: Computes the projection error (in pixels) between the camera-projected points and their corresponding locations on the left and right displays.
- **Visualization**: Plots projection error as a function of depth, visualizing how the error changes as objects move further from the camera.

## Requirements

To run the notebook, you will need the following dependencies:

- **Python 3.x**
- **Jupyter Notebook**
- **NumPy**: For numerical computations and matrix manipulations.
- **Matplotlib**: For plotting the projection error.
- **VTK**: For 3D visualization of objects like frustums, cylinders, and cuboids in the scene.

You can install these dependencies using `pip`:

```bash
pip install numpy matplotlib vtk
