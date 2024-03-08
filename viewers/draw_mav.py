"""
mavsim_python: drawing tools
    - Beard & McLain, PUP, 2012
    - Update history:
        1/13/2021 - TWM
        7/13/2023 - RWB
        1/16/2024 - RWB
"""
import numpy as np
import pyqtgraph.opengl as gl
from tools.rotations import euler_to_rotation
from tools.drawing import rotate_points, translate_points, points_to_mesh


class DrawMav:
    def __init__(self, state, window, scale=10):
        """
        Draw the Mav.

        The input to this function is a (message) class with properties that define the state.
        The following properties are assumed:
            state.north  # north position
            state.east  # east position
            state.altitude   # altitude
            state.phi  # roll angle
            state.theta  # pitch angle
            state.psi  # yaw angle
        """
        self.unit_length = scale
        sc_position = np.array([[state.north], [state.east], [-state.altitude]])  # NED coordinates
        # attitude of mav as a rotation matrix R from body to inertial
        R_bi = euler_to_rotation(state.phi, state.theta, state.psi)
        # convert North-East Down to East-North-Up for rendering
        self.R_ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        # get points that define the non-rotated, non-translated Mav and the mesh colors
        self.sc_points, self.sc_index, self.sc_meshColors = self.get_sc_points()
        self.sc_body = self.add_object(
            self.sc_points,
            self.sc_index,
            self.sc_meshColors,
            R_bi,
            sc_position)
        window.addItem(self.sc_body)  # add Mav to plot     

    def update(self, state):
        sc_position = np.array([[state.north], [state.east], [-state.altitude]])  # NED coordinates
        # attitude of mav as a rotation matrix R from body to inertial
        R_bi = euler_to_rotation(state.phi, state.theta, state.psi)
        self.sc_body = self.update_object(
            self.sc_body,
            self.sc_points,
            self.sc_index,
            self.sc_meshColors,
            R_bi,
            sc_position)

    def add_object(self, points, index, colors, R, position):
        rotated_points = rotate_points(points, R)
        translated_points = translate_points(rotated_points, position)
        translated_points = self.R_ned @ translated_points
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        mesh = points_to_mesh(translated_points, index)
        object = gl.GLMeshItem(
            vertexes=mesh,  # defines the triangular mesh (Nx3x3)
            vertexColors=colors,  # defines mesh colors (Nx1)
            drawEdges=True,  # draw edges between mesh elements
            smooth=False,  # speeds up rendering
            computeNormals=False)  # speeds up rendering
        return object

    def update_object(self, object, points, index, colors, R, position):
        rotated_points = rotate_points(points, R)
        translated_points = translate_points(rotated_points, position)
        translated_points = self.R_ned @ translated_points
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        mesh = points_to_mesh(translated_points, index)
        object.setMeshData(vertexes=mesh, vertexColors=colors)
        return object

    def get_sc_points(self):
        """"
            Points that define the Mav, and the colors of the triangular mesh
            Define the points on the Mav following information in Appendix C.3
        """
        # points are in XYZ coordinates
        #   define the points on the Mav according to Appendix C.3
        points = self.unit_length * np.array([
            [1.6, 0, .16],  # point 1 [0]
            [.89, .51, -.6],  # point 2 [1]
            [.89, -.51, -.6],  # point 3 [2]
            [.89, -.51, .6],  # point 4 [3]
            [.89, .51, .6],  # point 5 [4]
            [-3.57, 0, 0],  # point 6 [5]
            [0, 2.42, 0],  # point 7 [6]
            [-1, 2.42, 0],  # point 8 [7]
            [-1, -2.42, 0],  # point 9 [8]
            [0, -2.42, 0],  # point 10 [9]
            [-2.71, 1.24, 0],  # point 11 [10]
            [-3.57, 1.24, 0],  # point 12 [11]
            [-3.57, -1.24, 0],  # point 13 [12]
            [-2.71, -1.24, 0],  # point 14 [13]
            [-2.71, 0, 0],  # point 15 [14]
            [-3.57, 0, -1.48]  # point 16 [15]
            ]).T
        # point index that defines the mesh
        index = np.array([
            [0, 1, 2],  # Top Nose Face
            [0, 2, 3],  # Left Nose Face
            [0, 3, 4],  # Bottom Nose Face
            [0, 1, 4],  # Right Nose Face
            [1, 2, 5],  # Top Body Face
            [2, 3, 5],  # Left Body Face
            [3, 4, 5],  # Bottom Body Face
            [1, 4, 5],  # Right Body Face
            [14, 15, 5],  # Tail
            [6, 7, 8],  # Wing 1
            [6, 8, 9],  # Wing 2
            [10, 11, 12],  # Tailwing 1
            [10, 12, 13]  # Tailwing 2 
            ])
        #   define the colors for each face of triangular mesh
        red = np.array([1., 0., 0., 1])
        green = np.array([0., 1., 0., 1])
        blue = np.array([0., 0., 1., 1])
        yellow = np.array([1., 1., 0., 1])
        meshColors = np.empty((13, 3, 4), dtype=np.float32)
        meshColors[0] = red  # top nose face
        meshColors[1] = red  # left nose face
        meshColors[2] = red  # bottom nose face
        meshColors[3] = red  # right nose face
        meshColors[4] = blue  # top body face
        meshColors[5] = blue  # left body face
        meshColors[6] = blue  # bottom body face
        meshColors[7] = blue  # right body face
        meshColors[8] = red  # tail
        meshColors[9] = green # wing 1
        meshColors[10] = green  # wing 2
        meshColors[11] = green  # tailwing 1
        meshColors[12] = green  # tailwing 2 
        return points, index, meshColors

