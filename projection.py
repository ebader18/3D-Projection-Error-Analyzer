import numpy as np

# Functions to add
    # Tranform => select from what to what
    # Project
    # Back-project
class projection:
    def __init__(self, headset_params, cam_params, disp_params):
        self.headset_params = headset_params
        self.cam_params = cam_params
        self.disp_params = disp_params

        ## Compute intrinsic parameters for camera and displays
        self.K_cam = self.compute_camera_intrinsics()
        self.K_disp = self.compute_display_intrinsics()

        ## Compute extrinsic parameters for camera and displays
        self.t_w_cam = self.compute_translation_vector(self.headset_params['World']['Origin'], self.headset_params['Camera']['Origin'])
        self.t_w_ldisp = self.compute_translation_vector(self.headset_params['World']['Origin'], self.headset_params['Left display']['Origin'])
        self.t_w_rdisp = self.compute_translation_vector(self.headset_params['World']['Origin'], self.headset_params['Right display']['Origin'])
        self.R_cam = self.compute_rotation_matrix(self.headset_params['World']['Direction'], self.headset_params['Camera']['Direction'])
        self.R_ldisp = self.compute_rotation_matrix(self.headset_params['World']['Direction'], self.headset_params['Left display']['Direction'])
        self.R_rdisp = self.compute_rotation_matrix(self.headset_params['World']['Direction'], self.headset_params['Right display']['Direction'])

    def compute_focal_length(self, fov, size):
        """
        Calculate the focal length in pixels based on the field of view and sensor size.

        Parameters:
        fov (float): Field of view in degrees
        size (int): Size of the sensor in pixels (width or height)

        Returns:
        float: Focal length in pixels
        """
        fov_rad = np.deg2rad(fov)
        focal_length = size / (2 * np.tan(fov_rad / 2))
        return focal_length

    def compute_fov(self, focal_length, size):
        fov = np.rad2deg(np.arctan(focal_length/size))

        return fov

    def compute_camera_intrinsics(self):
        res_x = self.cam_params['Sensor']['Resolution']['Hor']
        res_y = self.cam_params['Sensor']['Resolution']['Ver']
        fov_h = self.cam_params['Optics']['FoV']['Hor']
        fov_v = self.cam_params['Optics']['FoV']['Ver']
        cx = self.cam_params['Sensor']['Principal Point']['Hor']
        cy = self.cam_params['Sensor']['Principal Point']['Ver']

        # Calculate focal lengths
        fx = self.compute_focal_length(fov_h, res_x)
        fy = self.compute_focal_length(fov_v, res_y)

        # Intrinsic parameters for camera
        K = np.array([  [fx, 0, cx],
                        [0, fy, cy],
                        [0, 0, 1]])

        return K

    def compute_display_intrinsics(self):
        res_x = self.disp_params['Panel']['Resolution']['Hor']
        res_y = self.disp_params['Panel']['Resolution']['Ver']
        cx = self.disp_params['Panel']['Principal Point']['Hor']
        cy = self.disp_params['Panel']['Principal Point']['Ver']
        eye_relief = self.disp_params['Eye']['Relief']
        #fov_h = compute_fov(eye_relief, res_x)
        #fov_v = compute_fov(eye_relief, res_y)
        
        # Intrinsic parameters for camera
        K = np.array([  [eye_relief, 0, cx],
                        [0, eye_relief, cy],
                        [0, 0, 1]])

        return K

    def compute_ray(self, K, pixel_point):
        # Convert the pixel point to homogeneous coordinates
        #ones_column = np.ones((pixel_point.shape[0], 1))
        #pixel_point_homogeneous = np.hstack((pixel_point, ones_column))
        pixel_point_homogeneous = np.array([pixel_point[0], pixel_point[1], 1.0])
        
        # Compute the inverse of the intrinsic matrix
        intrinsic_matrix_inv = np.linalg.inv(K)
        
        # Transform the pixel point to normalized image coordinates
        normalized_coords = intrinsic_matrix_inv @ pixel_point_homogeneous
        
        # The ray direction in camera coordinates is the normalized image coordinates
        ray_direction = normalized_coords / np.linalg.norm(normalized_coords)
        
        return ray_direction.reshape(3, 1)

    def compute_translation_vector(self, p1, p2):
        t = np.array(p1) - np.array(p2)

        return t.reshape(3, 1)

    def compute_rotation_matrix(self, thetas1, thetas2):
        theta_x = np.deg2rad(thetas1[0]) - np.deg2rad(thetas2[0]) # world origin x-axis direction minus camera/display origin x-axis direction
        theta_y = np.deg2rad(thetas1[1]) - np.deg2rad(thetas2[1])
        theta_z = np.deg2rad(thetas1[2]) - np.deg2rad(thetas2[2])

        # Rotation along x-axis
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(theta_x), -np.sin(theta_x)],
                        [0, np.sin(theta_x), np.cos(theta_x)]])

        # Rotation along y-axis
        R_y = np.array([[np.cos(theta_y), 0, np.sin(theta_y)],
                        [0, 1, 0],
                        [-np.sin(theta_y), 0, np.cos(theta_y)]])

        # Rotation along z-axis
        R_z = np.array([[np.cos(theta_z), -np.sin(theta_z), 0],
                        [np.sin(theta_z), np.cos(theta_z), 0],
                        [0, 0, 1]])
        
        # Combined rotation matrix
        R = R_z @ R_y @ R_x

        return R

    def project_point(self, point, K):
        x = K @ point
        #print(f'Homogenous 2d coordinate: {x}')
        
        x = x[:2] / x[2]
        #print(f'2d vector coordinate: {x}')

        return x

    def backproject2world(self, pt2d, K, R, t, depth):
        ray = self.compute_ray(K, pt2d)
        pt3d = ray  * depth
        pt3d_w = np.linalg.inv(R)@(pt3d - t)

        return pt3d_w
    
    def get_frustums(self):
        far = 25e-3
        
        xres, yres = 2 * self.K_cam[0, 2], 2 * self.K_cam[1, 2]
        pt0 = np.linalg.inv(self.R_cam)@(np.array([0, 0, 0]).reshape(3, 1) - self.t_w_cam)
        pt1 = self.backproject2world(np.array([0, 0]), self.K_cam, self.R_cam, self.t_w_cam, far)
        pt2 = self.backproject2world(np.array([xres, 0]), self.K_cam, self.R_cam, self.t_w_cam, far)
        pt3 = self.backproject2world(np.array([xres, yres]), self.K_cam, self.R_cam, self.t_w_cam, far)
        pt4 = self.backproject2world(np.array([0, yres]), self.K_cam, self.R_cam, self.t_w_cam, far)
        camera_frustum = [pt0, pt1, pt2, pt3, pt4]

        xres, yres = 2 * self.K_disp[0, 2], 2 * self.K_disp[1, 2]
        pt0 = np.linalg.inv(self.R_ldisp)@(np.array([0, 0, 0]).reshape(3, 1) - self.t_w_ldisp)
        pt1 = self.backproject2world(np.array([0, 0]), self.K_disp, self.R_ldisp, self.t_w_ldisp, far)
        pt2 = self.backproject2world(np.array([xres, 0]), self.K_disp, self.R_ldisp, self.t_w_ldisp, far)
        pt3 = self.backproject2world(np.array([xres, yres]), self.K_disp, self.R_ldisp, self.t_w_ldisp, far)
        pt4 = self.backproject2world(np.array([0, yres]), self.K_disp, self.R_ldisp, self.t_w_ldisp, far)
        ldisp_frustum = [pt0, pt1, pt2, pt3, pt4]

        pt0 = np.linalg.inv(self.R_rdisp)@(np.array([0, 0, 0]).reshape(3, 1) - self.t_w_rdisp)
        pt1 = self.backproject2world(np.array([0, 0]), self.K_disp, self.R_rdisp, self.t_w_rdisp, far)
        pt2 = self.backproject2world(np.array([xres, 0]), self.K_disp, self.R_rdisp, self.t_w_rdisp, far)
        pt3 = self.backproject2world(np.array([xres, yres]), self.K_disp, self.R_rdisp, self.t_w_rdisp, far)
        pt4 = self.backproject2world(np.array([0, yres]), self.K_disp, self.R_rdisp, self.t_w_rdisp, far)
        rdisp_frustum = [pt0, pt1, pt2, pt3, pt4]

        return [camera_frustum, ldisp_frustum, rdisp_frustum]
    
    def compute_projection_error(self, pt2d, depth_info):
        [dmin, dmax, dstep] = depth_info
        #depths = np.linspace(dmin, dmax, dstep).reshape(1, dstep)
        depths = np.geomspace(dmin, dmax, dstep).reshape(1, dstep)

        pt3d_w_cam= self.backproject2world(pt2d, self.K_cam, self.R_cam, self.t_w_cam, depths)

        pt3d_w_ldisp = self.R_ldisp@pt3d_w_cam + self.t_w_ldisp
        pt3d_w_rdisp = self.R_rdisp@pt3d_w_cam + self.t_w_rdisp

        pt2d_ldisp = self.project_point(pt3d_w_ldisp, self.K_disp)
        pt2d_rdisp = self.project_point(pt3d_w_rdisp, self.K_disp)

        return [pt2d, depths, pt2d_ldisp, pt2d_rdisp]