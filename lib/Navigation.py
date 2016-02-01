#!/usr/bin/python

### import guacamole libraries
import avango
import avango.gua
import avango.script
from avango.script import field_has_changed

### import framework libraries
from lib.Intersection import Intersection

### import Python libraries
import math
import time
   
class SteeringNavigation(avango.script.Script):

    ### fields ###

    ## input fields
    mf_dof = avango.MFFloat()
    mf_dof.value = [0.0,0.0,0.0,0.0,0.0,0.0,0.0] # init 7 channels

    sf_pointer_button0 = avango.SFBool()
    sf_pointer_button1 = avango.SFBool()

    sf_head_mat = avango.gua.SFMatrix4()
    sf_head_mat.value = avango.gua.make_trans_mat(0.0,0.0,1.0)

    ## internal fields
    mf_pointer_pick_result = avango.gua.MFPickResult()


    ## output fields
    sf_nav_mat = avango.gua.SFMatrix4()
    sf_nav_mat.value = avango.gua.make_identity_mat()

    
    ### constructor
    def __init__(self):
        self.super(SteeringNavigation).__init__()
        

    def my_constructor(self,
                       SCENEGRAPH,
                       PARENT_NODE,
                       MF_NAV_DOF, 
                       POINTER_STATION_NAME, 
                       POINTER_TRACKING_NAME, 
                       TRACKING_TRANSMITTER_OFFSET,
                       ):

        ### parameters ###
        self.ray_length = 100.0 # in meters
        self.ray_thickness = 0.01 # in meters

        self.intersection_point_size = 0.025 # in meters

        self.navidget_duration = 3.0 # in seconds
        self.navidget_sphere_size = 0.5 # in meters

        self.attenuation_factor = 1
        self.rot_center_offset = avango.gua.Vec3(0.0,0.0,0.0)

        ### variables ###
        self.navigation_mode = 0 # 0 = steering mode, 1 = maneuvering mode, 3 = Navidget target-mode, 4 = Navidget animation-mode

        self.navidget_start_pos = None
        self.navidget_target_pos = None
        self.navidget_start_quat = None
        self.navidget_target_quat = None
        self.navidget_start_time = None


        ### resources ###

        # init pointer sensors
        self.pointer_tracking_sensor = avango.daemon.nodes.DeviceSensor(DeviceService = avango.daemon.DeviceService())
        self.pointer_tracking_sensor.Station.value = POINTER_TRACKING_NAME
        self.pointer_tracking_sensor.ReceiverOffset.value = avango.gua.make_identity_mat()
        self.pointer_tracking_sensor.TransmitterOffset.value = TRACKING_TRANSMITTER_OFFSET

        self.pointer_device_sensor = avango.daemon.nodes.DeviceSensor(DeviceService = avango.daemon.DeviceService())
        self.pointer_device_sensor.Station.value = POINTER_STATION_NAME

        # init nodes and geometries
        _loader = avango.gua.nodes.TriMeshLoader() 

        self.ray_transform = avango.gua.nodes.TransformNode(Name = "ray_transform")
        PARENT_NODE.Children.value.append(self.ray_transform)

        self.ray_geometry = _loader.create_geometry_from_file("ray_geometry", "data/objects/cylinder.obj", avango.gua.LoaderFlags.DEFAULTS)
        self.ray_transform.Children.value.append(self.ray_geometry)
        self.ray_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(1.0, 0.0, 0.0, 1.0))
        self.ray_geometry.Transform.value = avango.gua.make_trans_mat(0.0,0.0,self.ray_length * -0.5) * \
                                            avango.gua.make_scale_mat(self.ray_thickness,self.ray_thickness,self.ray_length)

        self.intersection_point_geometry = _loader.create_geometry_from_file("intersection_point_geometry", "data/objects/sphere.obj", avango.gua.LoaderFlags.DEFAULTS)
        SCENEGRAPH.Root.value.Children.value.append(self.intersection_point_geometry)
        self.intersection_point_geometry.Tags.value = ["invisible"] # set geometry invisible

        self.maneuvering_point_geometry = _loader.create_geometry_from_file("maneuvering_point_geometry", "data/objects/sphere.obj", avango.gua.LoaderFlags.DEFAULTS)
        SCENEGRAPH.Root.value.Children.value.append(self.maneuvering_point_geometry)
        self.maneuvering_point_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(1.0, 0.0, 0.0, 0.25))
        self.maneuvering_point_geometry.Tags.value = ["invisible"] # set geometry invisible

        ## init Navidget nodes
        self.navidget_node = avango.gua.nodes.TransformNode(Name = "navidget_node")
        SCENEGRAPH.Root.value.Children.value.append(self.navidget_node)
        self.navidget_node.Tags.value = ["invisible"]

        self.navidget_sphere_geometry = _loader.create_geometry_from_file("navidget_sphere_geometry", "data/objects/sphere.obj", avango.gua.LoaderFlags.DEFAULTS | avango.gua.LoaderFlags.MAKE_PICKABLE)
        self.navidget_node.Children.value.append(self.navidget_sphere_geometry)
        self.navidget_sphere_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(1.0,1.0,1.0,0.25))
        self.navidget_sphere_geometry.Transform.value = avango.gua.make_scale_mat(self.navidget_sphere_size)

        self.navidget_stick_geometry = _loader.create_geometry_from_file("navidget_stick_geometry", "data/objects/cylinder.obj", avango.gua.LoaderFlags.DEFAULTS)
        self.navidget_node.Children.value.append(self.navidget_stick_geometry)
        self.navidget_stick_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(1.0,0.0,0.0,1.0))
        self.navidget_stick_geometry.Transform.value = avango.gua.make_trans_mat(0.0,0.0,self.navidget_sphere_size) * \
                                                       avango.gua.make_scale_mat(0.015,0.015,self.navidget_sphere_size * 2.0)

        self.navidget_camera_geometry = _loader.create_geometry_from_file("navidget_camera_geometry", "data/objects/cam.obj", avango.gua.LoaderFlags.DEFAULTS)
        self.navidget_node.Children.value.append(self.navidget_camera_geometry)
        #self.navidget_camera_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(1.0,0.0,0.0,1.0))
        self.navidget_camera_geometry.Transform.value = avango.gua.make_trans_mat(0.0,0.0,self.navidget_sphere_size * 2.0) * \
                                                        avango.gua.make_scale_mat(3.0)

        # init ray intersection for target identification
        self.intersection = Intersection(SCENEGRAPH = SCENEGRAPH)

        self.frame_trigger = avango.script.nodes.Update(Callback = self.frame_callback, Active = True)

        # init field connections
        self.mf_dof.connect_from(MF_NAV_DOF)
        self.sf_pointer_button0.connect_from(self.pointer_device_sensor.Button0)
        self.sf_pointer_button1.connect_from(self.pointer_device_sensor.Button1)
        self.ray_transform.Transform.connect_from(self.pointer_tracking_sensor.Matrix)


    
    ### callbacks ###
    @field_has_changed(mf_dof)
    def mf_dof_changed(self):                     
        ## handle translation input
        _x = self.mf_dof.value[0]
        _y = self.mf_dof.value[1]
        _z = self.mf_dof.value[2]

        _trans_vec    = avango.gua.Vec3(_x, _y, _z) * self.attenuation_factor
        _trans_input  = _trans_vec.length()
         
        if _trans_input > 0.0:

            ## transfer-function for translation
            #_exponent  = 3
            _exponent   = 2

            _multiple = int(_trans_input)
            _rest       = _trans_input - _multiple
            _factor     = _multiple + pow(_rest, _exponent)

            _trans_vec.normalize()
            _trans_vec = _trans_vec * _factor * 0.1

        ## handle rotation input
        _rx = self.mf_dof.value[3]
        _ry = self.mf_dof.value[4]
        _rz = self.mf_dof.value[5]

        _rot_vec    = avango.gua.Vec3(_rx, _ry, _rz) * self.attenuation_factor
        _rot_input  = _rot_vec.length()

        ## accumulate input        
        if self.navigation_mode == 0: # steering mode
            ## implement steering input
            # ...
            if _trans_input or _rot_input > 0.0:
                self.sf_nav_mat.value = self.sf_nav_mat.value * \
                                    avango.gua.make_trans_mat(_trans_vec) * \
                                    avango.gua.make_trans_mat(self.rot_center_offset) * \
                                    avango.gua.make_rot_mat(_rot_vec.y,0,1,0) * \
                                    avango.gua.make_rot_mat(_rot_vec.x,1,0,0) * \
                                    avango.gua.make_rot_mat(_rot_vec.z,0,0,1) * \
                                    avango.gua.make_trans_mat(self.rot_center_offset * -1)

        elif self.navigation_mode == 1: # maneuvering mode
            ## implement maneuvering input
            # ...
            
            pass


    @field_has_changed(sf_pointer_button0)
    def sf_pointer_button0_changed(self):
        ## toogle between steering and maneuvering mode    
        # ...
        
        pass


    @field_has_changed(sf_pointer_button1)
    def sf_pointer_button1_changed(self):
        ## enable Navidget target-mode on button press
        # ...
    
        ## start Navidget animation-mode on button release
        # ...
        
        pass
    
    

    def frame_callback(self): # executed every frame when active
        ## calc intersection
        _mf_pick_result = self.intersection.calc_pick_result(PICK_MAT = self.ray_transform.WorldTransform.value, PICK_LENGTH = self.ray_length)
        self.mf_pointer_pick_result.value = _mf_pick_result.value

        self.update_ray_parameters()

        if self.navigation_mode == 3: # Navidget target-mode
            self.update_navidget_parameters()

        elif self.navigation_mode == 4: # Navidget animation-mode
            self.animate_navidget()
                          

    ### functions ###
    def set_navigation_mode(self, MODE):
        self.navigation_mode = MODE
        
        if self.navigation_mode == 0: # switch to Steering mode
            # set other modes geometry invisible
            self.maneuvering_point_geometry.Tags.value = ["invisible"]
            self.navidget_node.Tags.value = ["invisible"]

            print("SWITCH TO STEERING MODE")

        elif self.navigation_mode == 1: # switch to maneuvering mode
            self.maneuvering_point_geometry.Tags.value = []

            print("SWITCH TO MANEUVERING MODE")
        
        elif self.navigation_mode == 3: # switch to Navidget target mode
                    
            if len(self.mf_pointer_pick_result.value) > 0: # intersection found
                _pick_result = self.mf_pointer_pick_result.value[0] # get first intersection target

                ## set initial Navidget GUI position and orientation
                # ...

                print("SWITCH TO NAVIDGET TARGET-MODE")

        elif self.navigation_mode == 4:        
            ## define start and target parameters for Navidget animation
            # ...

            print("SWITCH TO NAVIDGET ANIMATION-MODE")


    def update_ray_parameters(self):
        if len(self.mf_pointer_pick_result.value) > 0: # intersection found
            _pick_result = self.mf_pointer_pick_result.value[0] # get first intersection target

            _point = _pick_result.WorldPosition.value # intersection point in world coordinate system
            _distance = (_point - self.ray_transform.WorldTransform.value.get_translate()).length()

            # update intersection point visualization
            self.intersection_point_geometry.Tags.value = [] # set geometry visible

            self.intersection_point_geometry.Transform.value = avango.gua.make_trans_mat(_point) * \
                                                               avango.gua.make_scale_mat(self.intersection_point_size)


            # update ray visualization (ray length)
            self.ray_geometry.Transform.value = avango.gua.make_trans_mat(0.0,0.0,_distance * -0.5) * \
                                                avango.gua.make_scale_mat(self.ray_thickness,self.ray_thickness,_distance)

        else: # no intersection found
            # update intersection point visualization        
            self.intersection_point_geometry.Tags.value = ["invisible"] # set geometry invisible

            # update ray visualization (ray length)            
            self.ray_geometry.Transform.value = avango.gua.make_trans_mat(0.0,0.0,self.ray_length * -0.5) * \
                                                avango.gua.make_scale_mat(self.ray_thickness,self.ray_thickness,self.ray_length)
                                            
                                            
    def update_navidget_parameters(self):       
        ## update Navidget gui elements if Navidget GUI-sphere is hit
        # ...
        
        ## use the get_rotation_between_vectors() function to obtain the rotation matrix between a reference direction, e.g. Vec3(0.0,0.0,-1.0) the default viewing direction in world space, and a target direction
        # ...
        
        pass

    
    def animate_navidget(self):
        # use linear interpolation between two vectors to animate the position of the camera --> use function lerp_to() on vector
        # ...

        # use spherical linear interpolation between two quaternions to animate the orientation of the camera --> use function slerp_to on quaternion
        # ...
        
        # stop Navidget animation mode after animation duration has exceeded
        # ...

        pass


    def get_rotation_matrix_between_vectors(self, VEC1, VEC2):
        VEC1.normalize()
        VEC2.normalize()

        _angle = math.degrees(math.acos(VEC1.dot(VEC2)))
        _axis = VEC1.cross(VEC2)

        return avango.gua.make_rot_mat(_angle, _axis)
        
