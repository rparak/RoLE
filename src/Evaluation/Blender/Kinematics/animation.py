def Generate_Robot_Path_From_Animation(Robot_Cls: Lib.Blender.Core.Robot_Cls, SD_Poly_Cls: Lib.Blender.Core.Poly_3D_Cls):
    """
    Description:
        Generate the robot's path from the animation.
        
    Args:
        (1) Robot_Cls [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.
        (2) SD_Poly_Cls [Poly_3D_Cls(object)]: Visualization of a 3-D (dimensional) polyline.
    """
    
    # Initialize the size (length) of the polyline data set.
    SD_Poly_Cls.Initialization(bpy.context.scene.frame_end + 1)
    
    for frame in range(bpy.context.scene.frame_start, bpy.context.scene.frame_end + 1):
        bpy.context.scene.frame_set(frame)
        
        # Get the current absolute joint position in radians / meters.
        th = Get_Absolute_Joint_Position(Robot_Parameters_Str)

        # Add coordinates (x,y,z points) calculated from Forward Kinematics 
        # to the polyline.
        x, y, z = Kinematics.Forward_Kinematics(th, 'Modified', Robot_Parameters_Str)[1].p.all()
        SD_Poly_Cls.Add(frame, x, y, z) 