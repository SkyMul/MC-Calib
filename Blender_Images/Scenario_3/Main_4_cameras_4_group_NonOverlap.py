import bpy
import bpy_extras
import numpy as np
from math import *
from PIL import Image
from mathutils import Matrix
from mathutils import Vector
import imageio
from skimage.draw import circle
from skimage.transform import rescale, resize
import matplotlib.pyplot as plt
import random
import pdb
import json
import cv2
import math
import mathutils


# Build intrinsic camera parameters from Blender camera data
#
# See notes on this in 
# blender.stackexchange.com/questions/15102/what-is-blenders-camera-projection-matrix-model

def get_calibration_matrix_K_from_blender(camd):
    f_in_mm = camd.lens
    scene = bpy.context.scene
    resolution_x_in_px = scene.render.resolution_x
    resolution_y_in_px = scene.render.resolution_y
    scale = scene.render.resolution_percentage / 100
    sensor_width_in_mm = camd.sensor_width
    sensor_height_in_mm = camd.sensor_height
    pixel_aspect_ratio = scene.render.pixel_aspect_x / scene.render.pixel_aspect_y
    if (camd.sensor_fit == 'VERTICAL'):
        # the sensor height is fixed (sensor fit is horizontal), 
        # the sensor width is effectively changed with the pixel aspect ratio
        s_u = resolution_x_in_px * scale / sensor_width_in_mm / pixel_aspect_ratio 
        s_v = resolution_y_in_px * scale / sensor_height_in_mm
    else: # 'HORIZONTAL' and 'AUTO'
        # the sensor width is fixed (sensor fit is horizontal), 
        # the sensor height is effectively changed with the pixel aspect ratio
        pixel_aspect_ratio = scene.render.pixel_aspect_x / scene.render.pixel_aspect_y
        s_u = resolution_x_in_px * scale / sensor_width_in_mm
        s_v = resolution_y_in_px * scale * pixel_aspect_ratio / sensor_height_in_mm
    

    # Parameters of intrinsic calibration matrix K
    alpha_u = f_in_mm * s_u
    alpha_v = f_in_mm * s_v
    u_0 = resolution_x_in_px * scale / 2
    v_0 = resolution_y_in_px * scale / 2
    skew = 0 # only use rectangular pixels

    K = Matrix(
        ((alpha_u, skew,    u_0),
        (    0  , alpha_v, v_0),
        (    0  , 0,        1 )))
    return K


def totuple(a):
    try:
        return tuple(totuple(i) for i in a)
    except TypeError:
        return a


def look_at(obj_camera, point):
    loc_camera = obj_camera.matrix_world.to_translation()

    direction = point - loc_camera
    # point the cameras '-Z' and use its 'Y' as up
    rot_quat = direction.to_track_quat('-Z', 'Y')

    # assume we're using euler rotation
    obj_camera.rotation_euler = rot_quat.to_euler()

    #Return the quaternion
    return rot_quat

def sph2cart(azimuth,elevation,r):
    x = r * np.cos(elevation) * np.cos(azimuth)
    y = r * np.cos(elevation) * np.sin(azimuth)
    z = r * np.sin(elevation)
    return x, y, z

def project_with_matrix( scene, camera, location):

    width, height = scene.render.resolution_x, scene.render.resolution_y
    persp_mat = camera.calc_matrix_camera( x=width, y=height) * camera.matrix_world.inverted()
    loc = persp_mat * location.to_4d()
    
    return (loc.xyz / (2 * loc.w)) + Vector((0.5, 0.5, 0.5))



#Working directory
path_direct = "/home/francois/Documents/CppProject/Stereo_Calibration/Simulation/"
JsonFile = "collection.json"

#Prepare the csv file to write (motion of the camera)
file1 = open(path_direct + "TrainingCar.csv","w") 
L = ["ImagePath, t1, t2, t3, r1, r2, r3 \n"]  
file1.writelines(L) 

#Load the file
bpy.ops.wm.open_mainfile(filepath=path_direct + '8_Boards_NOnOverlap_4groups.blend')

#Path to save images
SavePath = path_direct + "Images/" 
#bpy.context.scene.render.image_settings.color_mode ='RGBA'
#bpy.context.scene.render.alpha_mode = 'TRANSPARENT' 

#List the object in the scene
objs = [obj for obj in bpy.data.objects]
#pdb.set_trace()
Board = bpy.data.objects['charuco_board_000.001']
#bpy.ops.object.origin_set(type='ORIGIN_GEOMETRY', center='BOUNDS')

# create the first camera
cam1 = bpy.data.cameras.new("Camera 1")
#cam1.lens = 18
cam1.angle = math.radians(60)

# create the camera object
cam_obj1 = bpy.data.objects.new("Camera1", cam1)
cam_obj1.location = (9.69, -10.85, 12.388)
cam_obj1.rotation_euler = (0.6799, 0, 0.8254)
scene = bpy.context.scene
scene.objects.link(cam_obj1)
bpy.context.scene.update()
obj_camera = bpy.data.objects["Camera1"]
scene.camera = obj_camera

#Generate the second camera
cam_obj2 = bpy.data.objects.new("Camera2", cam1)
cam_obj2.location = (9.69, -10.85, 12.388)
cam_obj2.rotation_euler = (0.6799, 0, 0.8254)
scene.objects.link(cam_obj2)
bpy.context.scene.update()
obj_camera2 = bpy.data.objects["Camera2"]
scene.camera = obj_camera2
# Pose of the second camera wrt to the first camera
eul_mat_2 = mathutils.Euler((0.0, math.radians(180.0), 0.0), 'XYZ').to_matrix()
mat_loc_2 = mathutils.Matrix.Translation((0,0,0.5))
cam2_transform = mat_loc_2 * eul_mat_2.to_4x4()


#Generate the third camera
cam_obj3 = bpy.data.objects.new("Camera3", cam1)
cam_obj3.location = (9.69, -10.85, 12.388)
cam_obj3.rotation_euler = (0.6799, 0, 0.8254)
scene.objects.link(cam_obj3)
bpy.context.scene.update()
obj_camera3 = bpy.data.objects["Camera3"]
scene.camera = obj_camera3
# Pose of the third camera wrt to the first camera
eul_mat_3 = mathutils.Euler((0.0, math.radians(90), 0.0), 'XYZ').to_matrix()
mat_loc_3 = mathutils.Matrix.Translation((-0.25,0,0.25))
cam3_transform = mat_loc_3 * eul_mat_3.to_4x4()


#Generate the fourth camera
cam_obj4 = bpy.data.objects.new("Camera4", cam1)
cam_obj4.location = (9.69, -10.85, 12.388)
cam_obj4.rotation_euler = (0.6799, 0, 0.8254)
scene.objects.link(cam_obj4)
bpy.context.scene.update()
obj_camera4 = bpy.data.objects["Camera4"]
scene.camera = obj_camera4
# Pose of the second camera wrt to the first camera
eul_mat_4 = mathutils.Euler((0.0, math.radians(-90.0), 0.0), 'XYZ').to_matrix()
mat_loc_4 = mathutils.Matrix.Translation((0.25,0.0,0.25))
cam4_transform = mat_loc_4 * eul_mat_4.to_4x4()

#Write GT of the camera pose
filenameGT = "GroundTruth.yml"
fs = cv2.FileStorage(filenameGT, cv2.FILE_STORAGE_WRITE)
fs.write("nb_camera", 4)
K1_blender = get_calibration_matrix_K_from_blender(cam_obj1.data)
K1 = np.identity(3)
K1[0,0] = K1_blender[0][0]
K1[1,1] = K1_blender[1][1]
K1[0,2] = K1_blender[0][2]
K1[1,2] = K1_blender[1][2]
fs.write("K_1", K1)
fs.write("P_1", np.identity(4))

K2_blender = get_calibration_matrix_K_from_blender(cam_obj2.data)
K2 = np.identity(3)
K2[0,0] = K2_blender[0][0]
K2[1,1] = K2_blender[1][1]
K2[0,2] = K2_blender[0][2]
K2[1,2] = K2_blender[1][2]
fs.write("K_2", K2)
P2 = np.identity(4)
for i in range(0,4):
    for j in range(0,4):
        P2[i,j] = cam2_transform[i][j]
fs.write("P_2", P2)

K3_blender = get_calibration_matrix_K_from_blender(cam_obj3.data)
K3 = np.identity(3)
K3[0,0] = K3_blender[0][0]
K3[1,1] = K3_blender[1][1]
K3[0,2] = K3_blender[0][2]
K3[1,2] = K3_blender[1][2]
fs.write("K_3", K3)
P3 = np.identity(4)
for i in range(0,4):
    for j in range(0,4):
        P3[i,j] = cam3_transform[i][j]
fs.write("P_3", P3)

K4_blender = get_calibration_matrix_K_from_blender(cam_obj4.data)
K4 = np.identity(3)
K4[0,0] = K4_blender[0][0]
K4[1,1] = K4_blender[1][1]
K4[0,2] = K4_blender[0][2]
K4[1,2] = K4_blender[1][2]
fs.write("K_4", K4)
P4 = np.identity(4)
for i in range(0,4):
    for j in range(0,4):
        P4[i,j] = cam4_transform[i][j]
fs.write("P_4", P4)

fs.release()

# Set render resolution
scene.render.resolution_x = 3648/2
scene.render.resolution_y = 2752/2
scene.render.resolution_percentage = 100

#make the camera look in the direction of the object
look_at(obj_camera, Board.matrix_world.to_translation())

#Generate cameras on a "hemisphere", different distance and positions
NbOfIm = 100;
#Range of the camera motions
range_cubex = [-0.5, 0.5]
range_cubey = [-3.5, -2.5]
range_cubez = [-0.5, 0.5]

range_ptsplanex = [-0.5, 0.5]
range_ptsplaney = [-0.5, 0.5]

for i in range(1, NbOfIm):

    # Random translation in front of the board in a range 0 -2 cube
    x = (range_cubex[1]-range_cubex[0])*np.random.uniform(0, 1) + range_cubex[0];
    y = (range_cubey[1]-range_cubey[0])*np.random.uniform(0, 1) + range_cubey[0];
    z = (range_cubez[1]-range_cubez[0])*np.random.uniform(0, 1) + range_cubez[0];
    t = [x,y,z]

    #Move the camera
    obj_camera.location = (x, y, z)
    bpy.context.scene.update()

    #Make the camera 1 look at a point on a plane Y X
    ptsx = (range_ptsplanex[1]-range_ptsplanex[0])*np.random.uniform(0, 1) + range_ptsplanex[0];
    ptsy = (range_ptsplaney[1]-range_ptsplaney[0])*np.random.uniform(0, 1) + range_ptsplaney[0];
    ptsz = 0
    pts = Vector((ptsx,ptsy,ptsz)) 
    rot_quat = look_at(obj_camera, pts)
    mat_rot = obj_camera.rotation_euler.to_matrix()
    mat_loc = obj_camera.location
    mat_transform =  mathutils.Matrix.Translation(mat_loc) * mat_rot.to_4x4()
    
    #Save camera 1
    #bpy.context.scene.render.filepath = SavePath+ str(i) + '.jpg'
    bpy.context.scene.camera = bpy.context.scene.objects["Camera1"]
    bpy.context.scene.render.filepath = 'temp.png'
    bpy.ops.render.render(write_still = True) 
    im = np.array(Image.open('temp.png'))
    Iname_save = str(i).zfill(5) + '.png'
    imageio.imwrite(SavePath + "Cam_001/" + Iname_save, im) 

    # Place camera 2 and save
    mat_pose_2 = mat_transform*cam2_transform.inverted()
    obj_camera2.location = mat_pose_2.to_translation()
    obj_camera2.rotation_euler = mat_pose_2.to_3x3().to_euler()
    bpy.context.scene.camera = bpy.context.scene.objects["Camera2"]
    bpy.context.scene.render.filepath = 'temp.png'
    bpy.ops.render.render(write_still = True) 
    im = np.array(Image.open('temp.png'))
    Iname_save = str(i).zfill(5) + '.png'
    imageio.imwrite(SavePath + "Cam_002/" + Iname_save, im) 

    # Place camera 3 and save
    mat_pose_3 = mat_transform*cam3_transform.inverted()
    obj_camera3.location = mat_pose_3.to_translation()
    obj_camera3.rotation_euler = mat_pose_3.to_3x3().to_euler()
    bpy.context.scene.camera = bpy.context.scene.objects["Camera3"]
    bpy.context.scene.render.filepath = 'temp.png'
    bpy.ops.render.render(write_still = True) 
    im = np.array(Image.open('temp.png'))
    Iname_save = str(i).zfill(5) + '.png'
    imageio.imwrite(SavePath + "Cam_003/" + Iname_save, im) 


    # Place camera 4 and save
    mat_pose_4 = mat_transform*cam4_transform.inverted()
    obj_camera4.location = mat_pose_4.to_translation()
    obj_camera4.rotation_euler = mat_pose_4.to_3x3().to_euler()
    bpy.context.scene.camera = bpy.context.scene.objects["Camera4"]
    bpy.context.scene.render.filepath = 'temp.png'
    bpy.ops.render.render(write_still = True) 
    im = np.array(Image.open('temp.png'))
    Iname_save = str(i).zfill(5) + '.png'
    imageio.imwrite(SavePath + "Cam_004/" + Iname_save, im) 


    #place camera 2
    #pdb.set_trace()
    
    

file1.close()

	


