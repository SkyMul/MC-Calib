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
bpy.ops.wm.open_mainfile(filepath=path_direct + 'Cube023467.blend')

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
cam1.angle = 45

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
eul_mat_2 = mathutils.Euler((0.0, math.radians(-180.0), math.radians(-0.0)), 'XYZ').to_matrix()
mat_loc_2 = mathutils.Matrix.Translation((0,0,-12)) # FRONT FACING
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
eul_mat_3 = mathutils.Euler((0.0, math.radians(-90), math.radians(-0)), 'XYZ').to_matrix()
mat_loc_3 = mathutils.Matrix.Translation((-6,0,-6))
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
eul_mat_4 = mathutils.Euler((0.0, math.radians(90), 0.0), 'XYZ').to_matrix()
mat_loc_4 = mathutils.Matrix.Translation((6.0,0,-6))
cam4_transform = mat_loc_4 * eul_mat_4.to_4x4()


# Set render resolution
scene.render.resolution_x = 3648/2
scene.render.resolution_y = 2752/2
scene.render.resolution_percentage = 100

#make the camera look in the direction of the object
look_at(obj_camera, Board.matrix_world.to_translation())

#Generate cameras on a "hemisphere", different distance and positions
NbOfIm = 100;

#Range of the camera motions
range_az = [0, 180]
range_el = [10, 160]
range_d = [5, 7]

for i in range(1, NbOfIm):

    # Sample a position on the sphere
    az_val = (range_az[1]-range_az[0])*np.random.uniform(0, 1) + range_az[0];
    el_val = (range_el[1]-range_el[0])*np.random.uniform(0, 1) + range_el[0];
    d_val = (range_d[1]-range_d[0])*np.random.uniform(0, 1) + range_d[0];
    [x,y,z] = sph2cart(np.deg2rad(az_val), np.deg2rad(el_val) ,d_val);
    t = [x,y,z]

    #Move the camera
    obj_camera.location = (x, y, z)
    bpy.context.scene.update()

    #Make the camera 1 look at the point 0 0 0
    pts = Vector((0,0,0)) 
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

	


