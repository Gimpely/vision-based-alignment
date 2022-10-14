 #!/usr/bin/env python
from copy import deepcopy
import os
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import Image
import sensor_msgs.point_cloud2 as pc2
import cv_bridge
import cv2 as cv
import math
import matplotlib.pyplot as plt
import open3d
import copy
import tf.transformations as tr



##### TO DO ###########
#### RANSAC parametri invariantni na kot
#### filtriranje transformacije
#### probaj pohitreti --> multithreding?
#### odometija transformacije

stage = 2

### Izmeri kot kamere glede na vertikalo
    
    
                                        #35
def pointcloud_object(cloud_data, kot=23.5, viz=False, debug=False):
    kot = np.radians(kot)
    if debug:
        viz = False
        mesh = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=np.array([0.0, 0.0, 0.0]))
        print("1.")

    open3d_cloud = open3d.geometry.PointCloud()
    xyz = [(x,y,z) for x,y,z,rgb in cloud_data ]
    open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))
    

    

    center_cloud = open3d.geometry.PointCloud()
    coord = np.array([[0,0,0]]) # Ustvari pointcloud iz tega
    center_cloud.points = open3d.utility.Vector3dVector(coord)

    
    downpcd = open3d_cloud.voxel_down_sample(voxel_size=0.01)


    if debug:
        print("Osnovna slika")
        open3d.visualization.draw_geometries([downpcd, mesh]) ##############

    points = np.asarray(downpcd.points)
    dists = downpcd.compute_point_cloud_distance(center_cloud)
    dists = np.asarray(dists)
    min_idx = np.argmin(dists)
    cc = points[min_idx][2]

    c = downpcd.get_center()
    downpcd.translate((-c[0],-c[1],-c[2]))
    if debug:
        print("Premik v center")
        open3d.visualization.draw_geometries([downpcd, mesh]) ##############

    R = downpcd.get_rotation_matrix_from_axis_angle([-kot,0,0])
    downpcd.rotate(R, center=True)
    if debug:
        print("Rotacija")
        open3d.visualization.draw_geometries([downpcd, mesh]) ##############

    downpcd.translate((c[0],(c[1]),c[2]))
    if debug:
        print("Premik iz centra")
        open3d.visualization.draw_geometries([downpcd, mesh]) ##############


    points = np.asarray(downpcd.points)
    dists = downpcd.compute_point_cloud_distance(center_cloud)
    dists = np.asarray(dists)
    min_idx = np.argmin(dists)


    downpcd.translate((0,-points[min_idx][1] + cc,0))
    if debug:
        print("Premik v izhodiscno pozicijo")
        open3d.visualization.draw_geometries([downpcd, mesh]) ##############
    
 
    ########################### visina kamere v metrih ################################
    #visina_kamere = 0.4
    visina_kamere = cc #+ 0.05
    ###############################################################################
    points = np.asarray(downpcd.points)
    cloud_Z = downpcd.select_down_sample(np.where(points[:,1] < visina_kamere)[0])
    points = np.asarray(cloud_Z.points)
    cloud_Z = cloud_Z.select_down_sample(np.where(points[:,2] < 1.8)[0])
    if debug:
        print("Rezanje po visini in dolzini vec kot 1.8m")
        open3d.visualization.draw_geometries([cloud_Z, mesh]) ##############
   

    plane_model, inliers = cloud_Z.segment_plane(distance_threshold=0.01,ransac_n=3,num_iterations=1000)
    [a, b, c, d] = plane_model
    outlier_cloud = cloud_Z.select_down_sample(inliers, invert=True)
    if debug:
        print("Odstranitev ravnin")
        open3d.visualization.draw_geometries([outlier_cloud, mesh]) ##############
 
    cl, ind = outlier_cloud.remove_radius_outlier(nb_points=50, radius=0.08)
    new_cloud = outlier_cloud.select_down_sample(ind)
    if debug:
        print("Filtriranje")
        open3d.visualization.draw_geometries([new_cloud, mesh]) ##############
    """
    points = np.asarray(new_cloud.points)
    dists = new_cloud.compute_point_cloud_distance(center_cloud)
    dists = np.asarray(dists)
    min_idx = np.argmin(dists)
    x = points[min_idx][0]
    y = points[min_idx][1]
    z = points[min_idx][2]

    """
    if viz:
        mesh = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=np.array([0.0, 0.0, 0.0]))
        open3d.visualization.draw_geometries([downpcd, mesh]) ##############
        open3d.visualization.draw_geometries([new_cloud, mesh]) ##############

    return new_cloud, open3d_cloud

################# izris obeh leg z transformacijo ########################

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)

    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.paint_uniform_color([1, 0.706, 0])

    source_temp.transform(transformation)
    mesh = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=np.array([0.0, 0.0, 0.0]))
    open3d.visualization.draw_geometries([source_temp, target_temp, mesh])

######### priprava pointclouda za obdelavo ###################

def preprocess_point_cloud(pcd, voxel_size): 
    pcd_down = pcd.voxel_down_sample(voxel_size)
    radius_normal = voxel_size * 2
    pcd_down.estimate_normals(open3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    pcd_fpfh = open3d.registration.compute_fpfh_feature(pcd_down,open3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

############### RANSAC iskanje transformacije emd zeljeno in trenutno lego ########################

def execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    result = open3d.registration.registration_ransac_based_on_feature_matching(source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
            open3d.registration.TransformationEstimationPointToPoint(False), 4, 
            [open3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),open3d.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)],
             open3d.registration.RANSACConvergenceCriteria(1000000, 600))
    return result

################# Rotacijska matrika v Eulerjeve kote  ################################

def rotationMatrixToEulerAngles(R) :
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else:
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return np.array([x, y, z])

def eulerToRotMatrix(y_kot):
    theta1 = 0
    theta2 = np.rad2deg(y_kot)
    theta3 = 0

    c1 = np.cos(theta1 * np.pi / 180)
    s1 = np.sin(theta1 * np.pi / 180)
    c2 = np.cos(theta2 * np.pi / 180)
    s2 = np.sin(theta2 * np.pi / 180)
    c3 = np.cos(theta3 * np.pi / 180)
    s3 = np.sin(theta3 * np.pi / 180)

    matrix=np.array([[c2*c3, -c2*s3, s2],
                     [c1*s3+c3*s1*s2, c1*c3-s1*s2*s3, -c2*s1],
                     [s1*s3-c1*c3*s2, c3*s1+c1*s2*s3, c1*c2]])
    return matrix


################# pridobitev transformacijske matrike, vrne T in R matriko ter rotacijo kot kvaternion ##########################
cnt = 0
def get_Transformation(source, target, voxel_size=0.02, debug=False, viz=False):
    crit = True
    global stage
    global cnt
    global korak
   

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size) 

    
    if stage == 2:

        cnt = 0         
        while crit:
            cnt += 1
            result_ransac = execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size)
            # print(result_ransac.inlier_rmse),
            # print(result_ransac.fitness)
            print("Trying")
            if debug:
                draw_registration_result(source_down, target_down, result_ransac.transformation)
            if (result_ransac.inlier_rmse < 0.016) and (result_ransac.fitness > 0.4) : ### za 22.6 stopinj dej rmse<0.016 pa fit>0.4
                                                                                        ### za 35 stopinj dej rmse<0.016 pa fit>0.48
                
               
                # print(result_ransac)
                rezult = result_ransac.transformation
                if viz:
                    draw_registration_result(source_down, target_down, result_ransac.transformation)
                # print("\n")
                # print(np.linalg.inv(rezult))
                T = np.linalg.inv(rezult)

                camera = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=np.array([0,0,0])) 
                target.paint_uniform_color([0, 0, 1])
                camera.transform(np.linalg.inv(rezult))
                mesh = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=np.array([0,0,0]))
                R = rezult[:3,:3]
                y_Eu = rotationMatrixToEulerAngles(R)[1]
                z_Eu = rotationMatrixToEulerAngles(R)[2]
                

                q = tr.quaternion_from_matrix(rezult)
                # print("Quaternion: ", q)

                # print("zarotiraj za: ",np.rad2deg(y_Eu))

                if viz:
                    open3d.visualization.draw_geometries([ camera, source,mesh])
                crit = False
                err = True
                cnt = 0
                return err, T, q
            elif cnt == 10:
                err = False
                return err
    
    if stage == 1:

        center_cloud = open3d.geometry.PointCloud()
        coord = np.array([[0,0,0]]) # Ustvari pointcloud iz tega
        center_cloud.points = open3d.utility.Vector3dVector(coord)
        points = np.asarray(source.points)
        dists = source.compute_point_cloud_distance(center_cloud)
        dists = np.asarray(dists)
        min_idx = np.argmin(dists) 
        #print(points[min_idx]) 
        x = points[min_idx][0]
        z = points[min_idx][2]
        z = z + 0.3
        
        trans_init = np.asarray([[0.0, 0.0, -1.0, z],
                                [0.0, 1.0, 0.0, 0.0],
                                [1.0, 0.0, 0.0, x],
                                [0.0, 0.0, 0.0, 1.0]])

        source_temp_temp = copy.deepcopy(source)
        camera = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=np.array([0,0,0]))

        source.transform(trans_init)
        
   

        cut = points[min_idx][0] - 0.65
        source = source.select_down_sample(np.where(points[:,0] > cut)[0]) ############## odstrani steno....izbrisi ce stene ni ####################
       
        

        points = np.asarray(source.points)
        dists = source.compute_point_cloud_distance(center_cloud)
        dists = np.asarray(dists)
        min_idx = np.argmin(dists)
        z = points[min_idx][2]

        t_points = np.asarray(target.points)
        dists = source.compute_point_cloud_distance(center_cloud)
        dists = np.asarray(dists)
        min_idx = np.argmin(dists)
        t_z = t_points[min_idx][2]

        if debug:
            target.paint_uniform_color([1, 0, 1])
            open3d.visualization.draw_geometries([ camera, source, target])


        www= t_z - z
        print(z),
        print(t_z),
        print(www)        
        trans = np.asarray([[1.0, 0.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0, 0.0],
                            [0.0, 0.0, 1.0, www],
                            [0.0, 0.0, 0.0, 1.0]])

        #source.transform(trans)
        if debug:
            target.paint_uniform_color([1, 0, 1])
            open3d.visualization.draw_geometries([ camera, source, target])

        source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
        target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
        cnt = 0 
        while crit:
            cnt += 1
            
            result_ransac = execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size)
            #TR_tmp = np.matmul(trans, trans_init)
            TR = np.matmul(result_ransac.transformation, trans_init)
            RR = TR[:3,:3]
            y_Eu_og = rotationMatrixToEulerAngles(RR)[1]
            invTR = np.linalg.inv(TR)
            R = invTR[:3,:3]
            y_Eu = rotationMatrixToEulerAngles(R)[1]
            z_Eu = rotationMatrixToEulerAngles(R)[2]
            d = 90 - np.abs(np.rad2deg(y_Eu))
            # print(result_ransac.inlier_rmse),
            # print(result_ransac.fitness),
            # print(d)
            print("Trying")
            
            
            if debug:
                draw_registration_result(source_down, target_down, result_ransac.transformation)
            if ((result_ransac.inlier_rmse < 0.01) or (result_ransac.fitness > 0.42)) and (np.abs(d) < 30):
                # print("\n")
                # print(result_ransac)
                if viz:
                    draw_registration_result(source_down, target_down, result_ransac.transformation)
                # print("\n")
                # print(np.rad2deg(y_Eu_og)),
                # print(np.rad2deg(y_Eu))
                # print("\n")
                y_only_metrix = eulerToRotMatrix(y_Eu)
                
                # print(invTR)
                # print("\n")
         
                invTR[0,0] = y_only_metrix[0,0]
                invTR[1,0] = y_only_metrix[1,0]
                invTR[2,0] = y_only_metrix[2,0]
                invTR[0,1] = y_only_metrix[0,1]
                invTR[1,1] = y_only_metrix[1,1]
                invTR[2,1] = y_only_metrix[2,1]
                invTR[0,2] = y_only_metrix[0,2]
                invTR[1,2] = y_only_metrix[1,2]
                invTR[2,2] = y_only_metrix[2,2]

                q = tr.quaternion_from_matrix(invTR)
                # print("Quaternion: ", q)
          
                
                
                

                #print("zarotiraj za: ",np.rad2deg(y_Eu))
                camera.transform(invTR)
                mesh = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=np.array([0,0,0]))
                
                # print("\n")
                # print(invTR)
                T = invTR

                

                #q = tr.quaternion_from_matrix(np.linalg.inv(TR))
                #print("invQuaternion: ", q)
                
                
                """
                dd = 180 - np.abs(np.rad2deg(z_Eu))
                print(dd)
                if np.abs(dd) < 50:
                    print("inv")
                    trans = np.asarray([[-1.0, 0.0, 0.0, 0.0],
                                        [0.0, -1.0, 0.0, 0.0],
                                        [0.0, 0.0, 1.0, 0.0],
                                        [0.0, 0.0, 0.0, 1.0]])



                    camera.transform(np.linalg.inv(trans))
                """
                if (viz) or (debug):
                    #open3d.visualization.draw_geometries([ camera, source_temp_temp,mesh])

                    camerra = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=np.array([0,0,0]))
                    camerra.transform(np.linalg.inv(TR))
                    open3d.visualization.draw_geometries([ camerra, source_temp_temp,mesh, camera])
                crit = False
                err = True
                cnt = 0
                return err, T, q
            elif cnt == 10:
                err = False
                return err


    




################### dodatno poravnavanje leg #####################

def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size, result_ransac):
    distance_threshold = voxel_size * 0.2
    
    result = open3d.registration.registration_icp(source, target, 0.02, result_ransac, open3d.registration.TransformationEstimationPointToPoint())
    return result


def makeTarget(pcl):
    mesh = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.4, origin=np.array([0.0, 0.0, 0.0]))
    open3d.visualization.draw_geometries([pcl, mesh])
    points = np.asarray(pcl.points)
    
    
    origin = open3d.geometry.PointCloud()
    origin_coord = np.array([[0,0,0]]) 
    origin.points = open3d.utility.Vector3dVector(origin_coord)

    points = np.asarray(pcl.points)
    dists = pcl.compute_point_cloud_distance(origin)
    dists = np.asarray(dists)
    min_idx = np.argmin(dists)
    z = points[min_idx][2]

    zz = z + 0.25
    start = pcl.select_down_sample(np.where(points[:,2] < zz)[0])

    
    
    open3d.visualization.draw_geometries([start, mesh])
    #open3d.io.write_point_cloud("target_pointcloud_2_rob.pcd", start)

def makeCev(target):
    center_cloud = open3d.geometry.PointCloud()
    coord = np.array([[0,0,0]]) # Ustvari pointcloud iz tega
    center_cloud.points = open3d.utility.Vector3dVector(coord)


    points = np.asarray(target.points)
    dists = target.compute_point_cloud_distance(center_cloud)
    dists = np.asarray(dists)
    min_idx = np.argmin(dists)
    z = points[min_idx][2]
   # target.translate((0,0,-z))
    pcd = target.select_down_sample(np.where(points[:,2] > 0.6)[0])
    mesh = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=np.array([0.0, 0.0, 0.0]))
    open3d.visualization.draw_geometries([pcd, mesh])
    #open3d.io.write_point_cloud("target_cevi.pcd", pcd)


def findStart(source):

    origin = open3d.geometry.PointCloud()
    origin_coord = np.array([[0,0,0]]) 
    origin.points = open3d.utility.Vector3dVector(origin_coord)

    points = np.asarray(source.points)
    dists = source.compute_point_cloud_distance(origin)
    dists = np.asarray(dists)
    min_idx = np.argmin(dists)
    z = points[min_idx][2]

    zz = z + 0.25
    start = source.select_down_sample(np.where(points[:,2] < zz)[0])

    return start





def Pointcloud_Callback(data):
    global buffq
    global stage
    global kot
    global korak
    cloud_data = list(pc2.read_points(data, field_names = ("x", "y", "z","rgb"), skip_nans=True))
    mesh = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.4, origin=np.array([0.0, 0.0, 0.0]))
    
    source, original = pointcloud_object(cloud_data, viz=False, debug=False)

    target = open3d.io.read_point_cloud("/home/staublixpc/catkin_ws/src/nav_mobile/lab_AGV-main/pcl_targets/target_pointcloud_2.pcd")
    short_target = open3d.io.read_point_cloud("/home/staublixpc/catkin_ws/src/nav_mobile/lab_AGV-main/pcl_targets/target_pointcloud_2_rob.pcd")

    voxel_size=0.02

    if stage == 1:

        draw_registration_result(source, target, np.identity(4))
        opcija = 1
        if opcija == 1:

            err = get_Transformation(source, target, voxel_size, debug=True, viz=True)
            if err:
                print("Done")
            else:
                print("Failed")

                   
        if opcija == 0:
            center_cloud = open3d.geometry.PointCloud()
            coord = np.array([[0,0,0]]) # Ustvari pointcloud iz tega
            center_cloud.points = open3d.utility.Vector3dVector(coord)
            points = np.asarray(source.points)
            dists = source.compute_point_cloud_distance(center_cloud)
            dists = np.asarray(dists)
            min_idx = np.argmin(dists) 
            #print(points[min_idx]) 
            x = points[min_idx][0]
            y = points[min_idx][1]
            z = points[min_idx][2]

            center_point = y+0.3
            camera = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=np.array([0,0,0]))
            mesh = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=np.array([x,y,z]))
            centr = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=np.array([0,0,center_point]))
            open3d.visualization.draw_geometries([source, mesh, camera, centr])
            print(center_point)
        
    if stage == 2:
        # open3d.visualization.draw_geometries([original, mesh])
        
        # draw_registration_result(source, target, np.identity(4))
        
        short_source = findStart(source)

        # draw_registration_result(source, short_target, np.identity(4))

        err, T, q = get_Transformation(short_source, short_target, voxel_size, debug=False, viz=False)
        
        if err:
            print("Done")
            print(T)
        else:
            print("Failed")

def listener():
    
    
    rospy.init_node('playback', anonymous=True)
    print("listen")
    rospy.Subscriber("/camera/depth/color/points",PointCloud2, Pointcloud_Callback)
    
    # rospy.spin()   



if __name__ == '__main__':
    
    print("Getting pointcloud")

    listener()

    
    