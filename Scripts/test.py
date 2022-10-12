 #!/usr/bin/env python
from copy import deepcopy
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


first = True
count = 0
queue = []
kot = 0
buff = []  
stage = 1
j = 0
k = 0

def empty(a):
    pass

color_image = np.zeros((640,480,3))



def pointcloud_object(cloud_data, kot=35, viz=False, debug=False):
    kot = np.radians(kot)
    if debug:
        viz = False
        mesh = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=np.array([0.0, 0.0, 0.0]))

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
    visina_kamere = cc + 0.05
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


def imageProcess(depth_alligned_image):
    global count
    global color_image
    global queue
    global stage

    clipping_distance_in_meters = 0.9 #1 meter
    depth_scale = 0.001
    clipping_distance = clipping_distance_in_meters / depth_scale



    grey_color = 255
    depth_image_3d = np.dstack((depth_alligned_image,depth_alligned_image,depth_alligned_image)) #depth image is 1 channel, color is 3 channels
    bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

    depth_colormap = cv.applyColorMap(cv.convertScaleAbs(depth_alligned_image, alpha=0.03), cv.COLORMAP_JET)

    kopija1 = bg_removed.copy()
    kopija2 = bg_removed.copy()
    
    desna_stran = kopija1[:, 320:]  #
    leva_stran = kopija2[:, 125:320]    #

    meja = np.argwhere(leva_stran<255)[0][0]- 15
    
    #blur = cv.GaussianBlur(color_image,(5,5),1)
    blur = cv.bilateralFilter(color_image,9,150,75)
    gray = cv.cvtColor(blur, cv.COLOR_BGR2GRAY)

    th1 = cv.getTrackbarPos("Th1", "Parametri") #11
    th2 = cv.getTrackbarPos("Th2", "Parametri") #34

    Canny = cv.Canny(gray, th1, th2)
    
    kernel = np.ones((5,5))
    #kernel = np.asarray([[1,0,1],
                    #[0,1,0],
                    #[1,0,1]], dtype="uint8")


    dil = cv.dilate(Canny, kernel, iterations=1)
    #imgContour = color_image.copy()
    th = cv.getTrackbarPos("areaTh", "Parametri") 
    #getContours(dil, imgContour, th)

    
    #print("leva",np.argwhere(leva_stran<255)[0][0])
    #print("desna",np.argwhere(desna_stran<255)[0][0])
    
    found = False

    if np.argwhere(desna_stran<255)[0][0]  < (np.argwhere(leva_stran<255)[0][0]- 15):
        
        found = True
    else:
    
        found = False
    queue.append(found)
    #print("len",len(queue))
        
    if len(queue) == 25:
        aa = np.asarray(queue)
        if np.all(aa==True):
            print("Zaznan je objekt")
            print("\n")
            stage = 1
            queue = []
            
        else:
            print("No")
            print("\n")
            queue = np.delete(queue,0)
            queue = queue.tolist()

    slika = np.hstack([leva_stran,desna_stran])
    return slika




def rgbImage_Callback(data):
    global color_image
    bridge = cv_bridge.CvBridge()
    try:
        color_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except cv_bridge.CvBridgeError as e:
        print(e)

    #cv.imshow("Image window", cv_image_rgb)
    #cv.waitKey(3) 
    
    
def aligned_callback(data):
    global count
    global color_image
    global queue
    global stage
    global first

    bridge = cv_bridge.CvBridge()
    try:
        depth_alligned_image = bridge.imgmsg_to_cv2(data)

        depth_alligned_array = np.array(depth_alligned_image)*0.001
    except cv_bridge.CvBridgeError as e:
        print(e)
   # if stage == 0:

        #slika = imageProcess(depth_alligned_image)
        
        #cv.imshow("Depth window", slika)#[(np.argwhere(bg_removed<255)[0][0])])
        #cv.waitKey(5)

    #else:
        #if first:
            #cv.destroyAllWindows()
            #first = False
        #else:
            #cv.imshow("Depth window", depth_alligned_array)
            #cv.waitKey(5)

    

def depth_Callback(data):
    global color_image
    global j
    global k
    global stage
    bridge = cv_bridge.CvBridge()
    if stage == 2:
        if j%50 == 0:
            k += 1
            print(k)
        j += 1
        if k == 6:
            stage = 3
            k = 0
            j = 0

    if stage == 4:
        if j%50 == 0:
            k += 1
            print(k)
        j += 1
        if k == 6:
            stage = 5
    try:
        depth_image = bridge.imgmsg_to_cv2(data)
        deep_image = depth_image
        
    except cv_bridge.CvBridgeError as e:
        print(e)
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

def preprocess_point_cloud(pcd, voxel_size): #0.015 maybe
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


################# pridobitev transformacijske matrike, vrne T in R matriko ter rotacijo kot kvaternion ##########################

def get_Transformation(source, target, voxel_size):
    crit = True
    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    
    while crit:
        result_ransac = execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size)
        print(result_ransac.inlier_rmse),
        print(result_ransac.fitness)
        if (result_ransac.inlier_rmse < 0.016) and (result_ransac.fitness > 0.48) :
            print(result_ransac)
            """
            print("\n")
            R = result_ransac.transformation[:3,:3]
            T = np.array(result_ransac.transformation)[:3,3]
            x = T[0]
            y = T[1]
            q = tr.quaternion_from_matrix(result_ransac.transformation)
            print("T: ",T)
            #print("\n")
            #print("R: ",R)

            y_Eu = rotationMatrixToEulerAngles(R)[1]
            #print("\n")
            #print("quaternion: ", q)
            print("\n")
            print("zarotiraj za: ",np.rad2deg(y_Eu))
            if x > 0:
                smer_x = "levo"
            else:
                smer_x = "desno"
            print("premakni  za: ", np.abs(x)*100),
            #print("cm v ", smer_x)
            """
            draw_registration_result(source_down, target_down, result_ransac.transformation)
            crit = False
    return result_ransac.transformation




################### dodatno poravnavanje leg #####################

def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size, result_ransac):
    distance_threshold = voxel_size * 0.2
    
    result = open3d.registration.registration_icp(source, target, 0.02, result_ransac, open3d.registration.TransformationEstimationPointToPoint())
    return result


def makeSide(target):


    center_cloud = open3d.geometry.PointCloud()
    coord = np.array([[0,0,0]]) # Ustvari pointcloud iz tega
    center_cloud.points = open3d.utility.Vector3dVector(coord)


    points = np.asarray(target.points)
    dists = target.compute_point_cloud_distance(center_cloud)
    dists = np.asarray(dists)
    min_idx = np.argmin(dists)
    z = points[min_idx][2]
   # target.translate((0,0,-z))
    pcd = target.select_down_sample(np.where(points[:,2] < 0.6)[0])
    mesh = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=np.array([0.0, 0.0, 0.0]))
    open3d.visualization.draw_geometries([pcd, mesh])
    #open3d.io.write_point_cloud("target_cropped_front.pcd", pcd)

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

    
    


bum = 0

def Pointcloud_Callback(data):
    global buff
    global stage
    global bum
    global kot
    cloud_data = list(pc2.read_points(data, field_names = ("x", "y", "z","rgb"), skip_nans=True))
    
    source, original = pointcloud_object(cloud_data, viz=False, debug=False)
    target = open3d.io.read_point_cloud("target.pcd")

    target_front_end = open3d.io.read_point_cloud("target_cropped_front.pcd")

    cevi = open3d.io.read_point_cloud("target_cevi.pcd")



    if stage == 1:

        draw_registration_result(source, target_front_end, np.identity(4))
        opcija = 1
        if opcija == 1:
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
            z = z + 0.3
            trans_init = np.asarray([[0.0, 0.0, -1.0, z],
                                    [0.0, 1.0, 0.0, 0.0],
                                    [1.0, 0.0, 0.0, x],
                                    [0.0, 0.0, 0.0, 1.0]])

            source_temp = copy.deepcopy(source)
            source_temp_temp = copy.deepcopy(source)
            camera = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=np.array([0,0,0]))
            
            
            source.transform(trans_init)

            cut = points[min_idx][0] - 0.65
            source = source.select_down_sample(np.where(points[:,0] > cut)[0]) ############## odstrani steno....izbrisi ce stene ni ####################

            voxel_size=0.02
            source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
            target_down, target_fpfh = preprocess_point_cloud(target_front_end, voxel_size)
            source_down_temp = copy.deepcopy(source_down)

            notGood = True
            while notGood:
                result_ransac = execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size)
                TR = np.matmul(result_ransac.transformation, trans_init)
                R = TR[:3,:3]
                y_Eu = rotationMatrixToEulerAngles(R)[1]

                d = 90 - np.abs(np.rad2deg(y_Eu))
                print(result_ransac)
                print(d)
                if ((result_ransac.inlier_rmse < 0.015) or (result_ransac.fitness > 0.5)) and (np.abs(d) < 15):
                    notGood = False
                    
            print("\n")
    

            draw_registration_result(source_down, target_down, result_ransac.transformation)

            TR = np.matmul(result_ransac.transformation, trans_init)
            print(np.linalg.inv(TR))
            R = TR[:3,:3]
            y_Eu = rotationMatrixToEulerAngles(R)[1]
            print("\n")
            print("zarotiraj za: ",np.rad2deg(y_Eu))
            source_temp.transform(TR)
            camera.transform(np.linalg.inv(TR))
            target.paint_uniform_color([0, 0, 1])
            mesh = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=np.array([0,0,0]))
            #open3d.visualization.draw_geometries([source_temp, camera, target, source_temp_temp,mesh])
            open3d.visualization.draw_geometries([ camera, source_temp_temp,mesh])

            stage = 2
        
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
        
    if stage == 3:

        draw_registration_result(source, target, np.identity(4))
        
        
        voxel_size=0.02
        """
        source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
        target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)   

        result_ransac = execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size)
        print(result_ransac)
        print("\n")
        R = result_ransac.transformation[:3,:3]
        T = np.array(result_ransac.transformation)[:3,3]
        print("T: ",T)

        y_Eu = rotationMatrixToEulerAngles(R)[1]
        print("\n")
        print("zarotiraj za: ",np.rad2deg(y_Eu))
        print('......................')
        print("\n")

        
        draw_registration_result(source_down, target, result_ransac.transformation)
        """

        rezult = get_Transformation(source, target, voxel_size)
        print(np.linalg.inv(rezult))
        camera = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=np.array([0,0,0])) 
        target.paint_uniform_color([0, 0, 1])
        camera.transform(np.linalg.inv(rezult))
        mesh = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=np.array([0,0,0]))
        R = rezult[:3,:3]
        y_Eu = rotationMatrixToEulerAngles(R)[1]
        z_Eu = rotationMatrixToEulerAngles(R)[2]
        print("\n")
        #print("zarotiraj za: ",np.rad2deg(y_Eu))
        d = 180 - np.rad2deg(z_Eu)
        if np.abs(d) < 50:
            print("inv")
            trans = np.asarray([[-1.0, 0.0, 0.0, 0.0],
                                [0.0, -1.0, 0.0, 0.0],
                                [0.0, 0.0, 1.0, 0.0],
                                [0.0, 0.0, 0.0, 1.0]])
            camera.transform(np.linalg.inv(trans))
        open3d.visualization.draw_geometries([ camera, source,mesh])

        stage = 4

    if stage == 5:

        draw_registration_result(source, cevi, np.identity(4))
        
        
        voxel_size=0.02

        rezult = get_Transformation(source, cevi, voxel_size)
        print(np.linalg.inv(rezult))
        camera = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=np.array([0,0,0])) 
        cevi.paint_uniform_color([0, 0, 1])
        camera.transform(np.linalg.inv(rezult))
        mesh = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=np.array([0,0,0]))

        open3d.visualization.draw_geometries([ camera, source,mesh])   


def listener():
    

    rospy.init_node('playback', anonymous=True)
   
    rospy.Subscriber("/camera/color/image_raw",Image, rgbImage_Callback)

    rospy.Subscriber("/camera/aligned_depth_to_color/image_raw",Image, aligned_callback)
    

    rospy.Subscriber("/camera/depth/image_rect_raw",Image, depth_Callback)

    
    rospy.Subscriber("/camera/depth/color/points",PointCloud2, Pointcloud_Callback)

    rospy.spin()   


if __name__ == '__main__':
    print("start")
    

    listener()
  

    