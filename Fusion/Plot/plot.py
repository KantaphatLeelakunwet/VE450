import open3d as o3d
import matplotlib.pyplot as plt
import time as t

import os

def fetch_new_pcd(i, path): 
    fetch = False
    color_raw = o3d.io.read_image(
    path + '\Camera\C' + str(i) + '.png')
    depth_raw = o3d.io.read_image(
    path + '\Lidar\L' + str(i) + '.png')

    pcd = o3d.geometry.PointCloud() # empty PCD

    if((not color_raw.is_empty()) and (not depth_raw.is_empty())): 
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_raw, depth_raw, depth_trunc= 50, depth_scale = 300, convert_rgb_to_intensity=False)

        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image,
            o3d.camera.PinholeCameraIntrinsic(
                o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault
            )
        )
        j = 1
        pcd.transform([[j, 0, 0, 0], [0, -j, 0, 0], [0, 0, -j, 0], [0, 0, 0, j]])
        fetch = True
    
    return fetch, pcd

def main(): 
    # path = os.getcwd()
    path = 'c:' + '\\' + 'Users\sofia\OneDrive\Documentos\SJTU\\10 . FALL SEM 2024\ME450 Capstone\code\VE450\DATA_TEST'

    # color_raw = o3d.io.read_image(
    #     path + '/2011_09_26_drive_0002_sync_image_0000000005_image_02.png')
    # # color_raw = o3d.io.read_image(
    # #     '/Users/kantaphat/Downloads/ECE4500J/VE450/Plot/2011_09_26_drive_0002_sync_image_0000000005_image_02.png')
    # print(color_raw)

    # depth_raw = o3d.io.read_image(
    #     # path + '/2011_09_26_drive_0002_sync_groundtruth_depth_0000000005_image_02.png')
    #     '/home/jetson/Documents/VE450/VE450/Fusion/Saved/best/results/2011_09_26_drive_0002_sync_velodyne_raw_0000000005_image_02.png')
    # print(depth_raw)

    i = 1
    vis = o3d.visualization.Visualizer()
    vis.create_window(width = 2000, height = 1200, left = 0, top = 0, visible = True)
    render_option = vis.get_render_option()
    render_option.mesh_show_back_face = True 
    render_option.mesh_show_wireframe = True
    render_option.point_show_normal = True
    geometry = o3d.geometry.PointCloud()
    # vis.add_geometry(geometry)

    fetched, pcd = fetch_new_pcd(0, path)
    geometry.colors = pcd.colors
    geometry.points = pcd.points
    if (fetched): vis.add_geometry(geometry)


    while(fetched): #reads through the entire video sequence
        #geometry = pcd
        geometry.colors = pcd.colors
        geometry.points = pcd.points
        
        vis.update_geometry(geometry)
        vis.poll_events()
        vis.update_renderer()
        fetched, pcd = fetch_new_pcd(i, path)
        # vis.reset_view_point(True)
        print(i)
        i = (i+1)%10
        t.sleep(0.05)
        # vis.clear_geometries()

      #width=2000,
#     height=1200,
#     point_show_normal=True,
#     mesh_show_wireframe=True,
#     mesh_show_back_face=True
    

main()
