import open3d as o3d
import matplotlib.pyplot as plt
import time as t

# import os

rgb_path = '/home/jetson/VE450/Fusion/Datasets/KITTI/depth_selection/val_selection_cropped/image/'
pc_path = '/home/jetson/VE450/Fusion/Saved/best/results/'
# og_depth = '/home/jetson/VE450/Fusion/Datasets/KITTI/depth_selection/val_selection_cropped/sofia_velodyne_raw/'

def fetch_new_pcd(i, path_rgb, path_pc): 
    fetch = False
    color_raw = o3d.io.read_image(
    path_rgb+ 'C' + str(i) + '.png')
    depth_raw = o3d.io.read_image(
    path_pc + 'F' + str(i) + '.png')

    pcd = o3d.geometry.PointCloud() # empty PCD

    if((not color_raw.is_empty()) and (not depth_raw.is_empty())): 
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_raw, depth_raw, depth_trunc= 50, convert_rgb_to_intensity=False)

        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image,
            o3d.camera.PinholeCameraIntrinsic(
                o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault
            )
        )
        j = 1
        pcd.transform([[j, 0, 0, 0], [0, -j, 0, 0], [0, 0, -j, 0], [0, 0, 0, j]])
        fetch = True
    
    return fetch, pcd, rgbd_image

def start_vis_window(option):
    i = 0
    first = True
    vis = o3d.visualization.Visualizer()
    vis.create_window(width = 2000, height = 1200, left = 0, top = 0, visible = True)
    render_option = vis.get_render_option()
    render_option.mesh_show_back_face = True 
    render_option.mesh_show_wireframe = True
    render_option.point_show_normal = True

    if(option == '2d'): 
        geometry = o3d.geometry.RGBDImage() # 2D RGBD display 
    else: 
        geometry = o3d.geometry.PointCloud() # Point Cloud 3D display 

    return geometry, vis

def open3D_vis(option): 
    # color_raw = o3d.io.read_image(
    #     path + '/2011_09_26_drive_0002_sync_image_0000000005_image_02.png')
    # # color_raw = o3d.io.read_image(
    # #     '/Users/kantaphat/Downloads/ECE4500J/VE450/Plot/2011_09_26_drive_0002_sync_image_0000000005_image_02.png')
    # print(color_raw)

    # depth_raw = o3d.io.read_image(
    #     # path + '/2011_09_26_drive_0002_sync_groundtruth_depth_0000000005_image_02.png')
    #     '/home/jetson/Documents/VE450/VE450/Fusion/Saved/best/results/2011_09_26_drive_0002_sync_velodyne_raw_0000000005_image_02.png')
    # print(depth_raw)

    # i = 0
    # first = True
    # vis = o3d.visualization.Visualizer()
    # vis.create_window(width = 2000, height = 1200, left = 0, top = 0, visible = True)
    # render_option = vis.get_render_option()
    # render_option.mesh_show_back_face = True 
    # render_option.mesh_show_wireframe = True
    # render_option.point_show_normal = True

    # if(option == '2d'): 
    #     geometry = o3d.geometry.RGBDImage() # 2D RGBD display 
    # else: 
    #     geometry = o3d.geometry.PointCloud() # Point Cloud 3D display 

    # fetched, pcd, rgbd_image = fetch_new_pcd(i, rgb_path, pc_path)
    geometry, vis = start_vis_window(option)
    i = 0
    fetched, pcd, rgbd_image = fetch_new_pcd(i, rgb_path, pc_path)
    first = True

    while(fetched): #reads through the entire video sequence
        #geometry = pcd
        if(option == '2d'): 
            geometry.color = rgbd_image.color
            geometry.depth = rgbd_image.depth
        else: # 3D as default 
            geometry.colors = pcd.colors
            geometry.points = pcd.points
        
        if(first): 
            vis.add_geometry(geometry)
            first = False
        else: 
            vis.update_geometry(geometry)
            vis.poll_events()
            vis.update_renderer()

        #update matplotlib plot 
        #im1.set_data(rgbd_image.color)
        #im2.set_data(rgbd_image.depth)
        #plt.draw()
        print(i)
        i = (i+1)%10
        fetched, pcd, rgbd_image = fetch_new_pcd(i, rgb_path, pc_path)
        # vis.reset_view_point(True)
        t.sleep(0.09)
        # vis.clear_geometries()
        

      #width=2000,
#     height=1200,
#     point_show_normal=True,
#     mesh_show_wireframe=True,
#     mesh_show_back_face=True

# function that initializes window

def render_image(i, vis, geometry, pc_path, rgb_path, first): 
    fetched, pcd, rgbd_image = fetch_new_pcd(i, rgb_path, pc_path)

    while(fetched): #reads through the entire video sequence
        #geometry = pcd
        geometry.colors = pcd.colors
        geometry.points = pcd.points
        
        if(first): 
            vis.add_geometry(geometry)
            first = False
        else: 
            vis.update_geometry(geometry)
            vis.poll_events()
            vis.update_renderer()


def matplotlib_vis(): 
    i = 0
    fetch, pcd, rgbd_image = fetch_new_pcd(i, rgb_path, pc_path)
    plt.subplot(1, 2, 1)
    plt.title('RGB image')
    im1 = plt.imshow(rgbd_image.color)
    plt.subplot(1, 2, 2)
    plt.title('Depth image')
    im2 = plt.imshow(rgbd_image.depth)
    plt.show()
    while(fetch): 
        i = i +1
        #update matplotlib plot 
        im1.set_data(rgbd_image.color)
        im2.set_data(rgbd_image.depth)
        plt.draw()
        print(i)
        i = (i+1)%10
        t.sleep(0.09)
        fetch, pcd, rgbd_image = fetch_new_pcd(i, rgb_path, pc_path)


def main():
    open3D_vis('3d')


main()
