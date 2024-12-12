# Test 
VE450 Capstone project 

to activate conda environment: ``` conda activate ve450```

to train: ```source Shell/train.sh /media/jetson/thinkplus``

## To run pretrained
python test.py --data_path ../Datasets/KITTI/
push test - sofia 

## move from thinkplus to jetson 
cd /VE450/Fusion/Datasets/KITTI/depth_selection/val_selection_cropped
mv /media/jetson/thinkplus/data_depth_selection/depth_selection/video_23/lidar/ velodyne_raw/

## move from jetson to thinbkplus 
cd /VE450/Fusion/Datasets/KITTI/depth_selection/val_selection_cropped
mv image/ /media/jetson/thinkplus/fused_images/image
