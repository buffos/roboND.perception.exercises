#!/usr/bin/env python

# Import modules
from pcl_helper import *

# TODO: Define functions as required

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # 1: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)

    # 2: Voxel Grid Downsampling
    vox = cloud.make_voxel_grid_filter()
    LEAF_SIZE = 0.01 # selecting voxel size
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter() # apply filter to the point cloud.


    # 3: PassThrough Filter
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 1.1
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()


    # 4: RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)  # Set the model you wish to fit
    seg.set_method_type(pcl.SAC_RANSAC)
    # Max distance for a point to be considered fitting the model
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
    inliers, coefficients = seg.segment()

    # 5: Extract inliers and outliers
    cloud_table = cloud_filtered.extract(inliers, negative=False)
    cloud_objects = cloud_filtered.extract(inliers, negative=True)

    # 6: Euclidean Clustering
    #  a. construct a k-d tree.
    white_cloud = XYZRGB_to_XYZ(cloud_objects) # XYZRGB point cloud to XYZ
    tree = white_cloud.make_kdtree()

    #  b. create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    #  c. set tolerances for distance threshold
    #     as well as minimum and maximum cluster size (in points)
    #     NOTE: These are poor choices of clustering parameters
    ec.set_ClusterTolerance(0.015)
    ec.set_MinClusterSize(20)
    ec.set_MaxClusterSize(1500)
    #  e. search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    #  f. extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    # 7: Create Cluster-Mask Point Cloud to visualize each cluster separately
    #  a. assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))
    #  b. for each point in each cluster append it to the
    #     color_cluster_point_list as XYZRGB list
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    #  c. create a new cloud containing all clusters each with a unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # 8: Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # 9: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)


if __name__ == '__main__':

    # 1. ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # 2: Create Subscribers.
    #    we're subscribing our node to the "sensor_stick/point_cloud" topic.
    #    Anytime a message arrives, the message data (a point cloud!) will be
    #    passed to the pcl_callback() function for processing.
    pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2,
    pcl_callback, queue_size = 1)

    # 3: Create Publishers
    #    creating two new publishers to publish the point cloud data for the
    #    table and the objects on the table
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size = 1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size = 1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size = 1)

    # 4: Initialize color_list
    get_color_list.color_list = []

    # 5: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
