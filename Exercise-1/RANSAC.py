# Import PCL module
import pcl

# Load Point Cloud file
cloud = pcl.load_XYZRGB('tabletop.pcd')

# --------------------Voxel Grid filter------------------------

# 1. Create a VoxelGrid filter object for our input point cloud
vox = cloud.make_voxel_grid_filter()
# 2. Choose a voxel (also known as leaf) size
LEAF_SIZE = 0.01
# 3. Set the voxel (or leaf) size  
vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
# 4. Call the filter function to obtain the resultant downsampled point cloud
cloud_filtered = vox.filter()
filename = 'voxel_downsampled.pcd'
pcl.save(cloud_filtered, filename)


# -------------------- PassThrough filter ---------------------

# 1. Create a PassThrough filter object
passthrough = cloud_filtered.make_passthrough_filter()
# 2. Assign axis and range to the passthrough filter object.
filter_axis = 'z'
passthrough.set_filter_field_name (filter_axis)
axis_min = 0.6
axis_max = 1.1
passthrough.set_filter_limits (axis_min, axis_max)
# 3. Finally use the filter function to obtain the resultant point cloud.
cloud_filtered = passthrough.filter()
filename = 'pass_through_filtered.pcd'
pcl.save(cloud_filtered, filename)

# ------------------- RANSAC plane segmentation -----------------

# 1. Create the segmentation object
seg = cloud_filtered.make_segmenter()
# 2. Set the model you wish to fit 
seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)
# 3. Max distance for a point to be considered fitting the model
max_distance = 0.01
seg.set_distance_threshold(max_distance)
# 4. Call the segment function to obtain set of inlier indices and model coefficients
inliers, coefficients = seg.segment()


# ----------  Extract inliers (the table in our case) --------------

extracted_inliers = cloud_filtered.extract(inliers, negative=False)
filename = 'extracted_inliers.pcd'
pcl.save(extracted_inliers, filename)


# ----------  Extract outliers (stuff on the table) --------------
extracted_outliers = cloud_filtered.extract(inliers, negative=True)
filename = 'extracted_outliers.pcd'
pcl.save(extracted_outliers, filename)


# ---------- Statistical outliers filtering --------------

# 1. Much like the previous filters, we start by creating a filter object: 
outlier_filter = cloud_filtered.make_statistical_outlier_filter()
# 2. Save pcd for # Set the number of neighboring points to analyze for any given point
outlier_filter.set_mean_k(50)
# 3. Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
outlier_filter.set_std_dev_mul_thresh(1.0)
# 4. Finally call the filter function for magic
cloud_filtered = outlier_filter.filter()
# 5. To get the outliers (the noise) you  have to do before calling the filter
# outlier_filter.set_negative(True)
# and then
# cloud_filtered = outlier_filter.filter()



