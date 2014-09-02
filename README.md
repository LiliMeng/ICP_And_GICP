ICP_And_GICP
============

ICP and Generalized ICP algorithm based on PCL


  The difference between ICP and GICP is :

        Just change

pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;    in icp.cpp

to
pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> gicp;    in GICP.cpp


Here are the transformation results.

ICP

0.999742   0.0175653   0.0144246  -0.0213903
  -0.017617     0.99984  0.00345219   0.0117261
 -0.0143607 -0.00370541    0.999891  -0.0352564
          0           0           0           1

Generalized ICP

  0.999862   0.0159368  0.00458374 -0.00115125
 -0.0158798    0.999799  -0.0122125    0.016856
-0.00477745   0.0121381    0.999915 0.000585352
          0           0           0           1


Change the corresponding file name in CMakeLists.txt
