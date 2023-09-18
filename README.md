# RLIO-KISS-ICP

This readme was created for the rlio adaptation of the kiss-icp, the original readme can be found [here](original_README.md).

For instructions on how to use the rlio-kiss-icp, please see the readme of the rlio repository.

As I only do hard resets on the kiss-icp map one needs to set the number of iterations in the code. This should be set to a rosparam...\
[KissICP.cpp](cpp/kiss_icp/pipeline/KissICP.cpp) line: 94

## changes

The original kiss-icp was adapted to work with the rlio framework. The main changes are:

- deskewing: instead of the constant velocity assumption, the kiss-icp is now subscribing to the high frequency pose publisher of the rlio framework. This allows to deskew the point clouds in the same way as [eth-lidar_undistorion](https://github.com/ethz-asl/lidar_undistortion) did it.

- initial guess: the new code subscribes to the initial guess publisher of the rlio framework. This allows a more accurate initial guess for the icp.

