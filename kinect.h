#ifndef KINECT_H
#define KINECT_H

#include <iostream>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

// VTK
#include <vtkActor.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkThinPlateSplineTransform.h>

// OpenCV
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/eigen.hpp>

#include "dirent.h"  // for simple file reading


namespace kin {

#define FRAME_WIDTH 640
#define FRAME_HEIGHT 480
#define FRAME_SIZE FRAME_WIDTH*FRAME_HEIGHT
#define FRAME_BYTE_SIZE FRAME_SIZE*sizeof(unsigned short)

// Exception codes
#define FILE_NOT_OPEN 10
//#define FILE_NOT_SPLIT 20


	/******************************* setupCamera ************************************
	* Input:                                                                        *
	*    a string with the yaml intrinsics filename                                 *
	* Output:                                                                       *
	*    the depth camera matrix                                                    *
	*    the color camera matrix                                                    *
	*    the depth distorion matrix                                                 *
	*    the color distorion matrix                                                 *
	*                                                                               *
	* NOTE: Assumes that depth parameters are saved as "right_camera_matrix" and    *
	*       "right_distorion_parameters" and color parameters as                    *
	*       "left_camera_matrix" and "left_distorion_parameters" (default)          *
	*********************************************************************************/
	void setupCamera(std::string intrinsicsFileName, cv::Mat& depthCameraMat, cv::Mat& colorCameraMat, cv::Mat& depthDistortionMat = cv::Mat(), cv::Mat& colorDistortionMat = cv::Mat());


	/*********************** splitDepthDataFile OVERLOAD 1 **************************
	* Input:                                                                        *
	*    a string with the .data depth filename                                     *
	*    the desired number n of frames (first n frames will be extracted only)     *
	* Output:                                                                       *
	*    a vector containing the extracted depth frames                             *
	*                                                                               *
	* NOTE: The frames are flipped so that they appear correctly                    *
	*       This is used in the background class, where the first n frames are used *
    *       to create the background model                                          * 
	*********************************************************************************/
	bool splitDepthDataFile(std::string filename, int noOfFrames, std::vector<cv::Mat>& frames); 

	/*********************** splitDepthDataFile OVERLOAD 2 **************************
	* Input:                                                                        *
	*    a string with the .data depth filename                                     *
	*    a string with the .txt info depth filename                                 *
	* Output:                                                                       *
	*    a vector containing the extracted depth frames                             *
	*    a vector containing the corresponding serials of the frames                *
	*                                                                               *
	* NOTE: The frames are flipped so that they appear correctly                    *
	*       The serials are normalized, i.e. start from 0                           *
	*********************************************************************************/
	bool splitDepthDataFile(std::string filename, std::string infoFilename, std::vector<cv::Mat>& frames, std::vector<int>& serials);

	
	/********************* splitColorDataFile OVERLOAD 1 ****************************
	* Input:                                                                        *
	*    a string with the .data color filename                                     *
	* Output:                                                                       *
	*    a vector containing the extracted color frames                             *
	*                                                                               *
	* NOTE: The frames are flipped so that they appear correctly                    *
	*********************************************************************************/
	bool splitColorDataFile(std::string filename, std::vector<cv::Mat>& frames);
	
	/********************* splitColorDataFile OVERLOAD 2 ****************************
	* Input:                                                                        *
	*    a string with the .data color filename                                     *
	*    a string with the .txt info color filename                                 *
	* Output:                                                                       *
	*    a vector containing the extracted color frames                             *
	*    a vector containing the corresponding serials of the frames                *
	*                                                                               *
	* NOTE: The frames are flipped so that they appear correctly                    *
	*       The serials are normalized, i.e. start from 0                           *
	*********************************************************************************/
	bool splitColorDataFile(std::string filename, std::string infoFilename, std::vector<cv::Mat>& frames, std::vector<int>& serials);


	/******************************** frameToCloud **********************************
	* Input:                                                                        *
	*    a depth frame                                                              *
	*    the depth camera intrinsics matrix                                         *
	* Output:                                                                       *
	*    an RGB pseudo-colored point cloud                                          *
	*                                                                               *
	* NOTE: This function just pseudo-colors the cloud, use colorCloud to map the   *
	*       corresponding color frame                                               * 
	*********************************************************************************/
	bool frameToCloud(cv::Mat frame, cv::Mat cameraMatrix, pcl::PointCloud<pcl::PointXYZRGB>& pointCloud);


	/******************************** colorCloud ************************************
	* Input:                                                                        *
	*    an RGB point cloud (probably pseudo colored)                               *
	*    the corresponding color frame as a cv::Mat                                 *
	*    a string with the intrinsics yaml filename                                 *
	* Output:                                                                       *
	*    the same point cloud correctly colored                                     *
	*                                                                               *
	* NOTE: The coloring is in-place, meaning the original cloud gets altered       *
	*********************************************************************************/
	void colorCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, cv::Mat& colorFrame, std::string filename);


	/********************************** RGBtoXYZ ************************************
	* Input:                                                                        *
	*    an RGB color cloud                                                         *
	* Output:                                                                       *
	*    the corresponding point cloud in XYZ format, with the color stripped       *
	*********************************************************************************/
	void RGBtoXYZ(pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	

    /********************************* cloudToPLY ***********************************
	* Input:                                                                        *
	*    an RGB point cloud to be saved in PLY format                               *
	*    the desired filename with the extension (e.g. "mycloud.ply")               *
	*********************************************************************************/	
	bool cloudToPLY(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string filename);

	
	/*********************** constructTransformationMat *****************************
	* Input:                                                                        *
	*    a string with the filename of a stereo calibration file                    *
	* Output:                                                                       *
	*    a transformation matrix which combines the rotation and translation        *
	*                                                                               *
	* NOTE: Assumes that rotation is saved as "rotation_martix" and translation as  *
	*       "translation_vector" in the calib file (which is the default)           *
	*********************************************************************************/	
	Eigen::Matrix4f constructTransformationMat(std::string filename);


	/***************************** readPcdFilenames *********************************
	* Input:                                                                        *
	*    a string with the path of the PCDs folder                                  *
	* Output:                                                                       *
	*    a vector of strings which contains the filenames of all PCDs in the folder *
	*                                                                               *
	* NOTE: previously known as: getPCDs(same arguments)                            *
	*********************************************************************************/
	bool readPcdFilenames(std::string filename, std::vector<std::string>& pcdsVec);

	
	/*************************** transformPointCloudTPS *****************************
	* Input:                                                                        *
	*    an RGB point cloud to be transformed (pointer specifically)                *
	*    a VTK Thin Plate Splines transform (pointer)                               *
	* Output:                                                                       *
	*    a transformed RGB point cloud (pointer)                                    *
	*                                                                               *
	* NOTE: the input point cloud does not get altered                              *
	*********************************************************************************/
	void transformPointCloudTPS(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, vtkSmartPointer<vtkThinPlateSplineTransform>);


	/******************************* readFramesIR ***********************************
	* Input:                                                                        *
	*    a string with the path of the folder that contains infrared images         *
	* Output:                                                                       *
	*    a vector of those images as cv::Mat                                        *
	*                                                                               *
	* NOTE: Assumes images were taken with the MS drivers so it flips them          *
	*       The image filename must contain the string "ir" (default)               *
	*********************************************************************************/
	bool readFramesIR(std::string folder, std::vector<cv::Mat>& frames); 

	/******************************* readFramesDepth ********************************
	* Input:                                                                        *
	*    a string with the path of the folder that contains depth images            *
	* Output:                                                                       *
	*    a vector of those images as cv::Mat                                        *
	*                                                                               *
	* NOTE: Assumes images were taken with the MS drivers so it flips them          *
	*       The image filename must contain the string "depth" (default)            *
	*       In order to get the correct depth values, it right shifts pixels by 3   *
	*********************************************************************************/
	bool readFramesDepth(std::string folder, std::vector<cv::Mat>& frames); 

	//bool readFramesRGB(std::string folder, std::vector<cv::Mat>& frames); 

	/******************************* cloudPointsToMat OVERLOAD 1 ********************
	* Input:                                                                        *
	*    a XYZ point cloud                                                          *
	* Output:                                                                       *
	*    a homogeneous matrix 4 x no_of_points_in_cloud                             *
	*********************************************************************************/
	void XYZPointsToMat(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, cv::Mat& matrix);

	/******************************* cloudPointsToMat OVERLOAD 2 ********************
	* Input:                                                                        *
	*    a txt file with X Y Z coordinates                                          *
	* Output:                                                                       *
	*    a homogeneous matrix 4 x no_of_points_in_cloud                             *
	*********************************************************************************/
	void XYZPointsToMat(std::string filename, cv::Mat& matrix);

	/******************************* cvMatToEigen ***********************************
	* Input:                                                                        *
	*    a cv::Mat                                                                  *
	* Output:                                                                       *
	*    the corresponding Eigen::Matrix4f                                          *
    *                                                                               *
	* NOTE: This function is for converting the transformation matrix only, which   *
    *       means that the cv::Mat must be a 4 x 4 matrix                           *                                   *
	*********************************************************************************/
	void cvMatToEigen(const cv::Mat& cvMatrix, Eigen::Matrix4f& eigenMatrix);

	/****************************** saveCloudToTxt **********************************
	* Input:                                                                        *
	*    a XYZ point cloud pointer                                                  *
	* Output:                                                                       *
	*    the txt file with the name filename                                        *
    *                                                                               *
	* NOTE:                                                                         *
	*********************************************************************************/
	bool saveCloudToTxt(pcl::PointCloud<pcl::PointXYZ>::Ptr, std::string filename);


	bool get3DCornersFromIRPairs(std::vector<cv::Mat>& irFramesL, std::vector<cv::Mat>& depthFramesL, cv::Mat depthCameraMatL, std::vector<cv::Mat>& irFramesR, std::vector<cv::Mat>& depthFramesR, cv::Mat depthCameraMatR, cv::Size boardSize, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudL, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudR);
	//void get3DCornersFromRGB(...);

	/* UNDER CONSIDERATION */
	//void resampleCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
	//void resampleCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
	void fitSphere(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::ModelCoefficients::Ptr);
	bool keepSameIndices(std::vector<std::string>&, std::vector<std::string>&, std::vector<std::string>&, std::vector<std::string>&);
	//void thresholdCloud(pcl::PointCloud<pcl::PointXYZRGB>&);
	//bool cloudToPLY(pcl::PointCloud<pcl::PointXYZ>::Ptr, std::string);
	void MatToCloud(cv::Mat matrix, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud);
}

#endif