#include "kinect.h"

namespace kin {

	/******************************* setupCamera ************************************/
	void setupCamera(std::string intrinsicsFileName, cv::Mat& depthCameraMat, cv::Mat& colorCameraMat, cv::Mat& depthDistortionMat, cv::Mat& colorDistortionMat) {

		cv::FileStorage intrinsicsFile; 
		intrinsicsFile.open(intrinsicsFileName, cv::FileStorage::READ);

		if (!intrinsicsFile.isOpened()) {
			throw FILE_NOT_OPEN;
		}

		intrinsicsFile["right_camera_matrix"] >> depthCameraMat;
		intrinsicsFile["right_distortion_parameters"] >> depthDistortionMat;

		intrinsicsFile["left_camera_matrix"] >> colorCameraMat;
		intrinsicsFile["left_distortion_parameters"] >> colorDistortionMat;

		//intrinsicsFile["depth_intrinsics"] >> cameraMat;
		//intrinsicsFile["depth_dist_coeff"] >> distortionMat;
	}

	/*********************** splitDepthDataFile OVERLOAD 1 **************************/
	bool splitDepthDataFile(std::string fileName, int noOfFrames, std::vector<cv::Mat>& frames) {
		ifstream file(fileName, ios::in | ios::binary);
		if(!file) {
			return 0;
		}

		const int byteFrameSize = FRAME_SIZE*sizeof(unsigned short);	
		unsigned short frameData[FRAME_SIZE];

		for (int frameIndex=0; frameIndex<noOfFrames; ++frameIndex) {
			file.seekg(frameIndex*byteFrameSize);
			file.read((char*)&frameData, byteFrameSize);
			for (int i=0; i<FRAME_SIZE; i++) {
				frameData[i] = frameData[i]>>3;  // shift to get the actual depth value
			}

			cv::Mat frame(FRAME_SIZE, 1, CV_16U, &frameData);
			frame = frame.reshape(1, FRAME_HEIGHT);
			cv::flip(frame, frame, 1);

			cv::Mat frameCopy = frame.clone();
			frames.push_back(frameCopy);
		}
		return 1;
	}

	/*********************** splitDepthDataFile OVERLOAD 2 **************************/
	bool splitDepthDataFile(std::string fileName, std::string infoFileName, std::vector<cv::Mat>& frames, std::vector<int>& serials) {

		cout<<"\nSplitting depth data...\n";

		int frameIndex = 0; // Just for counting frames
		int firstSerial;
		bool firstSerialSet = false;

		ifstream file(fileName, ios::in | ios::binary);
		if(!file) {
			return 0;
		}
		ifstream infoFile(infoFileName);
		if(!infoFile) {
			return 0;
		} 

		const int byteFrameSize = FRAME_WIDTH*FRAME_HEIGHT*sizeof(unsigned short);	
		const int temp = FRAME_WIDTH*FRAME_HEIGHT;
		unsigned short frameData[FRAME_WIDTH*FRAME_HEIGHT];

		while (!file.eof()  && !infoFile.eof()) {

			int serial;
			float no, timestamp, angle;
			infoFile >> no;
			infoFile >> timestamp; 
			infoFile >> serial;
			infoFile >> angle;

			if(!firstSerialSet) {
				firstSerial = serial;
				firstSerialSet = true;
			}

			serials.push_back(serial-firstSerial);

			cout<<"IR Frame: "<<frameIndex<<endl;

			file.seekg(frameIndex*byteFrameSize);
			file.read((char*)&frameData, byteFrameSize);
			for (int i=0; i<FRAME_WIDTH*FRAME_HEIGHT; i++) {
				frameData[i] = frameData[i]>>3;  // shift to get the actual depth value
			}

			cv::Mat frame(FRAME_WIDTH*FRAME_HEIGHT, 1, CV_16U, &frameData);
			frame = frame.reshape(1, FRAME_HEIGHT);
			cv::flip(frame, frame, 1);

			cv::Mat frameCopy = frame.clone();
			frames.push_back(frameCopy);

			frameIndex++ ;
		}

		// Discard last frame just to be sure
		frames.pop_back();
		return 1;
	}

	/********************* splitColorDataFile OVERLOAD 1 ****************************/
	bool splitColorDataFile(std::string filename, std::vector<cv::Mat>& frames) {

		cout<<"\nSplitting color data...\n";
		// Just for counting frames
		int totalFrames = frames.size()-1;
		int frameIndex = 0;

		const int byteFrameSize = FRAME_WIDTH*FRAME_HEIGHT*sizeof(unsigned char)*4; // 4 because BGRA		
		unsigned char blueData[ FRAME_WIDTH*FRAME_HEIGHT], greenData[ FRAME_WIDTH*FRAME_HEIGHT], redData[ FRAME_WIDTH*FRAME_HEIGHT], alphaData[ FRAME_WIDTH*FRAME_HEIGHT];
		std::vector<cv::Mat> channels;
		cv::Mat colorFrame;
		cv::Mat blueMat;
		cv::Mat greenMat;
		cv::Mat redMat;

		ifstream file(filename, ios::in | ios::binary);
		if(!file) {
			return 0;
		}

		while (!file.eof()) {
			cout<<"RGB Frame: "<<frameIndex<<endl;

			file.seekg(frameIndex*byteFrameSize);
			channels.clear();

			for (int i=0; i<FRAME_WIDTH*FRAME_HEIGHT; i++) {
				blueData[i] = file.get();  // get one byte from stream (unformatted data)
				greenData[i] = file.get();
				redData[i] = file.get();
				alphaData[i] = file.get();
			}

			blueMat = cv::Mat(FRAME_WIDTH*FRAME_HEIGHT, 1, CV_8U, &blueData);
			blueMat = blueMat.reshape(1, FRAME_HEIGHT);
			cv::flip(blueMat, blueMat, 1);
			//cv::imshow("Blue", blueMat);  // debug

			greenMat = cv::Mat(FRAME_WIDTH*FRAME_HEIGHT, 1, CV_8U, &greenData);
			greenMat = greenMat.reshape(1, FRAME_HEIGHT);
			cv::flip(greenMat, greenMat, 1);
			//cv::imshow("Green", greenMat);  //debug

			redMat = cv::Mat(FRAME_WIDTH*FRAME_HEIGHT, 1, CV_8U, &redData);
			redMat = redMat.reshape(1, FRAME_HEIGHT);
			cv::flip(redMat, redMat, 1);
			//cv::imshow("Red", redMat);  // debug
			//cv::waitKey();

			// construct the 3-channel image
			channels.push_back(blueMat);
			channels.push_back(greenMat);
			channels.push_back(redMat);
			cv::merge(channels, colorFrame);

			cv::Mat frameCopy = colorFrame.clone();
			frames.push_back(frameCopy);

			frameIndex++ ;
		}
		//frames.pop_back();
		return 1;
	}

	/********************* splitColorDataFile OVERLOAD 2 ****************************/
	bool splitColorDataFile(std::string filename, std::string infoFilename, std::vector<cv::Mat>& frames, std::vector<int>& serials) {

		cout<<"\nSplitting color data...\n";
		// Just for counting frames
		int totalFrames = frames.size()-1;
		int frameIndex = 0;

		int firstSerial;
		bool firstSerialSet = false;

		const int byteFrameSize = FRAME_WIDTH*FRAME_HEIGHT*sizeof(unsigned char)*4; // 4 because BGRA		
		unsigned char blueData[ FRAME_WIDTH*FRAME_HEIGHT], greenData[ FRAME_WIDTH*FRAME_HEIGHT], redData[ FRAME_WIDTH*FRAME_HEIGHT], alphaData[ FRAME_WIDTH*FRAME_HEIGHT];
		std::vector<cv::Mat> channels;
		cv::Mat colorFrame;
		cv::Mat blueMat;
		cv::Mat greenMat;
		cv::Mat redMat;

		ifstream file(filename, ios::in | ios::binary);
		if(!file) {
			return 0;
		}
		ifstream infoFile(infoFilename);
		if(!infoFile) {
			return 0;
		} 

		while (!file.eof() && !infoFile.eof()) {

			int serial;
			float no, timestamp, angle;
			infoFile >> no;
			infoFile >> timestamp; 
			infoFile >> serial;
			infoFile >> angle;

			if(!firstSerialSet) {
				firstSerial = serial;
				firstSerialSet = true;
			}

			serials.push_back(serial-firstSerial);

			cout<<"RGB Frame: "<<frameIndex<<"/"<<totalFrames<<endl;

			file.seekg(frameIndex*byteFrameSize);
			channels.clear();

			for (int i=0; i<FRAME_WIDTH*FRAME_HEIGHT; i++) {
				blueData[i] = file.get();  // get one byte from stream (unformatted data)
				greenData[i] = file.get();
				redData[i] = file.get();
				alphaData[i] = file.get();
			}

			blueMat = cv::Mat(FRAME_WIDTH*FRAME_HEIGHT, 1, CV_8U, &blueData);
			blueMat = blueMat.reshape(1, FRAME_HEIGHT);
			cv::flip(blueMat, blueMat, 1);
			//cv::imshow("Blue", blueMat);  // debug

			greenMat = cv::Mat(FRAME_WIDTH*FRAME_HEIGHT, 1, CV_8U, &greenData);
			greenMat = greenMat.reshape(1, FRAME_HEIGHT);
			cv::flip(greenMat, greenMat, 1);
			//cv::imshow("Green", greenMat);  //debug

			redMat = cv::Mat(FRAME_WIDTH*FRAME_HEIGHT, 1, CV_8U, &redData);
			redMat = redMat.reshape(1, FRAME_HEIGHT);
			cv::flip(redMat, redMat, 1);
			//cv::imshow("Red", redMat);  // debug
			//cv::waitKey();

			// construct the 3-channel image
			channels.push_back(blueMat);
			channels.push_back(greenMat);
			channels.push_back(redMat);
			cv::merge(channels, colorFrame);

			cv::Mat frameCopy = colorFrame.clone();
			frames.push_back(frameCopy);

			frameIndex++ ;
		}
		//frames.pop_back();
		return 1;
	}

	/******************************** frameToCloud **********************************/
	bool frameToCloud(cv::Mat frame, cv::Mat cameraMatrix, pcl::PointCloud<pcl::PointXYZRGB>& pointCloud) {
		float x, y, z;
		bool notEmpty = 0;

		//pcl::PointCloud<pcl::PointXYZRGB> pointCloud;

		for (int i=0; i<FRAME_HEIGHT; i++)
		{	
			for (int j=0; j<FRAME_WIDTH; j++)
			{	
				z = frame.at<unsigned short>(i,j);
				if (z!=0)
				{
					x=(float)((j - cameraMatrix.at<double>(0,2)) * z / cameraMatrix.at<double>(0,0));
					y=(float)((i - cameraMatrix.at<double>(1,2)) * z / cameraMatrix.at<double>(1,1));  

					pcl::PointXYZRGB p;
					p.x = x;
					p.y = y;
					p.z = z;
					p.r = p.z/16;
					p.g = p.z/16;
					p.b = p.z/16;

					pointCloud.push_back (p); 

					notEmpty = 1;
				}
			}
		}
		return notEmpty;
	}

	/******************************** colorCloud ************************************/
	void colorCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, cv::Mat& colorFrame, std::string filename) {

		cv::FileStorage intrinsicsFile; 
		intrinsicsFile.open(filename, cv::FileStorage::READ);
		if (!intrinsicsFile.isOpened()) {
			throw FILE_NOT_OPEN;
		}

		cv::Mat rgbCameraMat;
		intrinsicsFile["left_camera_matrix"] >> rgbCameraMat;
		double fx_rgb = rgbCameraMat.at<double>(0,0);        
		double fy_rgb = rgbCameraMat.at<double>(1,1);        
		double cx_rgb = rgbCameraMat.at<double>(0,2);        
		double cy_rgb = rgbCameraMat.at<double>(1,2); 

		// Transform the depth point cloud
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudTransformed(new pcl::PointCloud<pcl::PointXYZRGB>);
		Eigen::Matrix4f transformationMat = constructTransformationMat(filename);
		Eigen::Matrix4f inverseTransformationMat = transformationMat.inverse();
		pcl::transformPointCloud(*cloud, *cloudTransformed, inverseTransformationMat);

		int m, n;
		uint8_t r,g,b;
		for (int i=0; i<cloudTransformed->size(); ++i)
		{
			m = ((cloudTransformed->at(i).x/cloudTransformed->at(i).z)*fx_rgb + cx_rgb) + 0.5;
			n = ((cloudTransformed->at(i).y/cloudTransformed->at(i).z)*fy_rgb + cy_rgb) + 0.5;

			if (m>0 && n>0 && m<640 && n<480)
			{			
				/*uint8_t*/ r = colorFrame.at<cv::Vec3b>(n,m)[2];
				/*uint8_t*/ g = colorFrame.at<cv::Vec3b>(n,m)[1];
				/*uint8_t*/ b = colorFrame.at<cv::Vec3b>(n,m)[0];

				uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
				/*cloudTransformed*/cloud->at(i).rgb = *reinterpret_cast<float*>(&rgb);
			}
		}
	}

	/********************************** RGBtoXYZ ************************************/
	void RGBtoXYZ(pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
		for (int i=0, s=colorCloud->size(); i<s; ++i) {
			cloud->push_back(pcl::PointXYZ(colorCloud->at(i).x, colorCloud->at(i).y, colorCloud->at(i).z));
		}
	}

	 /********************************* cloudToPLY ***********************************/
	// check if file hasn't been opened for some reason
	// check if file needs to exist
	bool cloudToPLY(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string filename) {
		ofstream file;
		file.open (filename);

		file << "ply\n";
		file << "format ascii 1.0\n";
		file << "element vertex "<<cloud->points.size()<<"\n";
		file << "property float x\n";
		file << "property float y\n";
		file << "property float z\n";
		file << "property uchar red\n";
		file << "property uchar green\n";
		file << "property uchar blue\n";
		file << "end_header\n";

		for (int i=0; i<cloud->points.size(); i++)
		{
			file << cloud->at(i).x << " " <<cloud->at(i).y<< " "<< cloud->at(i).z<<" "<<(int)/*cloud->at(i).r*/255<<" "<<(int)cloud->at(i).g<<" "<<(int)cloud->at(i).b<<"\n";
		}	

		file.close();
		return 1;
	}

	/*********************** constructTransformationMat *****************************/
	Eigen::Matrix4f constructTransformationMat(std::string filename) {	
		cv::FileStorage extrinsicsFile; 
		extrinsicsFile.open(filename, cv::FileStorage::READ);
		if (!extrinsicsFile.isOpened()) {
			throw FILE_NOT_OPEN;
		}
		cv::Mat rotationMat;
		extrinsicsFile["rotation_matrix"] >> rotationMat;
		cv::Mat translationMat;
		extrinsicsFile["translation_vector"] >> translationMat;

		Eigen::Matrix4f transformationMat;
		transformationMat << rotationMat.at<double>(0,0), rotationMat.at<double>(0,1), rotationMat.at<double>(0,2), translationMat.at<double>(0,0), 
			rotationMat.at<double>(1,0), rotationMat.at<double>(1,1), rotationMat.at<double>(1,2), translationMat.at<double>(1,0), 
			rotationMat.at<double>(2,0), rotationMat.at<double>(2,1), rotationMat.at<double>(2,2), translationMat.at<double>(2,0), 
			0             ,              0             ,              0             ,                 1             ;
		return transformationMat;
	}

	/***************************** readPcdFilenames *********************************/
	bool readPcdFilenames(std::string filename, std::vector<std::string>& pcdsVec) {
		DIR *dir;
		struct dirent *ent;
		dir = opendir (filename.c_str());
		if (dir != NULL) 
		{
			while ((ent = readdir (dir)) != NULL) 
			{
				std::string fn = ent->d_name;
				if(fn.substr(fn.find_last_of(".") + 1) == "pcd") 
				{
					string pcdName = filename;
					pcdName += "/";
					pcdName += fn;
					pcdsVec.push_back(pcdName);
				} 
			}
			closedir (dir);
			return 1;
		} 
		else 
		{
			cerr<<endl<<"Could not open image directory"<<endl;
			cout<<endl<<"Press Enter to exit..."<<endl;
			cin.get();
			return 0;
		}
	}

	/*************************** transformPointCloudTPS *****************************/
	void transformPointCloudTPS(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut, vtkSmartPointer<vtkThinPlateSplineTransform> transform) {

		vtkSmartPointer< vtkPoints > pA = vtkSmartPointer< vtkPoints >::New();
		pA->SetNumberOfPoints(cloudIn->size());
		for (int i = 0; i<cloudIn->size(); ++i) {
			pA->SetPoint(i, cloudIn->at(i).x, cloudIn->at(i).y, cloudIn->at(i).z);
		}		
		vtkSmartPointer<vtkPolyData> pointsPolydataA = vtkSmartPointer<vtkPolyData>::New();
		pointsPolydataA->SetPoints(pA);
		vtkSmartPointer<vtkVertexGlyphFilter> vertexFilterA = vtkSmartPointer<vtkVertexGlyphFilter>::New();
		vertexFilterA->SetInputConnection(pointsPolydataA->GetProducerPort());
		vertexFilterA->Update(); 
		vtkSmartPointer<vtkPolyData> polydataA = vtkSmartPointer<vtkPolyData>::New();
		polydataA->ShallowCopy(vertexFilterA->GetOutput());
		// Apply transformation found
		vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
		transformFilter->SetInput(polydataA);
		transformFilter->SetTransform(transform);
		transformFilter->Update();
		// Create a mapper and actor
		vtkSmartPointer<vtkPolyDataMapper> transformedMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		transformedMapper->SetInputConnection(transformFilter->GetOutputPort());
		vtkSmartPointer<vtkActor> transformedActor = vtkSmartPointer<vtkActor>::New();
		transformedActor->SetMapper(transformedMapper);
		// Convert transformed polydata to pcl cloud
		//pcl::PointCloud<pcl::PointXYZ>::Ptr leftCloudTPS(new pcl::PointCloud<pcl::PointXYZ>);
		cloudOut->clear();
		for(vtkIdType i = 0; i < polydataA->GetNumberOfPoints(); i++) {
			//GetMapper()->GetInput()->GetPoint(vtkIdType, double[3]); 
			double p[3];
			transformedActor->GetMapper()->GetInput()->GetPoint(i, p);
			pcl::PointXYZRGB point;//(p[0], p[1], p[2]);
			point.x = p[0];
			point.y = p[1];
			point.z = p[2];
			point.r = cloudIn->at(i).r;
			point.g = cloudIn->at(i).g;
			point.b = cloudIn->at(i).b;
			cloudOut->push_back(point);
			//std::cout << "Point " << i << " : (" << p[0] << " " << p[1] << " " << p[2] << ")" << std::endl;
		}
	}

	/******************************* readFramesIR ***********************************/
	bool readFramesIR(std::string folder, std::vector<cv::Mat>& frames) {
		DIR* dir = opendir ((char*)folder.c_str());
		struct dirent *ent;

		if (dir != NULL) {
			while ((ent = readdir (dir)) != NULL) {
				std::string fn = ent->d_name;
				if(fn.substr(fn.find_last_of(".") + 1) == "png") {

					std::string filename = folder + "/" + fn;

					if (std::string::npos != fn.find("ir")) {
						cv::Mat frame = cv::imread(filename, 0);
						cv::flip(frame, frame, 1);
						frames.push_back(frame);
					}
				} 
			}
			closedir (dir);
			return 1;
		} 
		else {
			std::cout<<"\nCould not read IR frames. Check paths and the rest.\n";
			return 0;
		}
	}

	/******************************* readFramesDepth ********************************/
	bool readFramesDepth(std::string folder, std::vector<cv::Mat>& frames) {
		DIR* dir = opendir ((char*)folder.c_str());
		struct dirent *ent;

		if (dir != NULL) {
			while ((ent = readdir (dir)) != NULL) {
				std::string fn = ent->d_name;
				if(fn.substr(fn.find_last_of(".") + 1) == "png") {

					std::string filename = folder + "/" + fn;

					if (std::string::npos != fn.find("depth")) {
						cv::Mat frame = cv::imread(filename, -1);
						cv::flip(frame, frame, 1);
						// Shift bits
						for (int i=0; i<frame.rows; ++i) {
							for (int j=0; j<frame.cols; ++j) {
								int newPixelValue = frame.at<ushort>(i,j) >> 3;
								if (newPixelValue == 4095 || newPixelValue == 8191) {
									newPixelValue = 0;
								}
								frame.at<ushort>(i,j) = newPixelValue;
							}
						}
						frames.push_back(frame);
					}
				} 
			}
			closedir (dir);
			return 1;
		} 
		else {
			std::cout<<"\nCould not read depth frames. Check paths and the rest.\n";
			return 0;
		}
	}

	/******************************* cloudPointsToMat *******************************/
	void XYZPointsToMat(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, cv::Mat& matrix) {
		int noOfPoints = cloud->points.size();
		matrix = cv::Mat(4, noOfPoints, CV_64F);

		for (int i=0; i<noOfPoints; ++i) {
			matrix.at<double>(0,i) = cloud->at(i).x; 	
			matrix.at<double>(1,i) = cloud->at(i).y; 	
			matrix.at<double>(2,i) = cloud->at(i).z; 	
			matrix.at<double>(3,i) = 1; 	
		}

	}

	/******************************* cloudPointsToMat *******************************/
	void XYZPointsToMat(std::string filename, cv::Mat& matrix) {
		std::ifstream file(filename);
		if (!file.is_open()) {
			std::cout<<"Problem rading cloud from file";
			return;
		}

		// count lines
		std::string line;
		int noOfPoints = 0;		
		while( std::getline(file,line) ){
			noOfPoints++;
		}
		file.clear();
		file.seekg(0, ios::beg);
		
		// construct the matrix
		matrix = cv::Mat(4, noOfPoints, CV_64F);
		for (int i=0; i<noOfPoints; ++i) {
			file >> matrix.at<double>(0,i); 	
			file >> matrix.at<double>(1,i);
			file >> matrix.at<double>(2,i);	
			matrix.at<double>(3,i) = 1; 	
		}
	}

	/******************************* cvMatToEigen ***********************************/
	void cvMatToEigen(const cv::Mat& cvMatrix, Eigen::Matrix4f& eigenMatrix) {
		eigenMatrix << cvMatrix.at<double>(0,0), cvMatrix.at<double>(0,1), cvMatrix.at<double>(0,2), cvMatrix.at<double>(0,3), 
			           cvMatrix.at<double>(1,0), cvMatrix.at<double>(1,1), cvMatrix.at<double>(1,2), cvMatrix.at<double>(1,3),
					   cvMatrix.at<double>(2,0), cvMatrix.at<double>(2,1), cvMatrix.at<double>(2,2), cvMatrix.at<double>(2,3),
					   cvMatrix.at<double>(3,0), cvMatrix.at<double>(3,1), cvMatrix.at<double>(3,2), cvMatrix.at<double>(3,3);
	}

	/****************************** saveCloudToTxt **********************************/
	bool saveCloudToTxt(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string filename) {
		ofstream file;
		file.open (filename, ios::out | ios::trunc); 
		if (file.is_open()) {
			pcl::PointCloud<pcl::PointXYZ>::iterator it;
			for (it = cloud->begin(); it<cloud->end(); it++) {
				file << it->x << " " << it->y<< " " << it->z << "\n";
			}
		}
		else {
			std::cout<<"Problem writing cloud to file";
			return 0;
		}
		return 1;
	}


	bool get3DCornersFromIRPairs(std::vector<cv::Mat>& irFramesL, std::vector<cv::Mat>& depthFramesL, cv::Mat depthCameraMatL, std::vector<cv::Mat>& irFramesR, std::vector<cv::Mat>& depthFramesR, cv::Mat depthCameraMatR, cv::Size boardSize, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudL, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudR) {

		std::cout<<"\nConverting 2D image corner points to 3D...\n";

		int noOfFrames = irFramesL.size();

		for (int i=0; i<noOfFrames; ++i) {

			std::cout<<"Checking pair "<<i<<"...";

			cv::Mat currentLeftFrame = irFramesL[i];
			cv::Mat currentRightFrame = irFramesR[i];

			//// debug
			//cv::imshow("CHESSBOARD CORNERS LEFT", currentLeftFrame);
			//cv::imshow("CHESSBOARD CORNERS RIGHT", currentRightFrame);
			//cv::waitKey(1000);

			std::vector<cv::Point2f> leftCorners;  
			std::vector<cv::Point2f> rightCorners;
			bool foundLeft = cv::findChessboardCorners(currentLeftFrame, boardSize, leftCorners);
			bool foundRight = cv::findChessboardCorners(currentRightFrame, boardSize, rightCorners);

			if (foundLeft && foundRight) {

				std::cout<<"Corners found!\n";

				// Subpixel accuracy
				cv::cornerSubPix(currentLeftFrame, leftCorners, cv::Size(5,5), cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::MAX_ITER +	cv::TermCriteria::EPS, 30, 0.1));
				cv::cornerSubPix(currentRightFrame, rightCorners, cv::Size(5,5), cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::MAX_ITER +	cv::TermCriteria::EPS, 30, 0.1));

				// Draw corners 
				cv::Mat colorImageLeft = cv::Mat(currentLeftFrame.rows, currentLeftFrame.cols, CV_8UC3);
				cv::cvtColor(currentLeftFrame, colorImageLeft, CV_GRAY2BGR);
				cv::drawChessboardCorners(colorImageLeft, boardSize, leftCorners, foundLeft);	 
				cv::imshow("CHESSBOARD CORNERS LEFT", colorImageLeft);

				cv::Mat colorImageRight = cv::Mat(currentRightFrame.rows, currentRightFrame.cols, CV_8UC3);
				cv::cvtColor(currentRightFrame, colorImageRight, CV_GRAY2BGR);
				cv::drawChessboardCorners(colorImageRight, boardSize, rightCorners, foundRight);	
				cv::imshow("CHESSBOARD CORNERS RIGHT", colorImageRight);

				cv::waitKey(1000);
				
				for (int w=0; w<boardSize.area(); w++) {

					ushort zL = depthFramesL[i].at<ushort>((int)leftCorners[ w ].y, (int)leftCorners[ w ].x); 
					ushort zR = depthFramesR[i].at<ushort>((int)rightCorners[ w ].y, (int)rightCorners[ w ].x);
					//cout<<zL<<endl;
					if ( zL!=0 && zR!=0 ) {

						float xL = (float)((leftCorners[ w ].x - depthCameraMatL.at<double>(0,2)) * zL / depthCameraMatL.at<double>(0,0));
						float yL = (float)((leftCorners[ w ].y - depthCameraMatL.at<double>(1,2)) * zL / depthCameraMatL.at<double>(1,1));  
						cloudL->push_back(pcl::PointXYZ(xL, yL, zL));

						float xR = (float)((rightCorners[ w ].x - depthCameraMatR.at<double>(0,2)) * zR / depthCameraMatR.at<double>(0,0));
						float yR = (float)((rightCorners[ w ].y - depthCameraMatR.at<double>(1,2)) * zR / depthCameraMatR.at<double>(1,1));  
						cloudR->push_back(pcl::PointXYZ(xR, yR, zR));
					}
				}			
			}
			else {
				std::cout<<"Corners NOT found!\n";
			}
		}
		if (cloudL->points.size() == 0 || cloudR->points.size() == 0) {
			std::cout<<"No corners have been found...\n";
			return 0;
		}
		return 1;
	}

	
	/* UNDER CONSIDERATION */

	void fitSphere(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud, pcl::ModelCoefficients::Ptr sphereCoefficients) {
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

		pcl::SACSegmentation<pcl::PointXYZRGB> seg;
		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_SPHERE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setMaxIterations (1000000);
		seg.setDistanceThreshold (5.0);
		//seg.setDistanceThreshold (20.0);  // for box and ball data
		//seg.setRadiusLimits (0.4, 0.6);
		seg.setInputCloud (inputCloud);
		seg.segment (*inliers, *sphereCoefficients);

		pcl::ExtractIndices<pcl::PointXYZRGB> extract;
		extract.setInputCloud (inputCloud);
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter (*outputCloud);
	}

	bool keepSameIndices(std::vector<std::string>& cloudsA, std::vector<std::string>& cloudsB, std::vector<std::string>& cloudsC, std::vector<std::string>& cloudsD) {

		std::vector<std::string> cloudsAtemp, cloudsBtemp, cloudsCtemp, cloudsDtemp;
		std::vector<int> cloudsAind, cloudsBind, cloudsCind, cloudsDind;

		// Find the common path for each filename (path except the numbers and ".pcd")
		std::string cloudsApath = cloudsA[0].substr(0,cloudsA[0].find_last_of("_"));
		std::string cloudsBpath = cloudsB[0].substr(0,cloudsB[0].find_last_of("_"));
		std::string cloudsCpath = cloudsC[0].substr(0,cloudsC[0].find_last_of("_"));
		std::string cloudsDpath = cloudsD[0].substr(0,cloudsD[0].find_last_of("_"));

		// Put the numbers in corresponding vectors and sort them 
		for (int i=0, s=cloudsA.size(); i<s; ++i) {
			int length = cloudsA[i].size() - cloudsApath.size() - 5;
			int index = std::stoi(cloudsA[i].substr(cloudsA[i].find_last_of("_") +1, length));
			cloudsAind.push_back(index);
		}
		std::sort(cloudsAind.begin(), cloudsAind.end());
		for (int i=0, s=cloudsB.size(); i<s; ++i) {
			int length = cloudsB[i].size() - cloudsBpath.size() - 5;
			int index = std::stoi(cloudsB[i].substr(cloudsB[i].find_last_of("_") +1, length));
			cloudsBind.push_back(index);
		}
		std::sort(cloudsBind.begin(), cloudsBind.end());
		for (int i=0, s=cloudsC.size(); i<s; ++i) {
			int length = cloudsC[i].size() - cloudsCpath.size() - 5;
			int index = std::stoi(cloudsC[i].substr(cloudsC[i].find_last_of("_") +1, length));
			cloudsCind.push_back(index);
		}
		std::sort(cloudsCind.begin(), cloudsCind.end());
		for (int i=0, s=cloudsD.size(); i<s; ++i) {
			int length = cloudsD[i].size() - cloudsDpath.size() - 5;
			int index = std::stoi(cloudsD[i].substr(cloudsD[i].find_last_of("_") +1, length));
			cloudsDind.push_back(index);
		}
		std::sort(cloudsDind.begin(), cloudsDind.end());

		// Find the common vector
		// common of A-B
		std::vector<int> ABvec(cloudsAind.size());                     
		std::vector<int>::iterator ABit;
		ABit=std::set_intersection (cloudsAind.begin(), cloudsAind.end(), cloudsBind.begin(), cloudsBind.end(), ABvec.begin());
		ABvec.resize(ABit-ABvec.begin());
		// common of A-B-C
		std::vector<int> ABCvec(cloudsAind.size());                     
		std::vector<int>::iterator ABCit;
		ABCit=std::set_intersection (ABvec.begin(), ABvec.end(), cloudsCind.begin(), cloudsCind.end(), ABCvec.begin());
		ABCvec.resize(ABCit-ABCvec.begin());
		// common of A-B-C-D
		std::vector<int> ABCDvec(cloudsAind.size());                     
		std::vector<int>::iterator ABCDit;
		ABCDit=std::set_intersection (ABCvec.begin(), ABCvec.end(), cloudsDind.begin(), cloudsDind.end(), ABCDvec.begin());
		ABCDvec.resize(ABCDit-ABCDvec.begin());

		// Construct the full names with the common indices
		for (int i=0, s=ABCDvec.size(); i<s; ++i) {
			// A
			std::string tempA = cloudsApath + "_" + std::to_string(static_cast<long long>(ABCDvec[i])) + ".pcd";
			cloudsAtemp.push_back(tempA);
			// B
			std::string tempB = cloudsBpath + "_" + std::to_string(static_cast<long long>(ABCDvec[i])) + ".pcd";
			cloudsBtemp.push_back(tempB);
			// C
			std::string tempC = cloudsCpath + "_" + std::to_string(static_cast<long long>(ABCDvec[i])) + ".pcd";
			cloudsCtemp.push_back(tempC);
			// D
			std::string tempD = cloudsDpath + "_" + std::to_string(static_cast<long long>(ABCDvec[i])) + ".pcd";
			cloudsDtemp.push_back(tempD);
		}

		cloudsA.clear();
		cloudsA = cloudsAtemp;
		cloudsB.clear();
		cloudsB = cloudsBtemp;
		cloudsC.clear();
		cloudsC = cloudsCtemp;
		cloudsD.clear();
		cloudsD = cloudsDtemp;

		//for (int i=0; i<ABCDvec.size(); ++i) {
		//	cout<<ABCDvec[i]<<endl;
		//}


		//std::cout << "The intersection has " << (v.size()) << " elements:\n";
		//for (it=v.begin(); it!=v.end(); ++it)
		//  std::cout << ' ' << *it;
		//std::cout << '\n';

		//cout<<cloudsA[i].substr(cloudsA[i].find_last_of("_") +1,cloudsA[i].find_last_of(".")-1)<<endl;


		return 1;
	}



	//bool cloudToPLY(pcl::PointCloud<pcl::PointXYZ>::Ptr, std::string) {
	//	ofstream myfileLeft;
	//	myfileLeft.open ("head_leftkinect4_cloud_registered_color.ply");
	//	myfileLeft << "ply\n";
	//	myfileLeft << "format ascii 1.0\n";
	//	myfileLeft << "element vertex "<<tempCloud->points.size()<<"\n";
	//	myfileLeft << "property float x\n";
	//	myfileLeft << "property float y\n";
	//	myfileLeft << "property float z\n";
	//	myfileLeft << "property uchar red\n";
	//	myfileLeft << "property uchar green\n";
	//	myfileLeft << "property uchar blue\n";
	//	myfileLeft << "end_header\n";
	//	for (int i=0; i<tempCloud->points.size(); i++)
	//	{
	//		myfileLeft << tempCloud->at(i).x << " " <<tempCloud->at(i).y<< " "<< tempCloud->at(i).z<<" "<<(int)tempCloud->at(i).r<<" "<<(int)tempCloud->at(i).g<<" "<<(int)tempCloud->at(i).b<<"\n";
	//	}	
	//	myfileLeft.close();
	//}



	//void resampleCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud) {
		//pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
		//pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> mls; // Init object (second point type is for the normals, even if unused)			
		//mls.setInputCloud (inputCloud);
		//mls.setPolynomialFit (true);
		//mls.setSearchMethod (tree);
		//mls.setSearchRadius (50.0);	//100.0		
		//mls.reconstruct(*outputCloud);
	//}

	//void resampleCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud) {
		//pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
		//pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> mls; // Init object (second point type is for the normals, even if unused)			
		//mls.setInputCloud (inputCloud);
		//mls.setPolynomialFit (true);
		//mls.setSearchMethod (tree);
		//mls.setSearchRadius (50.0); // try 1000.0 for very smooth			
		//mls.reconstruct(*inputCloud);
	//}


	//void thresholdCloud(pcl::PointCloud<pcl::PointXYZRGB>& cloud) {
	//}

	/******************************** frameToCloud **********************************/
	void MatToCloud(cv::Mat matrix, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud) {
		for (int i=0; i<matrix.cols; i++){	
			pcl::PointXYZ p;
			p.x = matrix.at<double>(0,i); 	
			p.y = matrix.at<double>(1,i);
			p.z = matrix.at<double>(2,i);	

			pointCloud->push_back (p); 
		}
	}

	

}