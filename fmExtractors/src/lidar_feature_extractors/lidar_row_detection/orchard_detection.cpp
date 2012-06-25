/****************************************************************************
 # AES25 node
 # Copyright (c) 2011 Søren Hundevadt Nielsen <shn@kbm.sdu.dk>
 #
 # Permission is hereby granted, free of charge, to any person obtaining a copy
 # of this software and associated documentation files (the "Software"), to deal
 # in the Software without restriction, including without limitation the rights
 # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 # copies of the Software, and to permit persons to whom the Software is
 # furnished to do so, subject to the following conditions:
 #
 # The above copyright notice and this permission notice shall be included in
 # all copies or substantial portions of the Software.
 #
 # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 # THE SOFTWARE.
 #
 *****************************************************************************
 # File: AES25_node.cpp
 # Purpose: AES25 actuator driver.
 # Project: Field Robot - Vehicle Interface Computer
 # Author: Søren Hundevadt Nielsen <shn@kbm.sdu.dk>
 # Created: Jun 5, 2011 Søren Hundevadt Nielsen, Source written
 ****************************************************************************/

#include "orchard_detection.h"

OrchardDetector::OrchardDetector() {
	rawData_ = cvCreateImage(cvSize(600, 600), 8, 3);
	workingImg_ = cvCreateImage(cvSize(600, 600), 8, 1);

	inrow_cnt = 0;

	right_angle_ = CV_PI / 2;
	left_angle_ = CV_PI / 2;

	for (int i = 0; i < 100; i++) {
		for(int j = 0 ; j < 4; j++)
		rolling_mean_[i][j] = 0;
	}
}

void OrchardDetector::clearRawImage() {
	cvZero(rawData_);
	storage_ = cvCreateMemStorage(0);
	lines_ = 0;

}

void OrchardDetector::processLaserScan(

const sensor_msgs::LaserScan::ConstPtr& laser_scan) {
	float rthetamean = 0, rrhomean = 0, lthetamean = 0, lrhomean = 0,
			theta = 0, rho = 0;
	double x0 = 0, y0 = 0, a, b;
	int lmc = 0, rmc = 0;

	static int count = 0;

	clearRawImage();

	sensor_msgs::PointCloud cloud;
	projector_.projectLaser(*laser_scan, cloud);

	int size = cloud.points.size();

	for (int i = 0; i < size; i++) {
		if (abs(cloud.points[i].y) < 1.5 && abs(cloud.points[i].y) > 0.5 &&cloud.points[i].y < 0 && cloud.points[i].x > -1 && cloud.points[i].x < 3) {
			point1.x = ((int)(cloud.points[i].x * 50) + 300);
			point1.y = ((int)(cloud.points[i].y * 50) + 300);
			point2.x = point1.x - 4;
			point2.y = point1.y;
			cvLine(rawData_, point1, point2, CV_RGB(255,255,255), 2, CV_AA, 0);
		}
	}

	cvCvtColor(rawData_, workingImg_, CV_BGR2GRAY);
	//cvThreshold(workingImg_,workingImg_,)

	cvThreshold(workingImg_,workingImg_, 100, 255, CV_THRESH_BINARY);

	CvMemStorage* stor = cvCreateMemStorage(0);
	CvSeq* cont = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq),
			sizeof(CvPoint), stor);

	// find external contours
	cvFindContours(workingImg_, stor, &cont, sizeof(CvContour),
			CV_RETR_EXTERNAL, 2, cvPoint(0, 0));

	for (; cont; cont = cont->h_next) {
		// size of pointArray and polygon
		int point_cnt = cont->total;

		// no small contours
		if (point_cnt > 20) {

			CvPoint* PointArray = (CvPoint*) malloc(point_cnt * sizeof(CvPoint));

			// Get contour point set.
			cvCvtSeqToArray(cont, PointArray, CV_WHOLE_SEQ);

			for (int i = 0; i <= point_cnt; i++) {
				// Show the Pixelcoordinates
				// have some fun with the color
				cvLine(rawData_, PointArray[i % point_cnt], PointArray[(i + 1) % point_cnt], cvScalar(0, 0, 0), 10);
				cvLine(workingImg_, PointArray[i % point_cnt], PointArray[(i + 1) % point_cnt], cvScalar(0, 0, 0), 50);
			}

			continue;
		}

		// Allocate memory for contour point set.
		CvPoint* PointArray = (CvPoint*) malloc(point_cnt * sizeof(CvPoint));

		// Get contour point set.
		cvCvtSeqToArray(cont, PointArray, CV_WHOLE_SEQ);

		for (int i = 0; i <= point_cnt; i++) {
			// Show the Pixelcoordinates
			//cout << PointArray[i].x << " " << PointArray[i].y << endl;
			// have some fun with the color
			int h_value = int(i * 3.0 / point_cnt * i) % 100;
			cvLine(rawData_, PointArray[i % point_cnt], PointArray[(i + 1) % point_cnt], cvScalar(0, 255, 0), 4);
		}

	}

	//cvDilate(workingImg_,workingImg_, 0, 3);
	//cvErode(workingImg_,workingImg_, 0, 3);

	//cvShowImage("Wporking", workingImg_);
	cvWaitKey(10);

	lines_ = cvHoughLines2(workingImg_, storage_, CV_HOUGH_STANDARD, 1, CV_PI / 180, 15, 0, 0);

	//cvHoughLines2(edgesImage, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/360, 30, 10, MAXIMUM_GAP);


	for (int i = 0; i < MIN(lines_->total,15); i++) {


		float* line = (float*) cvGetSeqElem(lines_, i);
		rho = line[0];
		theta = line[1];

		a = cos(theta);
		b = sin(theta);
		x0 = a * rho;
		y0 = b * rho;
		point1.x = cvRound(x0 + 600 * (-b));
		point1.y = cvRound(y0 + 600 * (a));
		point2.x = cvRound(x0 - 600 * (-b));
		point2.y = cvRound(y0 - 600 * (a));
		point3.x = 300, point3.y = 300;
		point4.x = 300, point4.y = 600;
		point5.x = 300, point5.y = 0;

		cvLine(rawData_, point3, point4, CV_RGB(0,0,255), 1, CV_AA, 0);
		cvLine(rawData_, point3, point5, CV_RGB(0,0,255), 1, CV_AA, 0);

		if (intersect(point1, point2, point3, point4)) {
			{
				if(abs(left_angle_ -( (theta) - CV_PI / 2)) < 0.2){
					rrhomean += rho;
					rthetamean += theta;
					rmc++;
					cvLine(workingImg_, point1, point2, CV_RGB(0,0,255), 1, CV_AA, 0);
				}
			}
		} else if (intersect(point1, point2, point3, point5)) {
			{
				if(abs(right_angle_ -( (theta) - CV_PI / 2)) < 0.5){
					lrhomean += rho;
					lthetamean += theta;
					lmc++;
					cvLine(workingImg_, point1, point2, CV_RGB(255,255,255), 1,CV_AA, 0);
				}
			}
		}
	}
	if(lmc > 5){

		theta = lthetamean / lmc;
		rho = lrhomean / lmc;

		a = cos(theta);
		b = sin(theta);
		x0 = a * rho;
		y0 = b * rho;
		point1.x = cvRound(x0 + 600 * (-b)), point1.y = cvRound(y0 + 600 * (a));
		point2.x = cvRound(x0 - 600 * (-b)), point2.y = cvRound(y0 - 600 * (a));

		cvLine(rawData_, point1, point2, CV_RGB(255,0,0), 3, CV_AA, 0);

		point4.x = 300;
		point4.y = 300;
		point5.x = point4.x + 200 * sin(CV_PI - (theta - CV_PI / 2));
		point5.y = point4.y + 200 * cos(CV_PI - (theta - CV_PI / 2));

		cvLine(rawData_, point5, point4, CV_RGB(255,255,255), 1, CV_AA, 0);

		rows_.header.stamp = ros::Time::now();
		rows_.leftvalid = false;
		rows_.rightvalid = false;

		if (intersect(point1, point2, point4, point5)) {
			right_distance_ = sqrt(((intersection_.y - 300) * (intersection_.y
					- 300)) + ((intersection_.x - 300) * (intersection_.x - 300)))
					* 2;

			right_angle_ = (theta) - CV_PI / 2;
			count++;

			rolling_mean_[count % 100][0] = right_angle_;
			right_angle_ = 0;
			for (int i = 0; i < 100; i++) {
				right_angle_ += rolling_mean_[i][0];
			}
			right_angle_ = right_angle_ / 100;

			rolling_mean_[count % 50][1] = right_distance_;
			right_distance_ = 0;
			for (int i = 0; i < 50; i++) {
				right_distance_ += rolling_mean_[i][1];
			}
			right_distance_ = right_distance_ / 50;


			inrow_cnt++;
			if(inrow_cnt > 10)
				inrow_cnt = 10;

			/*

			ROS_INFO("angle: %f",right_angle_);
			//cvLine(rawData_, point1, point2, CV_RGB(0,255,0), 1, CV_AA, 0);
	 */
			geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(
					right_angle_);

			marker_r.header.frame_id = "/laser_link";
			marker_r.header.stamp = ros::Time::now();

			marker_r.ns = "basic_shapes";
			marker_r.id = 0;

			// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
			marker_r.type = visualization_msgs::Marker::CUBE;

			// Set the marker action.  Options are ADD and DELETE
			marker_r.action = visualization_msgs::Marker::ADD;

			// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
			marker_r.pose.position.x = 0;
			marker_r.pose.position.y = -((float) right_distance_) / 100;
			marker_r.pose.position.z = 0;
			marker_r.pose.orientation = pose_quat;

			// Set the scale of the marker -- 1x1x1 here means 1m on a side
			marker_r.scale.x = 10.0;
			marker_r.scale.y = 0.1;
			marker_r.scale.z = 0.5;

			// Set the color -- be sure to set alpha to something non-zero!
			marker_r.color.r = 0.0f;
			marker_r.color.g = 1.0f;
			marker_r.color.b = 0.0f;
			marker_r.color.a = 0.5;

			marker_r.lifetime = ros::Duration(.5);

			// Publish the marker
			marker_r_pub.publish(marker_r);

			rows_.rightvalid = true;
			rows_.rightdistance = ((float) right_distance_) / 100;
			rows_.rightangle = right_angle_;

		} else {

			inrow_cnt--;
			inrow_cnt--;
			if(inrow_cnt < -5){
				inrow_cnt = -5;
			}
					/*
			left_distance_ = -1;
			left_angle_ = 6000;

			rows_.rightdistance = ((float) left_distance_) / 100;
			rows_.rightangle = theta - CV_PI / 2;
			*/
			//	printf("\nDistance from right:%dmm",hough_lines.right_distance);
			//sprintf(textright, "Distance from right: %dmm, angle: %d",hough_lines.right_distance, hough_lines.right_angle);
		}

	}else{

		ROS_INFO_THROTTLE(1,"lmc = %d", lmc);

		inrow_cnt--;
		inrow_cnt--;
		if(inrow_cnt < -5){
			inrow_cnt = -5;
		}
	}
	CvFont font;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 1, CV_AA);


	if(inrow_cnt > 0){
		twist_msg_.twist.angular.z = -((right_angle_*1.5) - (((float)right_distance_/100) - 1.2) );
		cvPutText(rawData_, "INROW-NAVIGATION", cvPoint(10, 130), &font, cvScalar(255, 255, 255, 0));
		cvLine(rawData_, cvPoint(10, 140), cvPoint(10 + abs(inrow_cnt * 20),140),CV_RGB(0,255,0), 3, CV_AA, 0);
		rows_.headland = false;
	}else{
		twist_msg_.twist.angular.z = M_PI/4;
		cvPutText(rawData_, "HEADLAND", cvPoint(10, 130), &font, cvScalar(255, 255, 255, 0));
		cvLine(rawData_, cvPoint(10, 140), cvPoint(10 + abs(inrow_cnt * 20), 140 ),CV_RGB(255,0,0), 3, CV_AA, 0);
		rows_.headland = true;
	}
	twist_pub.publish(twist_msg_);

	cvLine(rawData_, cvPoint(0, 300 + 150), cvPoint(600, 300 + 150),CV_RGB(255,255,255), 1, CV_AA, 0);
	cvLine(rawData_, cvPoint(0, 300 - 150), cvPoint(600, 300 - 150),CV_RGB(255,255,255), 1, CV_AA, 0);
	rows_.header.stamp = ros::Time::now();

	row_pub.publish(rows_);

	cvShowImage("TEST", rawData_);
	cvWaitKey(10);
	//pc_pub.publish(cloud);

}

int OrchardDetector::intersect(CvPoint p1, CvPoint p2, CvPoint p3, CvPoint p4)
/****************************************************************************
 *   Function : See module specification (.h-file).
 ****************************************************************************/
{
	// Store the values for fast access and easy
	// equations-to-code conversion
	float x1 = p1.x, x2 = p2.x, x3 = p3.x, x4 = p4.x;
	float y1 = p1.y, y2 = p2.y, y3 = p3.y, y4 = p4.y;

	float d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
	// If d is zero, there is no intersection
	if (d == 0)
		return 0;

	// Get the x and y
	float pre = (x1 * y2 - y1 * x2), post = (x3 * y4 - y3 * x4);
	float x = (pre * (x3 - x4) - (x1 - x2) * post) / d;
	float y = (pre * (y3 - y4) - (y1 - y2) * post) / d;

	// Check if the x and y coordinates are within both lines
	if (x < MIN(x2, x1) || x > MAX(x2, x1) || x < MIN(x4, x3) || x
			> MAX(x4, x3))
		return 0;
	if (y < MIN(y2, y1) || y > MAX(y2, y1) || y < MIN(y4, y3) || y
			> MAX(y4, y3))
		return 0;

	intersection_.x = x;
	intersection_.y = y;

	return 1;
}

