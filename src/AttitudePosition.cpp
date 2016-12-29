#include "AttitudePosition.h"
#include "MarkerWorldCoornidate.h"
#include <vector>

using namespace std;
using namespace cv;
using namespace aruco;

extern std::vector<MarkerWorld> CoordinateTable;

//��������������
bool SortByDown(const float &p1, const float &p2)//��������������������������������vector����������������
{
	return p1 > p2;//��������
}

std::vector<cv::Mat> getR(float alpha_X, float alpha_Y, float alpha_Z)
{
	Mat R_X = Mat::eye(3, 3, CV_32FC1);
	Mat R_Y = Mat::eye(3, 3, CV_32FC1);
	Mat R_Z = Mat::eye(3, 3, CV_32FC1);

	alpha_X /= 57.3;
	alpha_Y /= 57.3;
	alpha_Z /= 57.3;

	R_X.at<float>(1, 1) = cos(alpha_X);
	R_X.at<float>(1, 2) = sin(alpha_X);
	R_X.at<float>(2, 1) = -sin(alpha_X);
	R_X.at<float>(2, 2) = cos(alpha_X);

	R_Y.at<float>(0, 0) = cos(alpha_Y);
	R_Y.at<float>(0, 2) = -sin(alpha_Y);
	R_Y.at<float>(2, 0) = sin(alpha_Y);
	R_Y.at<float>(2, 2) = cos(alpha_Y);

	R_Z.at<float>(0, 0) = cos(alpha_Z);
	R_Z.at<float>(0, 1) = sin(alpha_Z);
	R_Z.at<float>(1, 0) = -sin(alpha_Z);
	R_Z.at<float>(1, 1) = cos(alpha_Z);

	vector<Mat> dst;
	dst.push_back(R_X);
	dst.push_back(R_Y);
	dst.push_back(R_Z);

	return dst;

}

void getCameraPos(cv::Mat Rvec, cv::Mat Tvec, cv::Point3f &pos)
{
	Mat Rot(3, 3, CV_32FC1);
	Rodrigues(Rvec, Rot);

	Rot = Rot.t();  // rotation of inverse
	Mat pos_camera = -Rot * Tvec; // translation of inverse

	pos.x = pos_camera.at<float>(0, 0);
	pos.y = -pos_camera.at<float>(1, 0);
	pos.z = pos_camera.at<float>(2, 0);
}

void getCameraPosWithMarkers(std::vector< aruco::Marker > Markers, cv::Point3f &pos_camera, Attitude &atti_camera, int flag /*= 0*/)
{
	Point3f pos_world(0, 0, 0);
	Attitude atti_t;

	switch (flag)
	{
		case 0:
		{
			for (unsigned int i = 0; i < Markers.size(); i++)
			{
				getCameraPos(Markers[i].Rvec, Markers[i].Tvec, pos_world);

				pos_camera += CoordinateTable[Markers[i].id - 1].coordinate + pos_world;

			}

			if (Markers.size()>0)
			{
				pos_camera.x = pos_camera.x / Markers.size();
				pos_camera.y = pos_camera.y / Markers.size();
				pos_camera.z = pos_camera.z / Markers.size();
			}

			break;
		}
		case 1:
		{
			float dis = 0;
			float dismin = 100000;

			for (unsigned int i = 0; i < Markers.size(); i++)
			{
				getCameraPos(Markers[i].Rvec, Markers[i].Tvec, pos_world);

				dis = sqrt(pos_world.x*pos_world.x + pos_world.y*pos_world.y);

				if (dis<dismin)
				{
					dismin = dis;
					pos_camera = CoordinateTable[Markers[i].id - 1].coordinate + pos_world;
				}
			}
			break;
		}
		case 2:
		{
			std::vector<float> vec_x, vec_y, vec_yaw;
			for (unsigned int i = 0; i < Markers.size(); i++)
			{
				getCameraPos(Markers[i].Rvec, Markers[i].Tvec, pos_world);
				getAttitude(Markers[i], atti_t);
				pos_camera = CoordinateTable[Markers[i].id - 1].coordinate + pos_world;
				vec_x.push_back(pos_camera.x);
				vec_y.push_back(pos_camera.y);
				vec_yaw.push_back(atti_t.Yaw);
			}
			if (vec_x.size()>0)
			{
				sort(vec_x.begin(), vec_x.end(), SortByDown);
				sort(vec_y.begin(), vec_y.end(), SortByDown);
				sort(vec_yaw.begin(), vec_yaw.end(), SortByDown);

				pos_camera.x = vec_x[vec_x.size() / 2];
				pos_camera.y = vec_y[vec_y.size() / 2];
				//atti_camera.Yaw = vec_yaw[vec_yaw.size() / 2];
			}
			

			break;
		}
		case 3:
				{
					std::vector<float> vec_x, vec_y, vec_z,vec_pit,vec_rol,vec_yaw;
					for (unsigned int i = 0; i < Markers.size(); i++)
					{
						getCameraPos(Markers[i].Rvec, Markers[i].Tvec, pos_world);//camera to local marker o dis
						getAttitude(Markers[i], atti_t);
						//pos_camera = CoordinateTable[Markers[i].id - 1].coordinate + pos_world;//camera to global dis
						pos_camera.x=pos_world.x+CoordinateTable[Markers[i].id].coordinate.x;
						pos_camera.y=pos_world.y+CoordinateTable[Markers[i].id].coordinate.y;
						pos_camera.z=pos_world.z;
						vec_x.push_back(pos_camera.x);
						vec_y.push_back(pos_camera.y);
						vec_z.push_back(pos_camera.z);
						vec_pit.push_back(atti_t.Pit);
						vec_rol.push_back(atti_t.Rol);
						vec_yaw.push_back(atti_t.Yaw);
					}
					if (vec_x.size()>3)
					{
						sort(vec_x.begin(), vec_x.end(), SortByDown);
						sort(vec_y.begin(), vec_y.end(), SortByDown);
						sort(vec_z.begin(), vec_z.end(), SortByDown);
						sort(vec_pit.begin(), vec_pit.end(), SortByDown);
						sort(vec_rol.begin(), vec_rol.end(), SortByDown);
						sort(vec_yaw.begin(), vec_yaw.end(), SortByDown);
						float temp[6]={0};
						for(unsigned int i=1;i<vec_x.size()-1;i++)
						temp[0]+=vec_x[i];
						for(unsigned int i=1;i<vec_y.size()-1;i++)
						temp[1]+=vec_y[i];
						for(unsigned int i=1;i<vec_z.size()-1;i++)
						temp[2]+=vec_z[i];
						for(unsigned int i=1;i<vec_pit.size()-1;i++)
						temp[3]+=vec_pit[i];
						for(unsigned int i=1;i<vec_rol.size()-1;i++)
						temp[4]+=vec_rol[i];
						for(unsigned int i=1;i<vec_yaw.size()-1;i++)
						temp[5]+=vec_yaw[i];
						pos_camera.x=temp[0]/(vec_x.size()-2);
						pos_camera.y=temp[1]/(vec_y.size()-2);
						pos_camera.z=temp[2]/(vec_z.size()-2);
						atti_camera.Pit=temp[3]/(vec_pit.size()-2);
						atti_camera.Rol=temp[4]/(vec_rol.size()-2);
						atti_camera.Yaw=temp[5]/(vec_yaw.size()-2);
					}else if(vec_x.size()>0){
					float temp[6]={0};
					for(unsigned int i=0;i<vec_x.size();i++)
					temp[0]+=vec_x[i];
					for(unsigned int i=0;i<vec_y.size();i++)
					temp[1]+=vec_y[i];
					for(unsigned int i=0;i<vec_z.size();i++)
					temp[2]+=vec_z[i];
					for(unsigned int i=0;i<vec_pit.size();i++)
					temp[3]+=vec_pit[i];
					for(unsigned int i=0;i<vec_rol.size();i++)
					temp[4]+=vec_rol[i];
					for(unsigned int i=0;i<vec_yaw.size();i++)
					temp[5]+=vec_yaw[i];
					pos_camera.x=temp[0]/vec_x.size();
					pos_camera.y=temp[1]/vec_y.size();
					pos_camera.z=temp[2]/vec_z.size();
					atti_camera.Pit=temp[3]/vec_pit.size();
					atti_camera.Rol=temp[4]/vec_rol.size();
					atti_camera.Yaw=temp[5]/vec_yaw.size();
					}


					break;
				}
			
		default:
			break;
	}
	
}

void getAttitude(aruco::Marker marker, Attitude &attitude)
{
	double pos[3] = { 0 };
	double ori[4] = { 0 };

	double q0, q1, q2, q3;

	marker.OgreGetPoseParameters(pos, ori);
	pos[0] = -pos[0];
	pos[1] = -pos[1];

	q0 = ori[0]; q1 = ori[1]; q2 = ori[2]; q3 = ori[3];

	attitude.Pit = atan2(2 * (q0 * q1 + q2 * q3), -1 + 2 * (q1 * q1 + q2 * q2)) *57.3f;
	attitude.Rol = asin(2 * (q1 * q3 - q0 * q2)) *57.3f;
	attitude.Yaw = -atan2(2 * (-q1 * q2 - q0 * q3), 1 - 2 * (q0 * q0 + q1 * q1)) *57.3f;
}
