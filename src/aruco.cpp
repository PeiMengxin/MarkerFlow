
#include <iostream>
#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"
#include <sstream>
#include <serial/serial.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <thread>
#include <stdio.h>
#include <unistd.h>
#include "flow/ProfTimer.h"
#include "flow/iOpticalFlow.h"

#include <stdio.h>
#include <sys/time.h>
#include <string.h>
#include <unistd.h>
#include "AttitudePosition.h"
#include "MarkerWorldCoornidate.h"
#include "my_serial.h"

float flow_out[2];
void* thread_marker(void*);
void* thread_flow(void*);
pthread_t thread_m[10];
pthread_mutex_t mut,mut1;
int number=0, i_t;Mat frame_t1;
Mat frame_golbal[10];
extern std::vector<MarkerWorld> CoordinateTable;
serial::Serial serial_port("/dev/ttySAC0", 230400, serial::Timeout::simpleTimeout(1000));
cv::Point3f coordinate_camera;
Attitude attitude_camera;
std::vector< aruco::Marker > Markers;

using namespace std;
using namespace cv;
using namespace aruco;

#define TRANS_WORLD 1
#define WRITE_VIDEO  0

#define MED_WIDTH_NUM 50
#define MED_FIL_ITEM  20

float med_filter_tmp[MED_FIL_ITEM][MED_WIDTH_NUM];
float med_filter_out[MED_FIL_ITEM];

int med_fil_cnt[MED_FIL_ITEM];
// 1  2  3                                9
float Moving_Median(int item, int width_num, float in)
{
    int i, j;
    float t;
    float tmp[MED_WIDTH_NUM];

    if (item >= MED_FIL_ITEM || width_num >= MED_WIDTH_NUM)
    {
        return 0;
    }
    else
    {
        if (++med_fil_cnt[item] >= width_num)
        {
            med_fil_cnt[item] = 0;
        }

        med_filter_tmp[item][med_fil_cnt[item]] = in;

        for (i = 0; i < width_num; i++)
        {
            tmp[i] = med_filter_tmp[item][i];
        }

        for (i = 0; i < width_num - 1; i++)
        {
            for (j = 0; j<(width_num - 1 - i); j++)
            {
                if (tmp[j] > tmp[j + 1])
                {
                    t = tmp[j];
                    tmp[j] = tmp[j + 1];
                    tmp[j + 1] = t;
                }
            }
        }
        return (tmp[(int)width_num / 2]);
    }
}

Mat getHistogram1DImage(const Mat& hist, Size imgSize)
 {
    Mat histImg(imgSize, CV_8UC3);
    int Padding = 10;
   int W = imgSize.width - 2 * Padding;
    int H = imgSize.height - 2 * Padding;
   double _max;
   minMaxLoc(hist, NULL, &_max);
   double Per = (double)H / _max;
    const Point Orig(Padding, imgSize.height-Padding);
   int bin = W / (hist.rows + 2);

   //画方柱
    for (int i = 1; i <= hist.rows;i++)
    {
         Point pBottom(Orig.x + i * bin, Orig.y);
       Point pTop(pBottom.x, pBottom.y - Per * hist.at<float>(i-1));
       line(histImg, pBottom, pTop, Scalar(255, 0, 0), bin);
     }

    //画 3 条红线标明区域
    line(histImg, Point(Orig.x + bin, Orig.y - H), Point(Orig.x + hist.rows *  bin, Orig.y - H), Scalar(0, 0, 255), 1);
    line(histImg, Point(Orig.x + bin, Orig.y), Point(Orig.x + bin, Orig.y - H), Scalar(0, 0, 255), 1);
   line(histImg, Point(Orig.x + hist.rows * bin, Orig.y), Point(Orig.x + hist.rows *  bin, Orig.y - H), Scalar(0, 0, 255), 1);

    return histImg;
}

void initTraj(cv::Mat &traj, float rectsize, float offset)
{
    Point origin(offset, offset);
    for (size_t i = 0; i < 36; i++)
    {
        Rect rect((CoordinateTable[i].coordinate.x - rectsize / 2) * 2 + origin.x, (CoordinateTable[i].coordinate.y - rectsize / 2) * 2 + origin.y, rectsize * 2, rectsize * 2);

        rectangle(traj, rect, Scalar(0, 0, 0), 2);
    }

    cv::line(traj, origin, Point(origin.x, 690), Scalar(72, 61, 139), 2);
    cv::line(traj, origin, Point(690, origin.y), Scalar(72, 61, 139), 2);

    cv::line(traj, Point(690, origin.y), Point(690 - 20, origin.y - 10), Scalar(72, 61, 139), 2);
    cv::line(traj, Point(690, origin.y), Point(690 - 20, origin.y + 10), Scalar(72, 61, 139), 2);

    cv::line(traj, Point(origin.x, 690), Point(origin.x - 10, 690 - 10), Scalar(72, 61, 139), 2);
    cv::line(traj, Point(origin.x, 690), Point(origin.x + 10, 690 - 10), Scalar(72, 61, 139), 2);
}
cv::Mat image_camera,image_camera1,image_camera2;
bool isEnd = false;
VideoCapture cap;

void hist_show(Mat u_hist)
{
Mat* arrays = &u_hist;
int narrays = 1;
int channels[] = { 0 };
InputArray mask = noArray();
Mat hist;
int dims = 1;
int histSize[] = { 256 };
float hranges[] = { 0.0, 255.0 };
const float *ranges[] = { hranges };
 //调用 calcHist 计算直方图, 结果存放在 hist 中
calcHist(arrays, narrays, channels, mask, hist, dims, histSize, ranges);
Mat histImg = getHistogram1DImage(hist, Size(600, 420));

imshow("lena gray image histogram", histImg);
}

int calcRTfromHomo(CvMat* H, Mat &t, Mat &rodrot){

    double r[9];
    CvMat _r = cvMat(3, 3, CV_64F, r);          //rotation matrix
    double intrinsic[9]={1, 0, 0, 0, 1, 0, 0, 0, 1};
    CvMat _M = cvMat(3, 3, CV_64F, intrinsic); //intrinsic matrix, of no use in this implementation, reserved for future use
    double ones[]={1,1,1};
    CvMat _ones = cvMat(3, 1, CV_64F, ones);
    double rodrot1[3];
    CvMat _rodrot = cvMat(3, 1, CV_64F,rodrot1);
    //for SVD
    CvMat* U = cvCreateMat(3, 3, CV_64F);
    CvMat* W = cvCreateMat(3, 3, CV_64F);
    CvMat* V = cvCreateMat(3, 3, CV_64F);
    CvMat* invM = cvCreateMat(3, 3, CV_64F);
    // three columns of Homography matrix
    CvMat* h1 = cvCreateMat(3, 1, CV_64F);
    CvMat* h2 = cvCreateMat(3, 1, CV_64F);
    CvMat* h3 = cvCreateMat(3, 1, CV_64F);
    // three columns of rotation matrix
    CvMat* r1 = cvCreateMat(3, 1, CV_64F);
    CvMat* r2 = cvCreateMat(3, 1, CV_64F);
    CvMat* r3 = cvCreateMat(3, 1, CV_64F);
    // translation vector
    double t1[3];
    CvMat _t =  cvMat(3, 1, CV_64F,t1);
    cvGetCol(H,h1,0);
    cvGetCol(H,h2,1);
    cvGetCol(H,h3,2);
    cvGetCol(&_r,r1,0);
    cvGetCol(&_r,r2,1);
    cvGetCol(&_r,r3,2);

    cvInvert(&_M, invM);
    cvMatMul(invM,h1,r1);
    cvMatMul(invM,h2,r2);
    cvMatMul(invM,h3,&_t);

    cvNormalize(r1, r1);
    cvMul(r2,&_ones,r2,1/cvNorm(r1));
    cvMul(&_t,&_ones, &_t,1/cvNorm(r1) );
    cvCrossProduct(r1, r2, r3);
    cvSVD(&_r, W, U, V, CV_SVD_V_T);
    cvMatMul(U,V,&_r);
    cvRodrigues2(&_r, &_rodrot, NULL);
    Mat t_temp(&_t,true);
    Mat rodrot_temp(&_rodrot,true);
    t_temp.copyTo(t);
    rodrot_temp.copyTo(rodrot);
    return 1;
}

void featureDetection(Mat img_1, vector<Point2f>& points1)	{
  vector<KeyPoint> keypoints_1;
  int fast_threshold = 20;
  bool nonmaxSuppression = true;
  FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
  KeyPoint::convert(keypoints_1, points1, vector<int>());
}

void featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status)	{

//this function automatically gets rid of points for which tracking fails

  vector<float> err;
  Size winSize=Size(21,21);
  TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);
 calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);

  //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
  int indexCorrection = 0;
  for( int i=0; i<status.size(); i++)
     {  Point2f pt = points2.at(i- indexCorrection);
        if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))	{
              if((pt.x<0)||(pt.y<0))	{
                status.at(i) = 0;
              }
              points1.erase (points1.begin() + i - indexCorrection);
              points2.erase (points2.begin() + i - indexCorrection);
              indexCorrection++;
        }

     }

}


#define  MIN_NUM_FEAT 8*2
float scale=0.8;
void* thread_flow1(void*){
    Mat img_1_c,img_2_c;
    Mat img_1, img_2;
    Mat R_f, t_f; //the final rotation and tranlation vectors containing the
    Size m_size(320*scale, 280*scale);
    Mat temp;
    pthread_mutex_lock(&mut);
    cap >> temp;
    pthread_mutex_unlock(&mut);
    resize(temp, img_1_c, m_size);
    resize(temp, img_2_c, m_size);
    // we work with grayscale images
    cvtColor(img_1_c, img_1, COLOR_BGR2GRAY);
    cvtColor(img_2_c, img_2, COLOR_BGR2GRAY);

    // feature detection, tracking
    vector<Point2f> points1, points2;        //vectors to store the coordinates of the feature points
    featureDetection(img_1, points1);        //detect features in img_1
    vector<uchar> status;
    featureTracking(img_1,img_2,points1,points2, status); //track those features to img_2
    Mat E, R, t;
    E=findHomography(points2, points1,CV_RANSAC,3);
    CvMat E_temp = E;
    calcRTfromHomo(&E_temp,t,R);
    Mat prevImage = img_2;
    Mat currImage;
    vector<Point2f> prevFeatures = points2;
    vector<Point2f> currFeatures;
    R_f = R.clone();
    t_f = t.clone();
    cout<<"4_str"<<endl;
    ProfTimer time;
     //TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03);
     TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);
    Size subPixWinSize(10,10), winSize(21,21);


    const int MAX_COUNT = 888;
    bool needToInit = true;
   vector<Point2f> points[2];
   bool addRemovePt = false;
   Point2f point;
   int flag_init;
  while(1)	{
    time.Start ();
    Mat currImage_c, currImage_c_size;
    pthread_mutex_lock(&mut);
    cap >> currImage_c_size;
    pthread_mutex_unlock(&mut);
    resize(currImage_c_size, currImage_c, m_size);
    cvtColor(currImage_c, currImage, COLOR_BGR2GRAY);
    vector<uchar> status;
   if(needToInit )
    {cout << "automatic initialization " << endl;
        // automatic initialization
       //goodFeaturesToTrack(currImage, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
       featureDetection(currImage, points[1]);
        //cornerSubPix(currImage, points[1], subPixWinSize, Size(-1,-1), termcrit);
        needToInit = false;
    }

    else if( !points[0].empty() )
           {
               vector<uchar> status;
               vector<float> err;
               if(prevImage.empty())
                   currImage.copyTo(prevImage);
               calcOpticalFlowPyrLK(prevImage, currImage, points[0], points[1], status, err, winSize,
                                    3, termcrit, 0, 0.001);

               int indexCorrection = 0;
               for( int i=0; i<status.size(); i++)
                  {  Point2f pt = points[1].at(i- indexCorrection);
                     if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))	{
                           if((pt.x<0)||(pt.y<0))	{
                             status.at(i) = 0;
                           }
                           points[0].erase (points[0].begin() + i - indexCorrection);
                           points[1].erase (points[1].begin() + i - indexCorrection);
                           indexCorrection++;
                     }
                  }

               if (points[0].size() > 8&&!flag_init){
               E=cv::findHomography(points[0],points[1],CV_RANSAC,3);
               CvMat E_temp = E; calcRTfromHomo(&E_temp,t,R);
               int temp_1,temp_2,temp_1f,temp_2f;

               temp_1=int(t.at<double>(0,0)*10);temp_2=int(t.at<double>(0,1)*10);
               temp_1f=Moving_Median(0,5,temp_1);
               temp_2f=Moving_Median(1,5,temp_2);

               flow_out[0]=temp_1f;
               flow_out[1]=temp_2f;
               static int cnt;
               if(cnt++>0){cnt=0;
               //cout << t*10 << endl;
               cout << temp_1f <<"    "<<temp_2f<< endl;
               }
               }
               flag_init=0;
               size_t i, k;
               for( i = k = 0; i < points[1].size(); i++ )
               {
                   if( addRemovePt )
                   {
                       if( norm(point - points[1][i]) <= 5 )
                       {
                           addRemovePt = false;
                           continue;
                       }
                   }


                   if( !status[i] )
                       continue;


                   points[1][k++] = points[1][i];
                   circle( currImage_c, points[1][i], 2, Scalar(0,255,0), 1, 8,0);

               }
               points[1].resize(k);
           }

        static int cnt_rst;
        if(cnt_rst++>50){cnt_rst=0;needToInit=true;}

        if (points[0].size() < MIN_NUM_FEAT)	{
            vector<uchar> status;
            vector<float> err;
             featureDetection(prevImage, points[0]);\
             //goodFeaturesToTrack(prevImage, points[0], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
             if(points[0].size()>8)
             {cornerSubPix(prevImage, points[0], subPixWinSize, Size(-1,-1), termcrit);
             calcOpticalFlowPyrLK(prevImage, currImage, points[0], points[1], status, err, winSize,
                                  3, termcrit, 0, 0.001);
             }
             flag_init=1;
             cout << "Low feature" << endl;
             //goodFeaturesToTrack(currImage, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
             //cornerSubPix(prevImage, points[0], subPixWinSize, Size(-1,-1), termcrit);
        }

         std::swap(points[1], points[0]);
         cv::swap(prevImage, currImage);
          imshow("LK Demo", currImage_c);

    /*
    if (prevFeatures.size() < MIN_NUM_FEAT)	{
        //cout << "Number of tracked features reduced to " << prevFeatures.size() << endl;
       // cout << "trigerring redection" << endl;
        featureDetection(prevImage, prevFeatures);
        featureTracking(prevImage,currImage,prevFeatures,currFeatures, status);
   }

    featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);
    if (prevFeatures.size() > 8){
    E=findHomography(currFeatures,prevFeatures,CV_RANSAC,3);
    CvMat E_temp = E; calcRTfromHomo(&E_temp,t,R);
    static int cnt;
    if(cnt++>2){cnt=0;
    cout << t << endl;}
    }


   Mat prevPts(2,prevFeatures.size(), CV_64F), currPts(2,currFeatures.size(), CV_64F);


   for(int i=0;i<prevFeatures.size();i++)	{   //this (x,y) combination makes sense as observed from the source code of triangulatePoints on GitHub
        prevPts.at<double>(0,i) = prevFeatures.at(i).x;
        prevPts.at<double>(1,i) = prevFeatures.at(i).y;

        currPts.at<double>(0,i) = currFeatures.at(i).x;
        currPts.at<doucalcOpticalFlowPyrLKble>(1,i) = currFeatures.at(i).y;
    }

    //scale = getAbsoluteScale(numFrame, 0, t.at<double>(2));cout << "Scale is " << scale << endl;

    if ((scale>0.1)&&(t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {
      //t_f = t_f + scale*(R_f*t);
      //R_f = R*R_f;
    }


   Mat temp=currImage_c;
   for( int i = 0; i < currFeatures.size(); i++ ){
       circle( temp, currFeatures[i], 2, Scalar(0,255,0), 1, 8, 0 );
   }
   imshow("motionField",temp);
    prevImage = currImage.clone();
    prevFeatures = currFeatures;
*/
   time.Stop();
 // cout<<"time=: "<<time.GetDurationInSecs ()*1000<<"(ms)"<<endl;
    waitKey(15);

  }
  pthread_exit(NULL);
}

void get_flow_finial(Mat u,Mat v,float out_u,float out_v,int mode)
{float temp,temp1;
    switch(mode)
    {
    case 0:
        for(int i=0;i<u.total()/2;i++)
            temp+=u.at<float>(i);
        out_u=temp/u.total();
        for(int i=0;i<v.total()/2;i++)
            temp1+=v.at<float>(i);
        out_v=temp1/v.total();
        cout<<out_u*10<<"  "<<out_v*10<<endl;
      break;
    }
}


void caculateOpticalFlow_images(Mat& c,Mat& n,int wk)
{
    Mat gc,gn;
    cvtColor(c,gc,CV_RGB2GRAY);
    cvtColor(n,gn,CV_RGB2GRAY);

    Mat u,v;
    Mat motionImage=Mat::zeros (c.size (),CV_8UC3);
    Mat motionImage_hist=Mat::zeros (c.size (),CV_8UC3);
    c.copyTo(motionImage);
    c.copyTo(motionImage_hist);
    int width,height;
    cv::Size size=c.size ();
    width=size.width ;
    height=size.height ;

    int max_level = wk;
    int start_level = 0;
    /*/平滑迭代次数
    int n1;
    int n2;

    //高斯平滑参数
    float rho;
    float alpha;
    //高斯平滑参数
    float sigma;*/
    //Two pre and post smoothing steps, should be greater than zero
    int n1 =2;
    int n2 =2;

    //Smoothing and regularization parameters, experiment but keep them above zero
    float rho = 2;
    float alpha = 5000;//1400;
    float sigma = 1.5;

    ProfTimer t;
    OpticalFlow of(cv::Size (width, height), max_level, start_level, n1, n2, rho, alpha, sigma);
    t.Start ();
    caculateOpticalFlow(of,gc,gn,u,v);
    t.Stop();
    drawMotionField(u, v, motionImage, 10, 10, 1, 5,cv::Scalar (0,255,255));
    Mat p1(u.total(),2,CV_32F);
    Mat p2(u.total(),2,CV_32F);
    for(int i=0;i<width;i++)
     for(int j=0;j<height;j++)
    { int temp_pos=i*width+j;
        p1.at<float>(temp_pos,0)=i;
        p1.at<float>(temp_pos,1)=j;
        p2.at<float>(temp_pos,0)=i+u.at<float>(j,i);
        p2.at<float>(temp_pos,1)=j+v.at<float>(j,i); ;
    }
    Mat F;
    F=findHomography(p1,p2,CV_RANSAC,3);
    Mat Tvec,rodrot;
    CvMat F_temp = F;
    float U,V;
    calcRTfromHomo(&F_temp,Tvec,rodrot);
    get_flow_finial(u,v,U,V,0);
    static int cnt;
    if(cnt++>3){cnt=0;
    //cout<<Tvec*10<<endl;
    }

    Mat Rot(3, 3, CV_32FC1);
    Rodrigues(rodrot, Rot);

    Rot = Rot.t();  // rotation of inverse
    static Mat pos_camera = Rot * Tvec; // translation of inverse
    pos_camera = pos_camera+Rot * Tvec;
    //cout<<pos_camera<<endl;
    //drawMotionField(u_hist_32f[0], v_hist_32f[0], motionImage_hist, 10, 10, 1, 5,cv::Scalar (0,255,255));
   // cout<<"time=: "<<t.GetDurationInSecs ()*1000<<"(ms)"<<endl;
   imshow("motionField",motionImage);
   //imshow("motionField_hist",motionImage_hist);
}

float scale_flow=1;
float focus=1;
void* thread_flow(void*)
{
    Size m_size(128*scale_flow, 128*scale_flow);
    Mat c(m_size, CV_8UC3), n(m_size, CV_8UC3);
    Mat pic_t2;
    cout<<"2_str"<<endl;
    for(;;)
    { //cout<<"2"<<endl;
        pthread_mutex_lock(&mut);
        cap >> pic_t2;
        //frame_golbal[2].copyTo(pic_t2);
        pthread_mutex_unlock(&mut);
        Rect rect(pic_t2.cols/2-focus*pic_t2.cols/2,pic_t2.rows/2-focus*pic_t2.rows/2,
                  focus*pic_t2.cols/2,focus*pic_t2.rows/2);
        Mat temp,temp_gauss;
        pic_t2(rect).copyTo(temp);
        GaussianBlur(temp,temp_gauss,Size(5,5),0,0);
        resize(temp_gauss, n, m_size);
        caculateOpticalFlow_images(c, n,5);
        n.copyTo(c);
        char c = (char)waitKey(30);
        if( c == 27 )
            break;
    }
    pthread_exit(NULL);
}

#define MARKER_USE_320 1

void* thread_marker(void*)
{
    Mat InImage;
    Mat pic_t1;
    initCoordinateTable(CoordinateTable);
#if MARKER_USE_320
    string cameraParamFileName("/home/odroid/workspace/QT/MarkerFlow/PS3_320.yml");
    cv::Size InImage_size(320,240);
#else
    string cameraParamFileName("/home/odroid/workspace/QT/MarkerFlow/PS3_640.yml");
    cv::Size InImage_size(640,480);
#endif
    aruco::CameraParameters CamParam;
    MarkerDetector MDetector;
    float MarkerSize = MARKER_SIZE;
    cv::namedWindow("thes", 1);
    int p1 = 7;
    int p2 = 7;
    int t_p_range = 2;
    createTrackbar("p1", "thes", &p1, 101);
    createTrackbar("p2", "thes", &p2, 50);
    createTrackbar("range", "thes", &t_p_range, 31);
    ostringstream ostr_pos;
    ostringstream ostr_angle;
    Point3f pos_camera(0, 0, 0);
    Mat traj(720, 720, CV_8UC3, Scalar::all(255));
    float offset = 50;
    initTraj(traj, MarkerSize, offset);
    Mat traj_empty = traj.clone();
    int drawPointSize = 50;
    vector<Point2f> drawPointKF;
    vector<Point2f> drawPointSrc;
    drawPointKF.resize(drawPointSize,Point2f(0,0));
    drawPointSrc.resize(drawPointSize, Point2f(0, 0));
    cout<<"1_str"<<endl;

    cv::TickMeter tm;
    for(;;)
    { // cout<<"1"<<endl;
        tm.reset();
        tm.start();
        pthread_mutex_lock(&mut);
        cap >> pic_t1;
        cv::resize(pic_t1, InImage, InImage_size);
        //frame_golbal[1].copyTo(InImage);
        pthread_mutex_unlock(&mut);
        static int init;
        if(!init){init=1;
            //read camera parameters if specifed
            CamParam.readFromXMLFile(cameraParamFileName);
            // resizes the parameters to fit the size of the input image
            CamParam.resize(InImage_size);

            cout << CamParam.CameraMatrix << endl;
            cout << CamParam.Distorsion << endl;// cap >> pic_t1;
        }
        p1 = p1 / 2 * 2 + 1;
        p2 = p2 / 2 * 2 + 1;
        MDetector.setThresholdParamRange(t_p_range);
        MDetector.setThresholdParams(p1, p2);
        // Ok, let's detect
        MDetector.detect(InImage, Markers, CamParam, MarkerSize);
        // for each marker, draw info and its boundaries in the image
        for (unsigned int i = 0; i < Markers.size(); i++)
        {
            Markers[i].draw(InImage, Scalar(0, 0, 255), 2);
        }

        if (CamParam.isValid() && MarkerSize != -1)
        {
            for (unsigned int i = 0; i < Markers.size(); i++)
            {
                CvDrawingUtils::draw3dAxis(InImage, Markers[i], CamParam);
                getAttitude(Markers[i], attitude_camera);

                ostr_angle.clear();
                ostr_angle.str("");
                //ostr_angle << "          Pit=" << (int)1.1 << " " << "Yaw=" << (int)2.8 << " " << "Rol=" << (int)-1.3;
                ostr_angle << "          Pit=" << (int)attitude_camera.Pit << " " << "Yaw=" << (int)attitude_camera.Yaw << " " << "Rol=" << (int)attitude_camera.Rol;

#if TRANS_WORLD
                getCameraPos(Markers[i].Rvec, Markers[i].Tvec, pos_camera);

                ostr_pos.clear();
                ostr_pos.str("");
                ostr_pos << "          x=" << (int)pos_camera.x << " " << "y=" << (int)pos_camera.y << " " << "z=" << (int)pos_camera.z;
                putText(InImage, ostr_pos.str(), Markers[i].getCenter() - Point2f(0, 20), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(55, 255, 0), 2);

#endif
                putText(InImage, ostr_angle.str(), Markers[i].getCenter() + Point2f(0, 20), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(55, 255, 0), 2);

            }
        }
        coordinate_camera = Point3f(0,0,0);
        getCameraPosWithMarkers(Markers, coordinate_camera,attitude_camera, 3);

        ostr_pos.clear();
        ostr_pos.str("");
        ostr_pos << "          x=" << (int)coordinate_camera.x << " " << "y=" << (int)coordinate_camera.y << " " << "z=" << (int)attitude_camera.Yaw;

        putText(InImage, ostr_pos.str(), Point(30, 30), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(55, 255, 0), 2);

        static float x_show = 0, y_show = 0;
        static float x = 0;
        static float y = 0;
        if ((int(coordinate_camera.x) != 0) && (int(coordinate_camera.y) != 0))
        {
            x = (coordinate_camera.x) * 2;
            y = (coordinate_camera.y) * 2;
        }

        x_show = x;
        y_show = y;

        drawPointSrc.insert(drawPointSrc.begin(), Point(x_show, y_show) + Point(offset, offset));
        drawPointSrc.pop_back();

        for (size_t i = 0; i < drawPointSrc.size(); i++)
        {
            circle(traj, drawPointSrc[i], 8, CV_RGB(255, 0, 0), 1);
        }

        //circle(traj, Point(x_show, y_show)+Point(offset,offset), 8, CV_RGB(255, 0, 0), 1);
        // cout<<"x=: "<<x_show<<"   y=: "<<y_show<< endl;
        //imshow("Trajectory", traj);
        //show input with augmented information
        cv::imshow("in", InImage);
        // show also the internal image resulting from the threshold operation
        //cv::imshow("thes", MDetector.getThresholdedImage());
        traj = traj_empty.clone();

        tm.stop();
       // cout<<"marker:"<<tm.getTimeMilli()<<"ms"<<endl;

        char c = (char)waitKey(50);
        if( c == 27 )
            break;
    }
    pthread_exit(NULL);
}


void* thread_uart(void*)
{
    for(;;)
    {
        pthread_mutex_lock(&mut);
        //cout<<"Uart good!"<<endl;
        uartSent();
        pthread_mutex_unlock(&mut);
        char c = (char)waitKey(25);
        if( c == 27 )
            break;
    }
    pthread_exit(NULL);
}

void* thread_readcamera(void*)
{
    for(;;)
    {
        pthread_mutex_lock(&mut);
        cap >> frame_golbal[2];
        cap >> frame_golbal[1];
        pthread_mutex_unlock(&mut);
        char c = (char)waitKey(10);
        if( c == 27 )
            break;
    }
    pthread_exit(NULL);
}

void thread_create(void)
{
    memset(&thread_m, 0, sizeof(thread_m)); //comment1
    pthread_create(&thread_m[0], NULL, thread_marker, NULL); //comment2
    pthread_create(&thread_m[1], NULL, thread_flow1, NULL); //comment2
    pthread_create(&thread_m[2], NULL, thread_uart, NULL); //comment2
    // pthread_create(&thread_m[3], NULL, thread_readcamera, NULL); //comment2
}
void thread_wait(void)
{
    if(thread_m[0] !=0) { //comment4
        pthread_join(thread_m[0],NULL);
    }
    if(thread_m[1] !=0) { //comment5
        pthread_join(thread_m[1],NULL);
    }
    if(thread_m[2] !=0) { //comment5
        pthread_join(thread_m[2],NULL);
    }
    if(thread_m[3] !=0) { //comment5
        pthread_join(thread_m[3],NULL);
    }
}

int main(void)
{
    try
    {
        cap.open(0);
        usleep(1000*1000);
        thread_create();
        thread_wait();
//        while(1){
//            cout<<"idle"<<endl;
//            waitKey(15);
//        };

        return 0;
    }
    catch (std::exception &ex)
    {
        cout << "Exception :" << ex.what() << endl;
    }
}
