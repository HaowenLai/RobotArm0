/* *************************************************************
 *          Main task that the robotic arm performs
 *   This file is for robot arm 0
 *   The robotic arm will locate the target in three-dimensions.
 * Then, it will move to the place roughly and adjust precisely
 * by the feedback from camera attached to its clip. After that,
 * it will turn the bottom surface of the target and let it face
 * the front camera, and the program detects which letter it is.
 * Finally, judged by different detections, the arm moves the
 * target to corresponding places.
 *
 * @Author : Derek Lai (LHW)
 * @Date   : 2018/7/11
 * ************************************************************/

#include "parameters.hpp"
#include "UsbCAN.hpp"
#include "BpNetwork.hpp"
#include "ArucoMarker.hpp"
#include "RsVideoCapture.hpp"
#include "Wifi.hpp"
#include "control.hpp"

#include <pthread.h>
#include <stdio.h>
#include <iostream>

using namespace std;
using namespace cv;

#define _SINGLE_MOVE_MODE_
// #define _NO_VIDEO_MODE_

static void* camera_thread(void* data);

using namespace robot_arm::cameraParams;
ArucoMarker m2Marker0(vector<int>({5,6,8}), RS_CM, RS_Dist);    //rs
ArucoMarker m2Marker1(vector<int>({4}), arm_CM, arm_Dist);      //arm
ArucoMarker m2Marker2(vector<int>({4}), upper_CM, upper_Dist);  //up
RsVideoCapture camera_rs;


int main(int argc, char* argv[])
{
    UsbCAN canII;
    VCI_CAN_OBJ canFrame;
    if(canII.initCAN(UsbCAN::BAUDRATE_250K))
    {
        cout<<"init successfully\n"
            <<"transmitting..."<<endl;
    }

    //create camera thread
    pthread_t cameraThread;
    pthread_create(&cameraThread, NULL, camera_thread, NULL);

    //wifi communication
    Wifi c_wifi("192.168.11.103",1234);

    //Arm1 initialize
    vector<int> resetValues {150,50,150,125,250,15,100};
    vector<int> stampValues {150,242,100,125,128,15,100};
    
    evenVelMove(resetValues,canII,0);
    //------------------ begin algrithm -----------------------

    while(1)
    {
        //wait for stamp order
        while(!c_wifi.recvNewMSG());

        if(c_wifi.message==Wifi::MSG_FINISH_ALL_JOB)
            break;
        else if(c_wifi.message==Wifi::MSG_TARGET_IN_POSITION)
        {
            evenVelMove(stampValues,canII,0);
            usleep(500*1000);

            evenVelMove(resetValues,canII,0);
            c_wifi.sendMsg(Wifi::MSG_FINISH_STAMPING);
        }
    }

    //wait...
    cout<<"\n~~~ I FINISH MY JOB ~~~\n-- press 'q' to exit --\n";
    pthread_join(cameraThread, NULL);
    
    return 0;
}



//camera thread for func `moveInRoute`
static void* camera_thread(void* data)
{
    
    Mat img0, img1, img2;
    //VideoCapture camera1(0);    //arm camera
    VideoCapture camera2(0);    //upper camera

    #ifndef _NO_VIDEO_MODE_
    namedWindow("view_front", WINDOW_AUTOSIZE);
    //namedWindow("view_arm", WINDOW_AUTOSIZE);
    namedWindow("view_up", WINDOW_AUTOSIZE);
    #endif

    while((char)waitKey(20) != 'q')
    {
        camera_rs >> img0;
        //camera1 >> img1;
        camera2 >> img2;
        m2Marker0.detect(img0);
        //m2Marker1.detect(img1);
        m2Marker2.detect(img2);

        #ifndef _NO_VIDEO_MODE_
        m2Marker0.outputOffset(img0,Point(30,30));
        //m2Marker1.outputOffset(img1,Point(30,30));
        m2Marker2.outputOffset(img2,Point(30,30));
        imshow("view_front",img0);
        //imshow("view_arm",img1);
        imshow("view_up",img2);
        #endif

    }

    pthread_exit(0);
}
