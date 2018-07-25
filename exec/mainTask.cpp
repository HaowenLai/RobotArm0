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
 * @Date   : 2018/7/19
 * ************************************************************/

#include "parameters.hpp"
#include "UsbCAN.hpp"
#include "position.hpp"
#include "RsVideoCapture.hpp"
#include "NumberClassify.hpp"
#include "Wifi.hpp"
#include "control.hpp"

#include <pthread.h>
#include <stdio.h>
#include <iostream>

using namespace std;
using namespace cv;

// #define _SINGLE_MOVE_MODE_
// #define _NO_VIDEO_MODE_

static void* camera_thread(void* data);

using namespace robot_arm::cameraParams;
CubePosition armCube(robot_arm::cubePos::armArea,
                     robot_arm::cubePos::armValidLenMax,
                     robot_arm::cubePos::armValidLenMin);  //arm

//global variables, to communicate with camera thread
auto ckeckSurfaceFlag = robot_arm::MISSION_OK;
Mat detectNumImg;
Mat detectBeforeResize;

int main(int argc, char* argv[])
{
    UsbCAN canII;
    if(canII.initCAN(UsbCAN::BAUDRATE_250K))
    {
        cout<<"init successfully\n"
            <<"transmitting..."<<endl;
    }

    //create camera thread
    pthread_t cameraThread;
    pthread_create(&cameraThread, NULL, camera_thread, NULL);

    //create wifi client
    Wifi c_wifi("192.168.100.100",1234);
    const int msgLength = 50*50*3;
    unsigned char msg_buff[msgLength];

    //number classify network
    char programName[100];
    getcwd(programName,100);    //get program path
    string modulePath(programName);
    modulePath = modulePath.substr(0,modulePath.find_last_of('/'))+"/src";
    string moduleName = "number_classify";
    string funcName   = "main";
    NumberClassify network(modulePath,moduleName,funcName);

    //Arm1 initialize
    vector<int> motorValues;
    reset2initPos(motorValues,canII,0);
    cout<<"program is ready..Press <ENTER> to continue"<<endl;
    cin.get();
    
    //------------------ begin algrithm -----------------------
again:
    //white arm manipulating
    while(1)
    {
        while(!c_wifi.recvNewMSG());
        if(c_wifi.message == Wifi::MSG_TARGET_IN_POSITION ||
           c_wifi.message == Wifi::MSG_TARGET_QUALIFIED   ||
           c_wifi.message == Wifi::MSG_TARGET_UNQUALIFIED)
        {
            break;
        }
        else if(c_wifi.message == Wifi::MSG_PREPARE_TO_DETECT)
        {
            c_wifi.sendMsg(Wifi::MSG_ROGOR);

            while(!c_wifi.recvNewMSG(msg_buff,msgLength));
            Mat img(50,50,CV_8UC3,msg_buff);
            // imshow("detect",img);

            switch(network.detect(img))
            {
            case NumberClassify::NUMBER_4:
                c_wifi.sendMsg(Wifi::MSG_TARGET_QUALIFIED);
                cout<<"\n\n4\n\n";
                break;
            case NumberClassify::NUMBER_8:
                c_wifi.sendMsg(Wifi::MSG_TARGET_UNQUALIFIED);
                cout<<"\n\n8\n\n";
                break;
            case NumberClassify::NUMBER_BLANK:
                c_wifi.sendMsg(Wifi::MSG_TARGET_BLANK);
                cout<<"\n\nblank\n\n";
                break;
            }
        }  //end else if
    }//end while(1)


    //move to passing area
    motorValues = vector<int> {150,209,114,125,250,135,18};
    evenVelMove(motorValues,canII,0);
    cout<<"move to passing area"<<endl;
    usleep(1000*1000);
    
    //adjust motor #4(prepare for dig-into step)
    while(true)
    {
        const float epsilon = 5.0f;
        const float centerX = 268.0f;

        if(!armCube.isNewFrame())
            continue;

        if(armCube.cubeExist())
        {
            float targetX = armCube.getPosition().x;
            
            if(targetX - centerX > epsilon)
            {
                motorValues[3] += 1;
                fixStepMove(motorValues,canII,0);
            }
            else if(targetX - centerX < -epsilon)
            {
                motorValues[3] -= 1;
                fixStepMove(motorValues,canII,0);
            }
            else
            {
                cout<<"#4 is in position ~\n";
                break;
            }
        }//end if(m2Marker1.index(4) != -1)
        usleep(200*1000);
    }
    cout<<"adjust motor #4(prepare for dig-into step)"<<endl;

    //dig into the cube
    motorValues[1] = 229;
    motorValues[2] = 95;
    evenVelMove(motorValues,canII,0);
    usleep(200*1000);
    cout<<"dig into the cube"<<endl;

    //grab the cube
    motorValues[6] = 190;
    fixStepMove(motorValues,canII,0);
    usleep(200*1000);
    cout<<"grab the cube"<<endl;

    //move to the air
    motorValues[1] = 110;
    motorValues[2] = 142;
    evenVelMove(motorValues,canII,0);
    usleep(200*1000);

    //deal with upper detect result
    if(c_wifi.message == Wifi::MSG_TARGET_QUALIFIED)
    {
        motorValues = vector<int> {95,197,146,125,250,135,190};
        evenVelMove(motorValues,canII,0);
        usleep(200*1000);

        //loose the clip
        motorValues[6] = 18;
        fixStepMove(motorValues,canII,0);

    }
    else if(c_wifi.message == Wifi::MSG_TARGET_UNQUALIFIED)
    {
        motorValues = vector<int> {205,197,146,125,250,135,190};
        evenVelMove(motorValues,canII,0);
        usleep(200*1000);

        //loose the clip
        motorValues[6] = 18;
        fixStepMove(motorValues,canII,0);
    }
    else //check left and right surface
    {
        //left surface
        getDetectImg(robot_arm::CHECK_LEFT_SURFACE,
                 ckeckSurfaceFlag,canII,0);
        switch(network.detect(detectNumImg))
        {
            case NumberClassify::NUMBER_4:
                motorValues = vector<int> {95,197,146,125,250,135,190};
                evenVelMove(motorValues,canII,0);
                usleep(200*1000);

                //loose the clip
                motorValues[6] = 18;
                fixStepMove(motorValues,canII,0);
                break;
            case NumberClassify::NUMBER_8:
                motorValues = vector<int> {205,197,146,125,250,135,190};
                evenVelMove(motorValues,canII,0);
                usleep(200*1000);

                //loose the clip
                motorValues[6] = 18;
                fixStepMove(motorValues,canII,0);
                break;
            case NumberClassify::NUMBER_BLANK:
                getDetectImg(robot_arm::CHECK_RIGHT_SURFACE,
                  ckeckSurfaceFlag,canII,0);
                
                //check right surface
                switch(network.detect(detectNumImg))
                {
                case NumberClassify::NUMBER_4:
                    motorValues = vector<int> {95,197,146,125,250,135,190};
                    evenVelMove(motorValues,canII,0);
                    usleep(200*1000);

                    //loose the clip
                    motorValues[6] = 18;
                    fixStepMove(motorValues,canII,0);
                    break;
                case NumberClassify::NUMBER_8:
                case NumberClassify::NUMBER_BLANK:
                    motorValues = vector<int> {205,197,146,125,250,135,190};
                    evenVelMove(motorValues,canII,0);
                    usleep(200*1000);

                    //loose the clip
                    motorValues[6] = 18;
                    fixStepMove(motorValues,canII,0);
                    break;
                }
                break;
        }
    }//end else check left and right surface


    reset2initPos(motorValues,canII,0);
    c_wifi.sendMsg(Wifi::MSG_FINISH_A_JOB);
    while(!c_wifi.recvNewMSG());
    if(c_wifi.message == Wifi::MSG_START_A_NEW_JOB)
    {
        goto again;
    }
    else
    {
        //wait...
        cout<<"\n~~~ I FINISH MY JOB ~~~\n-- press 'q' to exit --\n";
        pthread_join(cameraThread, NULL);
    }

    return 0;
}



//camera thread for func `moveInRoute`
static void* camera_thread(void* data)
{
    //number detection area
    const Rect leftArea(Point(323,208),Point(366,263));
    const Rect rightArea(Point(229,118),Point(277,154));
    
    Mat rsImg,armImg;
    RsVideoCapture camera_rs;
    VideoCapture armCamera(3);    //upper camera

    
    #ifndef _NO_VIDEO_MODE_
    namedWindow("view_arm", WINDOW_AUTOSIZE);
    namedWindow("view_front",WINDOW_AUTOSIZE);
    #endif

    while((char)waitKey(30) != 'q')
    {
        camera_rs >> rsImg;
        armCamera >> armImg;
        
        armCube.detect(armImg);
        armCube.drawBoundry(armImg);

        #ifndef _NO_VIDEO_MODE_
        imshow("view_arm",armImg);
        imshow("view_front",rsImg);
        #endif

        //take detect roi
        if(ckeckSurfaceFlag == robot_arm::CHECK_LEFT_SURFACE)
        {
            detectNumImg = rsImg(leftArea).clone();
            imshow("detect",detectNumImg);
            ckeckSurfaceFlag = robot_arm::MISSION_OK;
        }
        else if(ckeckSurfaceFlag == robot_arm::CHECK_RIGHT_SURFACE)
        {
            detectNumImg = rsImg(rightArea).clone();
            imshow("detect",detectNumImg);
            ckeckSurfaceFlag = robot_arm::MISSION_OK;
        }
    }

    pthread_exit(0);
}
