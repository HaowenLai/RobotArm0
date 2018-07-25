/* ********************************************
 *   This program can let user manipulate the
 * robotic arm manually. You can refer to the 
 * help message for specific key method.
 * *******************************************/
#include "parameters.hpp"
#include "UsbCAN.hpp"
#include "control.hpp"
#include "position.hpp"
#include "RsVideoCapture.hpp"
#include <stdio.h>
#include <iostream>

using namespace std;
using namespace cv;

//help message
static inline void helpMsg()
{
    cout << "help messages for this program\n"
            "key '1' : select motor 1\n"
            "key '2' : select motor 2\n"
            "key '3' : select motor 3\n"
            "key '4' : select motor 4\n"
            "key '5' : select motor 5\n"
            "key '6' : select motor 6\n"
            "key '7' : select motor 7\n"
            "key ',' : increase value\n"
            "key '.' : decrease value\n"
            "key 'r' : reset to initial position\n"
            "key 'z' : decrease speed(step)\n"
            "key 'x' : increase speed(step)\n"
            "---press <Enter> to continue---";
    cin.get();
}

static void onMouse( int event, int x, int y, int, void* window_name )
{
	if( event == cv::EVENT_LBUTTONDOWN )
	{
		cout<<x<<","<<y<<endl;
	}
}


int main()
{
    using namespace robot_arm::cameraParams;

    ArucoMarker rsMarker(vector<int>({6}), RS_CM, RS_Dist);
    ArucoMarker upperMarker(vector<int>({4}), upper_CM, upper_Dist);
    CubePosition armCube(robot_arm::cubePos::armArea,
                         1600,
                         500);  //arm



    VideoCapture armCamera(3);   //arm camera
    RsVideoCapture rsCamera;
    /*if(!upperCamera.isOpened())
    {
        cout<<"cannot open camera"<<endl;
        return -1;
    }*/

    UsbCAN canII;
    if(canII.initCAN(UsbCAN::BAUDRATE_250K))
    {
        cout<<"CAN init successfully"<<endl;
    }

    Mat rsImg,armImg,upperImg;
    helpMsg();
    namedWindow("front",WINDOW_AUTOSIZE);
    //namedWindow("upper",WINDOW_AUTOSIZE);
    namedWindow("arm",WINDOW_AUTOSIZE);
    cv::setMouseCallback( "arm", onMouse, NULL );
    
    //Arm1 init
    vector<int> newValue1;
    reset2initPos(newValue1,canII,0);

    //control logic variables
    int motorNum = 0;
    int pwmValue = 128;
    int step = 1;
    
    while(1)
    {
        rsCamera    >> rsImg;
        //upperCamera >> upperImg;
        armCamera   >> armImg;
        
        rsMarker.detect(rsImg);
        rsMarker.outputOffset(rsImg,Point(30,30));
        //upperMarker.detect(upperImg);
        //upperMarker.outputOffset(upperImg,Point(30,30));
        armCube.detect(armImg);
        armCube.drawBoundry(armImg);
        
        imshow("front",rsImg);
        //imshow("upper",upperImg);
        imshow("arm",armImg);

        switch ((char)waitKey(30))
        {
          case '1':
            motorNum = 0;break;
          case '2':
            motorNum = 1;break;
          case '3':
            motorNum = 2;break;
          case '4':
            motorNum = 3;break;
          case '5':
            motorNum = 4;break;
          case '6':
            motorNum = 5;break;
          case '7':
            motorNum = 6;break;
          case ',':
            pwmValue = (pwmValue==255)?255:pwmValue+step;
            newValue1[motorNum] = pwmValue;
            fixStepMove(newValue1,canII,0);
            for(int i=0;i<7;i++)
                cout<<newValue1[i] << " ";
            cout<<endl;
            break;
          case '.':
            pwmValue = (pwmValue==0)?0:pwmValue-step;
            newValue1[motorNum] = pwmValue;
            fixStepMove(newValue1,canII,0);
            for(int i=0;i<7;i++)
                cout<<newValue1[i] << " ";
            cout<<endl;
            break;
          case 'r':
            reset2initPos(newValue1,canII,0);
            break;
          case 'z':
            step=(step==1)?1:step-1;
            cout<<"step is"<<step<<endl;
            break;
          case 'x':
            step++;
            cout<<"step is"<<step<<endl;
            break;
          
          default:
            break;
        }//end switch

        pwmValue = newValue1[motorNum];
    }//end while



    return 0;
}
