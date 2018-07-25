/* ********************************************************
 *   This is a class for calling python to perform letters 
 * classification.
 * 
 * @Author : Derek Lai
 * @Date   : 2018/6/29
 * @Version: v1.0
 * Copyright(c) All right reserved
 * ********************************************************/

#ifndef __LETTERSCLASSIFY_HPP__
#define __LETTERSCLASSIFY_HPP__

#include <string>
#include <opencv2/opencv.hpp>
#include <Python.h>


class NumberClassify
{
 public:
    enum Numbers
    {
        NUMBER_4,
        NUMBER_8,
        NUMBER_BLANK,
        NO_LETTER_ERROR
    };

    NumberClassify(std::string path,
                    std::string moduleName,
                    std::string funcName);
    ~NumberClassify();
    
    //input and output can be the same variable
    Numbers detect(cv::Mat& InputImage);

  private:
    PyObject* pModule;      //module obj
    PyObject* pDict;        //fuction dict obj
    PyObject* pFunc;        //target function
    PyObject* pArgs;        //target function argument
    PyObject* pRetVal;      //return value
    PyObject* pImageData;   //image data in List type
};


#endif
