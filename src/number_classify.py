#!/usr/bin/python3

from imageai.Prediction.Custom import CustomImagePrediction
import numpy as np
import os

execution_path = os.path.join(os.path.split(os.getcwd())[0],'data')
IMAGE_HEIGHT = 50
IMAGE_WIDTH  = 50
rtVal={'4':[0],'8':[1],'blank':[2]}

# load model
prediction = CustomImagePrediction()
prediction.setModelTypeAsResNet()
prediction.setModelPath(os.path.join(execution_path, "model_ex-097_acc-0.766667.h5"))
prediction.setJsonPath(os.path.join(execution_path, "model_class.json"))
prediction.loadModel(num_objects=7)

def main(imgList):
    img = np.array(imgList,dtype=np.uint8)
    img = np.reshape(img,(IMAGE_HEIGHT,IMAGE_WIDTH,3))
    predictions, probabilities = prediction.predictImage(img,input_type='array', result_count=1)

    print('from py file: %s' % predictions[0])
    print('probability is %s' % probabilities[0])

    #merge class
    if predictions[0] == '4-0' or predictions[0] == '4-90' or predictions[0] == '4-180' or predictions[0] == '4-270':
        return tuple(rtVal['4'])
    elif predictions[0] == '8-0' or predictions[0] == '8-180':
        return tuple(rtVal['8'])
    else:
        return tuple(rtVal['blank'])
