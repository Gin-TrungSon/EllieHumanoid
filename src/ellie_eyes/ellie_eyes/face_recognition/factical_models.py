import os
# we are using the model already trained
#https://github.com/davisking/dlib-models

def pose_predictor_model_location():
    return os.path.join(os.path.dirname(__file__), "models/shape_predictor_68_face_landmarks.dat")

def pose_predictor_five_point_model_location():
    return os.path.join(os.path.dirname(__file__), "models/shape_predictor_5_face_landmarks.dat")

def face_recognition_model_location():
    return os.path.join(os.path.dirname(__file__), "models/dlib_face_recognition_resnet_model_v1.dat")

def cnn_face_detector_model_location():
    return os.path.join(os.path.dirname(__file__), "models/mmod_human_face_detector.dat")

if __name__=="__main__":
    pose_predictor_model_location()
    pose_predictor_five_point_model_location()
    face_recognition_model_location()
    cnn_face_detector_model_location()