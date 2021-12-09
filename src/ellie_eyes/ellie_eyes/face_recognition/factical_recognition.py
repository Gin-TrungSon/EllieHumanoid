# region import
import sys
sys.path.append("")
import os
import dlib
from PIL import ImageFile
import numpy as np
import PIL.Image
import cv2.cv2 as cv2
import time
import pickle


try:
    import ellie_eyes.face_recognition.factical_models as factical_models
except Exception:
    print("Missing models")
    quit()
# endregion


# region define models
start = time.time()
ImageFile.LOAD_TRUNCATED_IMAGES = True

face_detector = dlib.get_frontal_face_detector()

predictor_68_point_model = factical_models.pose_predictor_model_location()
pose_predictor_68_point = dlib.shape_predictor(predictor_68_point_model)

predictor_5_point_model = factical_models.pose_predictor_five_point_model_location()
pose_predictor_5_point = dlib.shape_predictor(predictor_5_point_model)

cnn_face_detection_model = factical_models.cnn_face_detector_model_location()
cnn_face_detector = dlib.cnn_face_detection_model_v1(cnn_face_detection_model)

face_recognition_model = factical_models.face_recognition_model_location()
face_encoder = dlib.face_recognition_model_v1(face_recognition_model)
initiate_time = time.time()-start
print("Initiate time : {:.4f}".format(initiate_time))
# endregion


# region private functions
def _rect_to_corners(rect):
    """
    Convert a dlib 'rect' object to a plain tuple in (top, right, bottom, left)
    """
    return rect.top(), rect.right(), rect.bottom(), rect.left()


def _corners_to_rect(corners):
    """
    convert a tuple in (top, right, bottom, left) order to a dlib `rect` object
    """
    return dlib.rectangle(corners[3], corners[0], corners[1], corners[2])


def _trim_to_bounds(corners, image_shape):
    """
    corners: plain tuple representation of the rect in (top, right, bottom, left) order
    """
    return max(corners[0], 0), min(corners[1], image_shape[1]), min(corners[2], image_shape[0]), max(corners[3], 0)


def _raw_face_locations(img, number_of_times_to_upsample=1, model="hog"):
    """
    "hog"-- Histogram of Oriented Gradients-- is less accurate but faster on CPUs. "cnn" is a more accurate
        deep-learning model which is GPU/CUDA accelerated
    """
    if model == "cnn":
        return cnn_face_detector(img, number_of_times_to_upsample)
    else:
        return face_detector(img, number_of_times_to_upsample)


def _raw_face_locations_batched(images, number_of_times_to_upsample=1, batch_size=128):
    """
    Returns an 2d array of dlib rects of human faces in a image using the cnn face detector
    """
    return cnn_face_detector(images, number_of_times_to_upsample, batch_size=batch_size)


def _raw_face_landmarks(face_image, face_locations=None, model="small"):
    if face_locations is None:
        face_locations = _raw_face_locations(face_image)
    else:
        face_locations = [_corners_to_rect(
            face_location) for face_location in face_locations]

    pose_predictor = pose_predictor_68_point

    if model == "small":
        pose_predictor = pose_predictor_5_point

    return [pose_predictor(face_image, face_location) for face_location in face_locations]


def _face_distance(face_encodings, face_to_compare):
    if len(face_encodings) == 0:
        return np.empty((0))
    return np.linalg.norm(face_encodings - face_to_compare, axis=1)
# endregion

# region public funtions


def load_image(file, mode="RGB"):
    im = PIL.Image.open(file)
    if mode:
        im = im.convert(mode)
    im = cv2.cvtColor(np.array(im), cv2.COLOR_BGR2RGB)
    return im

DATA_PATH =os.path.join(os.path.dirname(__file__),"models/pre_loaded.dts")
def load_images(path, reload = False):
    """
    return encoded images in folder
    """
    if os.path.exists(DATA_PATH) and reload== False:
        data = pickle.load(open(DATA_PATH, "rb"))
        return data["faces"],data["names"]
    faces = []
    names = []
    for name in os.listdir(path):
        for filename in os.listdir(f"{path}/{name}"):
            image = load_image(f"{path}/{name}/{filename}")
            encoding = face_encodings(image)[0]
            faces.append(encoding)
            names.append(name)
    pickle.dump({'faces': faces, 'names': names}, open(DATA_PATH, "wb"))
    return faces, names


def detect_face_locations(img, number_of_times_to_upsample=1, model="hog"):
    if model == "cnn":
        return [_trim_to_bounds(_rect_to_corners(face.rect), img.shape) for face in _raw_face_locations(img, number_of_times_to_upsample, "cnn")]
    else:
        return [_trim_to_bounds(_rect_to_corners(face), img.shape) for face in _raw_face_locations(img, number_of_times_to_upsample, model)]


def batch_face_locations(images, number_of_times_to_upsample=1, batch_size=128):
    """
    Returns an 2d array of bounding boxes of human faces in a image using the cnn face detector
    """
    def convert_cnn_detections_to_css(detections):
        return [_trim_to_bounds(_rect_to_corners(face.rect), images[0].shape) for face in detections]

    raw_detections_batched = _raw_face_locations_batched(
        images, number_of_times_to_upsample, batch_size)

    return list(map(convert_cnn_detections_to_css, raw_detections_batched))


def face_landmarks(face_image, face_locations=None, model="small"):
    """
    Given an image, returns a dict of face feature locations (eyes, nose, etc) for each face in the image
    """
    landmarks = _raw_face_landmarks(face_image, face_locations, model)
    landmarks_as_tuples = [[(p.x, p.y) for p in landmark.parts()]
                           for landmark in landmarks]

    # see https://cdn-images-1.medium.com/max/1600/1*AbEg31EgkbXSQehuNJBlWg.png
    if model == 'large':
        return [{
            "chin": points[0:17],
            "left_eyebrow": points[17:22],
            "right_eyebrow": points[22:27],
            "nose_bridge": points[27:31],
            "nose_tip": points[31:36],
            "left_eye": points[36:42],
            "right_eye": points[42:48],
            "top_lip": points[48:55] + [points[64]] + [points[63]] + [points[62]] + [points[61]] + [points[60]],
            "bottom_lip": points[54:60] + [points[48]] + [points[60]] + [points[67]] + [points[66]] + [points[65]] + [points[64]]
        } for points in landmarks_as_tuples]
    elif model == 'small':
        return [{
            "nose_tip": [points[4]],
            "left_eye": points[2:4],
            "right_eye": points[0:2],
        } for points in landmarks_as_tuples]
    else:
        raise ValueError(
            "Invalid landmarks model type. Supported models are ['small', 'large'].")


def face_encodings(face_image, known_face_locations=None, num_jitters=1, model="small"):
    """
    return: A list of 128-dimensional face encodings (one for each face in the image)
    """
    raw_landmarks = _raw_face_landmarks(
        face_image, known_face_locations, model)
    return [np.array(face_encoder.compute_face_descriptor(face_image, raw_landmark_set, num_jitters)) for raw_landmark_set in raw_landmarks]


def compare_faces(known_face_encodings, face_encoding_to_check, tolerance=0.6):
    """
    Compare a list of face encodings against a candidate encoding to see if they match.
    """
    distances = _face_distance(known_face_encodings, face_encoding_to_check)
    ismatched = distances <= tolerance
    return ismatched, distances

# endregion
