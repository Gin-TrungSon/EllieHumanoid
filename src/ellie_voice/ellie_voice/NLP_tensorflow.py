import warnings
import pickle
import json
import tensorflow as tf
import tflearn
import random
import os
import numpy as np
from ament_index_python.packages import get_package_share_directory
import nltk
from nltk.stem.lancaster import LancasterStemmer
from tflearn.layers.core import activation
stemmer = LancasterStemmer()
#nltk.download('punkt')
EPOCHS = 500
warnings.filterwarnings("ignore")

INTENTS_PATH = os.path.join(os.path.dirname(__file__), "intents/intents.json")
WEIGHT_PATH = os.path.join(os.path.dirname(__file__),"weights/ellie.dts")
IGNORE_WORDS = ["!", "?"]
TRAINED_DATA_PATH = os.path.join(os.path.dirname(__file__),"data/data.txt")


def training():

    if not os.path.exists(INTENTS_PATH):
        print("Intents path does not exist ...")
        exit()
    with open(INTENTS_PATH, encoding="utf-8") as json_data:
        intents = json.load(json_data)

    words = []
    classes = []
    documents = []

    for intent in intents["intents"]:
        for pattern in intent["patterns"]:
            w = nltk.word_tokenize(pattern, language="german")
            words.extend(w)
            documents.append((w, intent["tag"]))

            if intent["tag"] not in classes:
                classes.append(intent["tag"])

    words = [stemmer.stem(w.lower()) for w in words if w not in IGNORE_WORDS]
    words = sorted(set(words))
    print(classes)

    training = []
    output = []
    output_empty = [0] * len(classes)

    for doc in documents:
        bag = []
        pattern_words = doc[0]
        pattern_words = [stemmer.stem(word.lower()) for word in pattern_words]

        for w in words:
            if w in pattern_words:
                bag.append(1)
            else:
                bag.append(0)

        output = list(output_empty)
        output[classes.index(doc[1])] = 1

        training.append([bag, output])

    random.shuffle(training)
    training = np.array(training)

    train_x = list(training[:, 0])
    train_y = list(training[:, 1])

    tf.compat.v1.reset_default_graph()

    tflearn.init_graph(num_cores=8)

    net = tflearn.input_data(shape=[None, len(train_x[0])])
    net = tflearn.fully_connected(net, 8)
    net = tflearn.fully_connected(net, 16)
    net = tflearn.dropout(net, 0.5)
    net = tflearn.fully_connected(net, len(train_y[0]), activation="softmax")
    net = tflearn.regression(net)

    #model = tflearn.DNN(net, tensorboard_dir="tflearn_logs")
    model = tflearn.DNN(net)
    model.fit(train_x, train_y, n_epoch=EPOCHS, batch_size=8, show_metric=True)
    model.save(WEIGHT_PATH)
    pickle.dump({'words': words, 'classes': classes, 'train_x': train_x,
                'train_y': train_y}, open(TRAINED_DATA_PATH, "wb"))


def load_model():
    # def load_model():
    data = pickle.load(open(TRAINED_DATA_PATH, "rb"))
    words = list(data["words"])
    classes = list(data["classes"])
    train_x = data["train_x"]
    train_y = data["train_y"]

    with open(INTENTS_PATH, encoding="utf-8") as json_data:
        intents = json.load(json_data)

    net = tflearn.input_data(shape=[None, len(train_x[0])])
    net = tflearn.fully_connected(net, 8)
    net = tflearn.fully_connected(net, 16)
    net = tflearn.dropout(net, 0.5)
    net = tflearn.fully_connected(net, len(train_y[0]), activation="softmax")
    net = tflearn.regression(net)

    model = tflearn.DNN(net)
    model.load(WEIGHT_PATH)

    return model, words, classes, intents


class Inference():
    def __init__(self, threshold=0.2):
        model, words, classes, intents = load_model()
        self.model = model
        self.words = words
        self.classes = classes
        self.threshold = threshold
        self.intents = intents
        self.context = {}

    def classify(self, sentence):
        results = self.model.predict([self.bow(sentence)])[0]
        results = [[i, r] for i, r in enumerate(results) if r > self.threshold]
        results.sort(key=lambda x: x[1], reverse=True)
        return_list = []
        for r in results:
            return_list.append((self.classes[r[0]], r[1]))

        return return_list

    def response(self, sentence, userID='friend'):
        results = self.classify(sentence)
        if results:
            while results:
                for i in self.intents["intents"]:
                    if i["tag"] == results[0][0]:
                        print(results[0][1])
                        if"context_set" in i:
                            if i["context_set"] != "":
                                self.context[userID] = i["context_set"]

                        # check if this intent is contextual and applies to this user's conversation
                        if not "context_filter" in i or (userID in self.context and 'context_filter' in i and i['context_filter'] == self.context[userID]):
                            res = random.choice(i["responses"])
                            if res is None:
                                res = "Ich kann nicht verstehen, könnten Sie bitte wiederholen"
                            return res
                results.pop(0)
            return "Ich kann nicht verstehen, könnten Sie bitte wiederholen"

    def clean_up_sentence(self, sentence):
        sentence_words = nltk.word_tokenize(sentence, language="german")
        sentence_words = [stemmer.stem(
            word.lower()) for word in sentence_words if word not in IGNORE_WORDS]
        return sentence_words

    def bow(self, sentence):
        sentence_words = self.clean_up_sentence(sentence)
        bag = [0]*len(self.words)
        for s in sentence_words:
            for i, w in enumerate(self.words):
                if w == s:
                    bag[i] = 1
        return np.array(bag)


if __name__ == "__main__":
    # training()
    infer = Inference()
    while True:
        input_data = input("You- ")
        print(input_data)
        answer = infer.response(input_data)
        print(answer)
