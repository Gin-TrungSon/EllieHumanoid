import numpy as np
import os

import tensorflow as tf
assert tf.__version__.startswith('2')

from tflite_model_maker import configs
from tflite_model_maker import ExportFormat
from tflite_model_maker import model_spec
from tflite_model_maker import question_answer
from tflite_model_maker import QuestionAnswerDataLoader
import tflite_runtime.interpreter as tflite
import numpy as np
import tensorflow_hub as hub
import tensorflow_text as text
from transformers import MobileBertTokenizer
from transformers import BertForQuestionAnswering



interpreter = tflite.Interpreter(model_path="src/voice/weights/lite-model_mobilebert_1_metadata_1.tflite")
interpreter.allocate_tensors()


# Get input and output tensors.
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Test model on random input data.
tokenzier = MobileBertTokenizer.from_pretrained("bert-base-uncased")
feature = tokenzier.encode_plus("this a good movie!")

input_ids = feature["input_ids"]
input_mask = feature["attention_mask"]
segment_ids = feature["token_type_ids"]

input_ids = np.array(input_ids, dtype=np.int32)
input_mask = np.array(input_mask, dtype=np.int32)
segment_ids = np.array(segment_ids, dtype=np.int32)

print(input_ids.shape)
exit()
interpreter.set_tensor(input_details[0]["index"], input_ids)
interpreter.set_tensor(input_details[1]["index"], input_mask)
interpreter.set_tensor(input_details[2]["index"], segment_ids)
interpreter.invoke()

# Get output logits.
end_logits = interpreter.get_tensor(output_details[0]["index"])[0]
start_logits = interpreter.get_tensor(output_details[1]["index"])[0]

print(start_logits)
print(end_logits)