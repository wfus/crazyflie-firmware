import tensorflow as tf

lite_model_name = 'lite_model_file.tflite'
converter = tf.contrib.lite.TFLiteConverter.from_keras_model_file('test_mnist.h5')
converter.post_training_quantize=True
tf_lite_model = converter.convert()
with open(lite_model_name, "wb") as f:
    f.write(tf_lite_model)