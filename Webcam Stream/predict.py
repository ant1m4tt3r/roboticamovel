# encoding: UTF-8
import tensorflow as tf
import numpy as np

from node_lookup import NodeLookup

class Predictor(object):

    @staticmethod
    def predict(image_name):
        MAX_PREDICTIONS = 5
        # Unpersists graph from file
        with tf.gfile.FastGFile('tensor_flow_data/classify_image_graph_def.pb', 'rb') as fin:
            graph_def = tf.GraphDef()
            graph_def.ParseFromString(fin.read())
            _ = tf.import_graph_def(graph_def, name='')

        with tf.Session() as sess:
            # And capture continuously forever.
            softmax_tensor = sess.graph.get_tensor_by_name('softmax:0')
            image_data = tf.gfile.FastGFile(image_name, 'rb').read()

            # Make the prediction. Big thanks to this SO answer:
            # http://stackoverflow.com/questions/34484148/feeding-image-data-in-tensorflow-for-transfer-learning
            predictions = sess.run(
                softmax_tensor, {'DecodeJpeg/contents:0': image_data})
            predictions = np.squeeze(predictions)

            # Creates node ID --> English string lookup.
            node_lookup = NodeLookup()

            top_k = predictions.argsort()[-MAX_PREDICTIONS:][::-1]
            response = []

            for node_id in top_k:
                human_string = node_lookup.id_to_string(node_id)
                score = predictions[node_id]
                # É necessário converter o score para um tipo nativo do python
                response.append(
                    {"string": human_string, "score": np.asscalar(score)})

            return response
