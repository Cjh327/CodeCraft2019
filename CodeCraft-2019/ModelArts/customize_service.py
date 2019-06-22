# -*- coding: utf-8 -*-

import mxnet as mx
import os
import numpy as np
import cv2
from mxnet.io import DataBatch
from mms.model_service.mxnet_model_service import MXNetBaseService
from mms.utils.mxnet import image

plate_len = 9
chinese_chars = list("深秦京海成南杭苏松")
digit_chars = [str(i) for i in range(10)]
letter_chars = list("ABCDEFGHJKLMNPQRSTUVWXYZ")
digit_char_pos = len(chinese_chars)
letter_char_pos = digit_char_pos + len(digit_chars)
chars = list()
chars.extend(chinese_chars)
chars.extend(digit_chars)
chars.extend(letter_chars)
index = dict([(chars[i], i) for i in range(len(chars))])


class DLSMXNetBaseService(MXNetBaseService):
    def __init__(self, model_name, model_dir, manifest, gpu=None):
        print("init")
        self.model_name = model_name
        self.ctx = mx.gpu(int(gpu)) if gpu is not None else mx.cpu()
        self._signature = manifest['Model']['Signature']
        param_filename = manifest['Model']['Parameters']
        epoch = 100 # 知道模型文件后几位编号为15，代码中强行指定为15
        batch_size = 1
        # model_dir模型所在文件夹，model_name模型名称
        model_path = os.path.join(model_dir, self.model_name)
        # 获取模型文件
        network, arg_params, __ = mx.model.load_checkpoint(model_path, epoch)
        data_shape = ("data", (batch_size, 3, 30, 120))
        self._signature['inputs'] = [{'data_name': 'images', 'data_shape': data_shape}]
        label_shape = ("softmax_label", (9,))
        input_shapes = dict([data_shape, label_shape])
        self.executor = network.simple_bind(ctx=mx.cpu(), **input_shapes)
        for key in self.executor.arg_dict.keys():
            if key in arg_params:
                arg_params[key].copyto(self.executor.arg_dict[key])



        for key in self.executor.arg_dict.keys():
            if key in arg_params:
                arg_params[key].copyto(self.executor.arg_dict[key])

    def _preprocess(self, data):
        # data为文件流，单张图片使用data[0]
        print("decode")

        img = mx.img.imdecode(data[0], flag=1, to_rgb=False)
        img = img.asnumpy()
        img = cv2.resize(img, (292, 72))
        img = cv2.resize(img, (120, 30))
        img = mx.nd.array(img)
        img = mx.nd.transpose(img, (2, 0, 1))
        print(img.shape)
        return img

    def _postprocess(self, probs):
        line = ''
        for i in range(probs.shape[0]):
            if i == 0:
                result = np.argmax(probs[i][0:digit_char_pos])
            elif i == 1:
                result = np.argmax(probs[i][letter_char_pos:]) + letter_char_pos
            else:  # if i > 1:
                result = np.argmax(probs[i][digit_char_pos:]) + digit_char_pos

            line += str(chars[result])
        ggg = list(line)
        ggg[0] = str(chinese_chars.index(ggg[0]))
        return "".join(ggg)

    def _inference(self, img):
        self.executor.forward(is_train=True, data=mx.nd.stack(*[img]))
        print("return probs")
        probs = self.executor.outputs[0].asnumpy()
        return probs

    def ping(self):
        '''Ping to get system's health.

        Returns
        -------
        String
            MXNet version to show system is healthy.
        '''
        return mx.__version__

    @property
    def signature(self):
        '''Signiture for model service.

        Returns
        -------
        Dict
            Model service signiture.
        '''
        return self._signature
