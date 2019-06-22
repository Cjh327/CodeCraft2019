# coding=utf-8
import argparse
import os
import sys

import cv2
import mxnet as mx
import numpy as np
import logging

import moxing as mox
mox.file.shift('os', 'mox')
sys.path.insert(0, "../../python")
sys.setrecursionlimit(1000000)

def get_all_data(height,width,is_train,data_path,label_path):

    # chinese_dict = {'深':0,'秦':1,'京':2,'海':3,'成':4,'南':5,'杭':6,'苏':7,'松':8}
    char_index = {'深':0,"秦":1,"京":2,"海":3,"成":4,"南":5,"杭":6,"苏":7,"松":8,
             "0": 9, "1": 10, "2": 11, "3": 12, "4": 13, "5": 14,"6": 15, "7": 16, "8": 17, "9": 18,
             "A": 19, "B": 20, "C": 21, "D": 22, "E": 23, "F": 24, "G": 25, "H": 26,"J": 27, "K": 28,
             "L": 29, "M": 30, "N": 31, "P": 32, "Q": 33, "R": 34, "S": 35, "T": 36, "U": 37, "V": 38,
             "W": 39, "X": 40, "Y": 41, "Z": 42};
    # print char_index

    chars = ["深","秦","京","海","成","南","杭","苏","松", "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "A", "B",
             "C", "D", "E", "F", "G", "H", "J", "K", "L", "M", "N", "P", "Q", "R", "S", "T", "U", "V", "W", "X","Y", "Z"]
    all_data = []
    data = []
    labels = []
    with open(label_path,'r', encoding='UTF-8-sig') as f:
        lines = f.readlines()
    ch = set()
    for line in lines:
        new_line = line.split(',')
        file_name = new_line[1].strip()
        plate = new_line[0].strip()
        img_path = data_path + file_name
        img = cv2.imread(img_path)
        img = cv2.resize(img,(width,height))

        img = np.multiply(img, 1 / 255.0)
        img = img.transpose(2, 0, 1)

        label = []
        for _ in plate:
            ch.add(_)
            label.append(char_index[_])
        all_data.append([img,label])

    if(is_train):
        return all_data[0:3900]
    else:
        return all_data[3900:]

class Batch(object):
    def __init__(self, data_names, data, label_names, label):
        self.data = data
        self.label = label
        self.data_names = data_names
        self.label_names = label_names

    @property
    def provide_data(self):
        return [(n, x.shape) for n, x in zip(self.data_names, self.data)]

    @property
    def provide_label(self):
        return [(n, x.shape) for n, x in zip(self.label_names, self.label)]

def Accuracy(label, pred):
    label = label.T.reshape((-1, ))
    hit = 0
    total = 0
    for i in range(int(pred.shape[0] / 9)):
        ok = True
        for j in range(9):
            k = i * 9 + j
            if np.argmax(pred[k]) != int(label[k]):
                ok = False
                break
        if ok:
            hit += 1
        total += 1
    return 1.0 * hit / total

class OCRIter(mx.io.DataIter):
    def __init__(self, count, batch_size, num_label, height, width, is_train, data_path, label_path):
        super(OCRIter, self).__init__()
        self.batch_size = batch_size
        self.count = count
        self.height = height
        self.width = width
        self.is_train = is_train
        self.data_path = data_path
        self.label_path = label_path
        self.train_data = get_all_data(height,width,is_train,data_path,label_path)
        self.test_data= get_all_data(height, width,is_train,data_path,label_path)
        self.provide_data = [('data', (self.batch_size,3, height, width))]
        self.provide_label = [('softmax_label', (self.batch_size, num_label))]
        print("start")

    def __iter__(self):
        for k in range(int(self.count / self.batch_size)):
            data = []
            label = []
            for i in range(self.batch_size):
                if(self.is_train):
                    [img,num] = self.train_data[self.batch_size * k + i]
                else:
                    [img,num] = self.test_data[self.batch_size * k + i]
                data.append(img)
                label.append(num) 
            data_all = [mx.nd.array(data)]
            label_all = [mx.nd.array(label)]
            data_names = ['data']
            label_names = ['softmax_label']
            data_batch = Batch(data_names, data_all, label_names, label_all)
            yield data_batch

    def reset(self):
        pass

def get_net():
    data = mx.symbol.Variable('data')
    label = mx.symbol.Variable('softmax_label')
    conv1 = mx.symbol.Convolution(data=data, kernel=(5, 5), num_filter=32)
    pool1 = mx.symbol.Pooling(data=conv1, pool_type="max", kernel=(2, 2), stride=(1, 1))
    relu1 = mx.symbol.Activation(data=pool1, act_type="relu")

    conv2 = mx.symbol.Convolution(data=relu1, kernel=(5, 5), num_filter=32)
    pool2 = mx.symbol.Pooling(data=conv2, pool_type="max", kernel=(2, 2), stride=(1, 1))
    relu2 = mx.symbol.Activation(data=pool2, act_type="relu")

    conv3 = mx.symbol.Convolution(data=relu2, kernel=(3,3), num_filter=32)
    pool3 = mx.symbol.Pooling(data=conv3, pool_type="max", kernel=(2,2), stride=(1, 1))
    relu3 = mx.symbol.Activation(data=pool3, act_type="relu")

    conv4 = mx.symbol.Convolution(data=relu3, kernel=(3,3), num_filter=32)
    pool4 = mx.symbol.Pooling(data=conv4, pool_type="max", kernel=(2,2), stride=(1, 1))
    relu4 = mx.symbol.Activation(data=pool4, act_type="relu")

    flatten = mx.symbol.Flatten(data=relu4)
    fc1 = mx.symbol.FullyConnected(data=flatten, num_hidden=120)
    fc21 = mx.symbol.FullyConnected(data=fc1, num_hidden=43)
    fc22 = mx.symbol.FullyConnected(data=fc1, num_hidden=43)
    fc23 = mx.symbol.FullyConnected(data=fc1, num_hidden=43)
    fc24 = mx.symbol.FullyConnected(data=fc1, num_hidden=43)
    fc25 = mx.symbol.FullyConnected(data=fc1, num_hidden=43)
    fc26 = mx.symbol.FullyConnected(data=fc1, num_hidden=43)
    fc27 = mx.symbol.FullyConnected(data=fc1, num_hidden=43)
    fc28 = mx.symbol.FullyConnected(data=fc1, num_hidden=43)
    fc29 = mx.symbol.FullyConnected(data=fc1, num_hidden=43)
    fc2 = mx.symbol.Concat(*[fc21, fc22, fc23, fc24, fc25, fc26, fc27,fc28,fc29], dim=0)
    label = mx.symbol.transpose(data=label)
    label = mx.symbol.Reshape(data=label, target_shape=(0,))
    return mx.symbol.SoftmaxOutput(data=fc2, label=label, name="softmax")


def train(path_model, data_path):
    network = get_net()
    devs = [mx.gpu(i) for i in range(1)]
    #ctx=devs,
    model = mx.model.FeedForward(
                                 ctx=devs,
                                 symbol=network,
                                 num_epoch=10,
                                 learning_rate=0.001,
                                 wd=0.00001,
                                 initializer=mx.init.Xavier(factor_type="in", magnitude=2.34),
                                 momentum=0.9)
    batch_size = 20
    data_path = os.path.join(data_path, "train-data/")
    label_path = os.path.join(data_path, "train-data-label.txt")

    data_train = OCRIter(3900, batch_size, 9, 30, 120, True, data_path, label_path)

    # 像操作本地数据一样进行数据的操作，使用os.path.join定位OBS中的文件夹路径
    data_test = OCRIter(100, batch_size, 9, 30, 120, False, data_path, label_path)

    
    head = '%(asctime)-15s %(message)s'
    logging.basicConfig(level=logging.DEBUG, format=head)
    model.fit(X=data_train, eval_data=data_test, eval_metric=Accuracy,
              batch_end_callback=mx.callback.Speedometer(batch_size, 50))

    # 像操作本地数据一样进行数据的操作，使用os.path.join定位OBS中的文件夹路径
    model.save(os.path.join(path_model, "obs-car-reg325"))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="train mnist",
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('--data_url', type=str, default='s3://obs-car-reg325/data/', help='the training data')

    parser.add_argument('--train_url', type=str, default='s3://obs-car-reg325/model/', help='the path model saved')
    args, unkown = parser.parse_known_args()

    mox.file.copy_parallel(src_url= args.data_url, dst_url='/cache/my_data')
    data_path = '/cache/my_data'
    train(args.train_url, data_path=data_path)