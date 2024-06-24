#!/usr/bin/env python2
# coding:utf-8 
from __future__ import division
from __future__ import with_statement
from __future__ import absolute_import
import time
import cv2
import PIL
import torch
import torchvision
import numpy as np
from reid.data import make_data_loader
from reid.data.transforms import build_transforms
from reid.modeling import build_model
from reid.config import cfg as reidCfg

from sensor_msgs.msg import Image
from cv_bridge import CvBridge #, CvBridgeError
import rospy
# from kcf_follower.msg import person
from message_filters import TimeSynchronizer, Subscriber
from kcf_follower.srv import *

import  os
print os.getcwd() #获取当前工作目录路径

# Initialize
global device,query_loader,num_query,reidModel,query_feats,device,im0
device = torch.device(u'cuda:0' if torch.cuda.is_available() else u'cpu')
torch.backends.cudnn.benchmark = False  # set False for reproducible results

############# 行人重识别模型初始化 #############

def get_query(data):
    global device,query_loader,num_query,reidModel,query_feats
    query_loader, num_query = make_data_loader(reidCfg)
    reidModel = build_model(reidCfg, num_classes=10126)
    reidModel.load_param(reidCfg.TEST.WEIGHT)
    reidModel.to(device).eval()
    query_feats = []
    query_pids  = []
    with torch.no_grad():
        for i, batch in enumerate(query_loader):
            img, pid, camid = batch
            img = img.to(device)
            feat = reidModel(img)         # 一共2张待查询图片，每张图片特征向量2048 torch.Size([2, 2048])
            query_feats.append(feat)
            query_pids.extend(np.asarray(pid))  # extend() 函数用于在列表末尾一次性追加另一个序列中的多个值（用新列表扩展原来的列表）。

        query_feats = torch.cat(query_feats, dim=0)  # torch.Size([2, 2048])
        query_feats = torch.nn.functional.normalize(query_feats, dim=1, p=2) # 计算出查询图片的特征向量
    rospy.loginfo("Ready to handle the init_person_recognition request！")
    return True

def person_recognize(data):
    global device,query_loader,num_query,reidModel,im0,query_feats
    dist_thres=1.0
    im0 = CvBridge().imgmsg_to_cv2(data.rgb_image, "bgr8")
    gallery_img = []
    gallery_loc = []
    gallery_loc.append((data.xMin, data.yMin, data.xMin+data.width, data.yMin+data.height))
    crop_img = im0[data.yMin:data.yMin+data.height, data.xMin:data.xMin+data.width] 
    crop_img = PIL.Image.fromarray(cv2.cvtColor(crop_img, cv2.COLOR_BGR2RGB))  # PIL: (233, 602)
    crop_img = build_transforms(reidCfg)(crop_img).unsqueeze(0)  # torch.Size([1, 3, 256, 128])
    gallery_img.append(crop_img)

    with torch.no_grad():
        gallery_img = torch.cat(gallery_img, dim=0)  # torch.Size([7, 3, 256, 128])
        gallery_img = gallery_img.to(device)
        gallery_feats = reidModel(gallery_img) # torch.Size([7, 2048])
        gallery_feats = torch.nn.functional.normalize(gallery_feats, dim=1, p=2)  # 计算出查询图片的特征向量

        m, n = query_feats.shape[0], gallery_feats.shape[0]
        distmat = torch.pow(query_feats, 2).sum(dim=1, keepdim=True).expand(m, n) + \
                    torch.pow(gallery_feats, 2).sum(dim=1, keepdim=True).expand(n, m).t()
        distmat.addmm_(1, -2, query_feats, gallery_feats.t())
        distmat = distmat.cpu().numpy()  # <class 'tuple'>: (3, 12)
        distmat = distmat.sum(axis=0) / len(query_feats) # 平均一下query中同一行人的多个结果
        index = distmat.argmin()
        print u'距离：%s'%distmat[index]
        if distmat[index] < dist_thres:
            return True
        else:
            return False
            # cv2.rectangle(im0, (gallery_loc[0][0], gallery_loc[0][1]), (gallery_loc[0][2], gallery_loc[0][3]), (0, 255, 0), 2)
        # cv2.imshow('person search', im0)
        # cv2.waitKey()

    
if __name__ == u'__main__':

    rospy.init_node('REID', anonymous=True)
    rospy.Service("init_person_recognition", init_Recognize, get_query)
    rospy.Service("person_recognition", Recognize, person_recognize)

    rospy.spin()
