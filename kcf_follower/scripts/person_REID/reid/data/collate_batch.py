# encoding: utf-8
u"""
@author:  liaoxingyu
@contact: sherlockliao01@gmail.com
"""

from __future__ import absolute_import
import torch
from itertools import izip


def train_collate_fn(batch):
    imgs, pids, _, _, = izip(*batch)
    pids = torch.tensor(pids, dtype=torch.int64)
    return torch.stack(imgs, dim=0), pids


def val_collate_fn(batch):
    imgs, pids, camids, _ = izip(*batch)
    return torch.stack(imgs, dim=0), pids, camids
