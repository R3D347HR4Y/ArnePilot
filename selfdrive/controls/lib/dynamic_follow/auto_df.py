"""
  Generated using Konverter: https://github.com/ShaneSmiskol/Konverter
"""
from pathlib import Path
import numpy as np
from common.travis_checker import travis

path = Path(__file__).parent / 'auto_df_weights.npz'

wb = np.load(path, allow_pickle=True)

# if travis:
#   wb = np.load('/tmp/openpilot/selfdrive/controls/lib/dynamic_follow/auto_df_weights.npz', allow_pickle=True)
# else:
#   wb = np.load('/data/openpilot/selfdrive/controls/lib/dynamic_follow/auto_df_weights.npz', allow_pickle=True)

w, b = wb['wb']

def softmax(x):
  return np.exp(x) / np.sum(np.exp(x), axis=0)

def predict(x):
  l0 = np.dot(x, w[0]) + b[0]
  l0 = np.maximum(0, l0)
  l1 = np.dot(l0, w[1]) + b[1]
  l1 = np.maximum(0, l1)
  l2 = np.dot(l1, w[2]) + b[2]
  l2 = softmax(l2)
  return l2
