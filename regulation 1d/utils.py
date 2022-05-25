import random

def clamp(v, min_v, max_v):
    return min(max_v, max(min_v, v))

def random_float(min_v, max_v):
    return random.random()*(max_v - min_v) + min_v
