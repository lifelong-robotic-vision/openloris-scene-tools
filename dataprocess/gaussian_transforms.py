#!/usr/bin/env python2
from segway_transforms import TFParser, tf_values_to_matrix, tf_matrix_to_values
import tf
import numpy as np

def get_tfs():
    tf_tree = [
        ('tree', 'base_link', 'laser', [0.35,0.0,1.28,-0.009737,0.05002,-0.004493,0.9986]),
        ('tree', 'laser', 'd400_color', [0.0559649,0.0311189 ,-0.0756753 ,-0.52531 ,0.494111 ,-0.469561 ,0.50933]),
        ('tree', 'd400_color', 'd400_depth', [0.0147083755582571 ,3.83682090614457e-05,  0.000288475974230096   ,0.000504782, 0.00444093, -0.00264019, 0.999987]),
        ('tree', 'd400_color', 'd400_imu', [0.0200968869030476 , -0.0050785536877811 , -0.0115051260218024 ,-0.000504782,0.00444093, -0.00264019, 0.999987]),
        ('tree', 'base_link', 't265_fisheye1', [0.873, 0.026, 0.680, -0.395, 0.406, -0.600, 0.565]),
        ('tree', 't265_fisheye1', 't265_fisheye2', [0.0640782490372658 , 0.000390329543733969 , -0.000322926469380036,0.00186088, 0.0031785, 0.000162728, 0.999993]),
        ('tree', 't265_fisheye1', 't265_imu', [0.0106999985873699, 7.27595761418343e-12, -2.91038304567337e-11, 0.00554841, 0.00136098, 0.99998062, -0.00245956]),
    ]
    return tf_tree

"""
Get the best transform matrix
"""
def get_tf_matrix(parent, child):
    if parent == child:
        return tf_values_to_matrix([0,0,0,0,0,0,1])
    tfs = TFParser(get_tfs())
    t = tfs.get_tf_matrix('tree', parent, child)
    if t is not None: return t
    t = tfs.get_tf_matrix_inverse('tree', child, parent)
    if t is not None: return t
    t1 = tfs.get_tf_matrix_inverse('tree', 'base_link', parent)
    t2 = tfs.get_tf_matrix('tree', 'base_link', child)
    if t1 is not None and t2 is not None:
        return np.dot(t1, t2)
    if parent == 'base_link' and child == 'd400_color':
        t1 = tfs.get_tf_matrix('tree', 'base_link', 'laser')
        t2 = tfs.get_tf_matrix('tree', 'laser', 'd400_color')
        return np.dot(t1, t2)
    if parent == 'd400_color' and child == 'base_link':
        t1 = tfs.get_tf_matrix_inverse('tree', 'laser', 'd400_color')
        t2 = tfs.get_tf_matrix_inverse('tree', 'base_link', 'laser')
        return np.dot(t1, t2)
    if child.startswith('t265'):
        t1 = get_tf_matrix(parent, 't265_fisheye1')
        t2 = get_tf_matrix('t265_fisheye1', child)
        if t1 is not None and t2 is not None:
            return np.dot(t1, t2)
    if parent.startswith('t265'):
        t1 = get_tf_matrix('t265_fisheye1', parent)
        t2 = get_tf_matrix(child, 't265_fisheye1')
        if t1 is not None and t2 is not None:
            return tf.transformations.inverse_matrix(np.dot(t2, t1))
    if child.startswith('d400'):
        t1 = get_tf_matrix(parent, 'd400_color')
        t2 = get_tf_matrix('d400_color', child)
        if t1 is not None and t2 is not None:
            return np.dot(t1, t2)
    if parent.startswith('d400'):
        t1 = get_tf_matrix('d400_color', parent)
        t2 = get_tf_matrix(child, 'd400_color')
        if t1 is not None and t2 is not None:
            return tf.transformations.inverse_matrix(np.dot(t2, t1))
    exit('Not sure how to get tf of %s from %s' % (parent, child))

def print_tf():
    targets = ['d400_color', 'd400_depth', 'd400_imu', 't265_fisheye1', 't265_fisheye2', 't265_imu']
    for target in targets:
        m = get_tf_matrix('base_link', target)
        v = tf_matrix_to_values(m)
        print("('%s', '%s', %s)," % ('base_link', target, str(v)))
    for target in targets:
        m = get_tf_matrix(target, 'base_link')
        v = tf_matrix_to_values(m)
        print("('%s', '%s', %s)," % (target, 'base_link', str(v)))

if __name__ == '__main__':
    print_tf()