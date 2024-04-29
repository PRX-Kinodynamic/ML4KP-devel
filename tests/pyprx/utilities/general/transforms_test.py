import os
import math
import pytest
import PyML4KP as prx


def test_transform():
	tf = prx.transform()
	tf.setIdentity()
	rot = tf.rotation()
	assert(rot.determinant() == 1)
	assert(rot(0,0) == 1)
	assert(rot(1,1) == 1)
	assert(rot(2,2) == 1)

	tf.translation(prx.vector(0.1,0,0.5))
	tr = tf.translation()
	assert(tr[0] == 0.1)
	assert(tr[1] == 0)
	assert(tr[2] == 0.5)
