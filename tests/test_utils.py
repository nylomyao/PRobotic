import numpy as np
from numpy.testing import assert_allclose
import conftest
from utils import angle

def test_angle_mod():
    assert_allclose(angle.angle_mod(-4.0), 2.2831853071795862)
    assert isinstance(angle.angle_mod(-4.0), float)
    assert_allclose(angle.angle_mod([-9.0]), [-2.7168146928204138])
    assert isinstance(angle.angle_mod([-9.0]), np.ndarray)

    assert_allclose(angle.angle_mod([100.0, 200.0, -150.0, -190.0], degree=True), 
                                    [99.99999999999999, -160.00000000000003, -150.0, 170.0])
    assert_allclose(angle.angle_mod([-150.0, 190.0, 350], degree=True),
                                    [-150., -170., -10.])

    assert_allclose(angle.angle_mod([-4.0, 2.0, 3.0, 2 * np.pi], zero_2_2pi=True), 
                                    [2.2831853071795862, 2.0, 3.0, 0.0])
    

if __name__ == '__main__':
    conftest.run_this_test(__file__)