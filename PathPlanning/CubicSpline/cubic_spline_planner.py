'''
Cubic spline planner

Date: 09/06/25
'''
import math
import numpy as np
import bisect


class CubicSpline1D:
    '''
    1D Cubic Spline class
    
    Parameters
    ----------
    x : list
        x coordinates for data points.
        This x coordinates must be sorted.
        in ascending order.
    y : list
        y coordinates for data points.
    '''

    def __init__(self, x, y):
        h = np.diff(x)
        if np.any(h < 0):
            raise ValueError('x coordinates must be sorted in ascending order.')
        
        self.a, self.b, self.c, self.d = [], [], [], []
        self.x = x
        self.y = y
        self.nx = len(x)    # dimension of x

        # calc coefficient a
        self.a = [iy for iy in y]

        # calc coefficient c
        A = self.__calc_A(h)
        B = self.__calc_B(h, self.a)
        self.c = np.linalg.solve(A, B)

        # calc coefficient b, d
        for i in range(self.nx - 1):
            d = (self.c[i + 1] - self.c[i]) / (3.0 * h[i])
            b = 1.0 / h[i] * (self.a[i + 1] - self.a[i])\
                - h[i] / 3.0 * (2.0 * self.c[i] + self.c[i + 1])
            self.d.append(d)
            self.b.append(b)

    def calc_position(self, x):
        '''
        Calc 'y' position for given 'x'.
        if 'x' is outside the data point's 'x' range, return None.

        Parameters
        ---------
        x : float
            x position to calculate.

        Returns:
        y : float
            y position for given x.
        '''
        if x < self.x[0]:
            return None
        if x > self.x[-1]:
            return None
        
        i = self.__search_index(x)
        dx = x - self.x[i]
        position = self.a[i] + self.b[i] * dx + \
            self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0
        
        return position
    
    def calc_first_derivative(self, x):
        '''
        Calculate first derivatives at give x.
        if 'x' is outside the input data's 'x' range, return None.

        Parameters
        ----------
        x : float
            input data x to calculate.

        Returns:
        ----------
            first derivatives at 'x'.
        '''

        if x < self.x[0]:
            return None
        if x > self.x[-1]:
            return None
        
        i = self.__search_index(x)
        dx = x - self.x[i]
        dy = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
        return dy
    
    def calc_second_derivative(self, x):
        '''
        Calculate second derivatives at give x.
        if 'x' is outside the input data's 'x' range, return None.

        Parameters
        ----------
        x : float
            input data x to calculate.

        Returns:
        ----------
            second derivatives at 'x'.
        '''
        if x < self.x[0]:
            return None
        if x > self.x[-1]:
            return None
        
        i = self.__search_index(x)
        dx = x - self.x[i]
        ddy = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
        return ddy

    def calc_third_derivative(self, x):
        '''
        Calculate third derivatives at give x.
        if 'x' is outside the input data's 'x' range, return None.

        Parameters
        ----------
        x : float
            input data x to calculate.

        Returns:
        ----------
            third derivatives at 'x'.
        '''
        if x < self.x[0]:
            return None
        if x > self.x[-1]:
            return None
        
        i = self.__search_index(x)
        dddy = 6.0 * self.d[i]
        return dddy

    def __search_index(self, x):
        '''
        search data segment index
        '''
        return bisect.bisect(self.x, x) - 1

    def __calc_A(self, h):
        '''
        calc matrix A for spline coefficient c
        '''
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        # 三对角矩阵，每次填充i相关的对角线上的数值.
        for i in range(self.nx - 1):
            if i != (self.nx -2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]

        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0
        A[self.nx - 1, self.nx - 1] = 1.0
        return A

    def __calc_B(self, h, a):
        '''
        calc matrix B for spline coefficient c.
        '''
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = 3.0 * (a[i + 2] - a[i + 1]) / h[i + 1]\
                        - 3.0 * (a[i + 1] - a[i]) / h[i]
        return B


class CubicSpline2D:
    '''
    Cubic Spline2D class

    Parameters
    ----------
    x : list
        x coordinate for data points.
    y : list
        y coordinate for data points.
    '''

    def __init__(self, x, y):
        self.s = self.__calc_s(x, y)
        self.sx = CubicSpline1D(self.s, x)
        self.sy = CubicSpline1D(self.s, y)

    def __calc_s(self, x, y):
        '''
        >>> import numpy as np
        >>> x = np.array([3, 6, 9])
        >>> y = np.array([4, 8, 12])
        >>> np.hypot(x, y)
        >>> [5, 10, 13]
        >>> np.cumsum(np.hypot(x, y))
        >>> [5, 15, 28]
        '''
        
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = np.hypot(dx, dy) # 求一组点的距离并返回距离数组
        s = [0]
        s.extend(np.cumsum(self.ds)) # 计算组元素累积和的函数
        return s

    
    def calc_position(self, s):
        '''
        calc position

        Parameters
        ----------
        s : float
            distance from the start point. if 's' is outside the data point's
            range, return None.

        Returns
        -------
        x : float
            x position for given s.
        y : float
            y position for given s.
        '''

        x = self.sx.calc_position(s)
        y = self.sy.calc_position(s)
        return x, y
    
    def calc_curvature(self, s):
        '''
        calc curvature

        Parameters
        ----------
        s : float
            distance from the start point. if 's' is outside the data point's
            range, return None.

        Returns
        -------
        k : float
            curvature for give s.
        '''
        dx = self.sx.calc_first_derivative(s)
        ddx = self.sx.calc_second_derivative(s)
        dy = self.sy.calc_first_derivative(s)
        ddy = self.sy.calc_second_derivative(s)

        k = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2) ** (3 / 2))
        return k
    
    def calc_curvature_rate(self, s):
        '''
        calc curvature

        Parameters
        ----------
        s : float
            distance from the start point. if 's' is outside the data point's
            range, return None.

        Returns
        -------
        k_rate : float
            curvature rate for give s.
        '''
        dx = self.sx.calc_first_derivative(s)
        dy = self.sy.calc_first_derivative(s)
        ddx = self.sx.calc_second_derivative(s)
        ddy = self.sy.calc_second_derivative(s)
        dddx = self.sx.calc_third_derivative(s)
        dddy = self.sy.calc_third_derivative(s)

        a = dx ** 2 + dy ** 2
        b = dx * dddy - dddx * dy
        c = dx * ddy - ddx * dy
        d = dx * ddx + dy * ddy

        k_rate = (a * b - 3.0 * c * d) / (d ** (5 / 2))
        return k_rate

    def calc_yaw(self, s):
        '''
        calc curvature

        Parameters
        ----------
        s : float
            distance from the start point. if 's' is outside the data point's
            range, return None.

        Returns
        -------
        yaw : float
            yaw angle (tangent vector) for give s.
        '''

        dx = self.sx.calc_first_derivative(s)
        dy = self.sy.calc_second_derivative(s)
        yaw = math.atan2(dy, dx)
        return yaw
    
def calc_spline_course(x, y, ds=0.1):
    sp = CubicSpline2D(x, y)
    s = list(np.arange(0, sp.s[-1], ds))

    rx, ry, ryaw, rk = [], [], [], []

    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))

    return rx, ry, ryaw, rk


def main_1d():
    print("CubicSplin1D test")
    import matplotlib.pyplot as plt
    x = np.arange(5)
    y = [1.7, -6.0, 5.0, 6.5, -1.0]
    sp = CubicSpline1D(x, y)
    xi = np.linspace(0.0, 5.0)

    plt.plot(x, y, 'xb', label="Data points")
    plt.plot(xi, [sp.calc_position(x) for x in xi], "r",
             label="Cubic spline interpolation")
    plt.grid(True)
    plt.legend()
    plt.show()


def main_2d():
    print('CubicSpline2D test.')
    import matplotlib.pyplot as plt

    x = [-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0]
    y = [0.7, -6.0, 5.0, 6.5, 0.0, 5.0, -2.0]
    ds = 0.1 # [m] distance of each interpolated points

    sp = CubicSpline2D(x, y)
    s = list(np.arange(0, sp.s[-1], ds))

    rx, ry, ryaw, rk = [], [], [], []

    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))

    plt.subplots(1)
    plt.plot(x, y, "xb", label="Data points")
    plt.plot(rx, ry, '-r', label="Cubic spline path")
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.legend()

    plt.subplots(1)
    plt.plot(s, [np.rad2deg(iyaw) for iyaw in ryaw], "-r", label="yaw")
    plt.grid(True)
    plt.legend()
    plt.xlabel("line length[m]")
    plt.ylabel("yaw angle[deg]")

    plt.plot(s, rk, "-.k", label="curvature")
    plt.grid(True)
    plt.legend()
    plt.xlabel("line length[m]")
    plt.ylabel("curvature [1/m]")

    plt.show()

if __name__ == '__main__':
    # main_1d()
    main_2d()
        