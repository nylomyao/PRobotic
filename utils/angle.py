import numpy as np
from scipy.spatial.transform import Rotation as R

import matplotlib.pyplot as plt

def rotation_scipy():
    def plot_rotated_axes(ax, r, name=None, offset=(0, 0, 0), scale=1):
        colors = ("#FF6666", "#005533", "#1199EE")
        loc = np.array([offset, offset])
        for i, (axis, c) in enumerate(zip((ax.xaxis, ax.yaxis, ax.zaxis),
                                          colors)):
            axlabel = axis.axis_name
            axis.set_label_text(axlabel)
            axis.label.set_color(c)
            axis.line.set_color(c)
            axis.set_tick_params(colors=c)
            line = np.zeros((2, 3))
            line[1, i] = scale
            line_rot = r.apply(line)
            line_plot = line_rot + loc
            ax.plot(line_plot[:, 0], line_plot[:, 1], line_plot[:, 2], c)
            text_loc = line[1] * 1.2
            text_loc_rot = r.apply(text_loc)
            text_plot = text_loc_rot + loc[0]
            ax.text(*text_plot, axlabel.upper(), color=c,
                    va='center', ha='center')
        ax.text(*offset, name, color='k', va='center', ha='center',
                bbox={'fc': 'w', 'alpha': 0.8, 'boxstyle': 'circle'})
        
    # 恒等变换，不变换
    r0 = R.identity()
    # intrinsic
    r1 = R.from_euler('zyx', [90, -30, 0], degrees=True)
    # extrinsic
    r2 = R.from_euler('zyx', [90, -30, 0], degrees=True)

    print(r2.as_matrix())

    # 创建一个Axes3D的对象，ax对象有很多方法，内部的呈现的子坐标是画出来的
    ax = plt.figure().add_subplot(projection='3d', proj_type='ortho')
    plot_rotated_axes(ax, r0, name='r0', offset=(0, 0, 0))
    # r2是在r0上进行旋转，不是在r1的基础上
    plot_rotated_axes(ax, r1, name='r1', offset=(3, 0, 0))
    plot_rotated_axes(ax, r2, name='r2', offset=(6, 0, 0))
    # '_' 占位符, 表示不打算使用它
    _ = ax.annotate(
        "r0: Identity Rotation\n"
        "r1: Intrinsic Euler Rotation (ZYX)\n"
        "r2: Extrinsic Euler Rotation (zyx)",
        xy=(0.6, 0.7), xycoords="axes fraction", ha="left"
    )

    ax.set(xlim=(-1.25, 7.25), ylim=(-1.25, 1.25), zlim=(-1.25, 1.25))
    ax.set(xticks=range(-1, 8), yticks=[-1, 0, 1], zticks=[-1, 0, 1])
    ax.set_aspect("equal", adjustable="box")
    ax.figure.set_size_inches(6, 5)
    plt.tight_layout()

    plt.show()



        

def rot_mat_2d():
    pass




def angle_mod(x, zero_2_2pi=False, degree=False):
    if isinstance(x, float):
        is_float = True
    else:
        is_float = False

    x = np.asarray(x).flatten()
    if degree:
        x = np.deg2rad(x)

    if zero_2_2pi:
        mod_angle = x % (2 * np.pi)
    else:
        mod_angle = (x + np.pi) % (2 * np.pi) - np.pi

    if degree:
        mod_angle = np.rad2deg(mod_angle)

    if is_float:
        return mod_angle.item()
    else:
        return mod_angle
