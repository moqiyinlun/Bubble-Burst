import open3d as o3d
import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

class UI_Config:
    def __init__(self):
        # Window size
        self.win_w = 12801
        self.win_h = 720
        # Simulation Control
        self.step = False
        self.run = False
        self.autoload = False
        # Camera parameter
        self.view_theta = 0.0
        self.view_alpha = 0.0
        self.view_dist = 4
        # Mouse position and dragging status
        self.mouse_x = 0
        self.mouse_y = 0
        self.ldrag = False
        self.ldrag_start_x = 0
        self.ldrag_start_y = 0
        self.rdrag = False
        self.rdrag_start_x = 0
        self.rdrag_start_y = 0

def load_mesh():
    # 读取.obj文件
    mesh = o3d.io.read_triangle_mesh("assets/sample_mesh/6bubbles.obj")
    vertices = np.asarray(mesh.vertices)
    if not mesh.has_triangle_normals():
        mesh.compute_triangle_normals()
    normals = np.asarray(mesh.triangle_normals)
    triangles = np.asarray(mesh.triangles)
    return vertices, normals, triangles

def display():
    global config
    glClearColor(1, 1, 1, 1)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    # 设置投影矩阵
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, float(config.win_w) / float(config.win_h), 0.001, 50)

    # 设置模型视图矩阵
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    gluLookAt(0, -config.view_dist, 0, 0, 0, 0, 0, 0, 1)
    glRotated(config.view_alpha, 1, 0, 0)
    glRotated(config.view_theta, 0, 0, 1)

    # 绘制坐标轴
    glBegin(GL_LINES)
    glColor3d(1, 0, 0)
    glVertex3d(0, 0, 0)
    glVertex3d(2, 0, 0)
    glColor3d(0, 1, 0)
    glVertex3d(0, 0, 0)
    glVertex3d(0, 2, 0)
    glColor3d(0, 0, 1)
    glVertex3d(0, 0, 0)
    glVertex3d(0, 0, 2)
    glEnd()

    # 绘制模型
    vertices, normals, triangles = load_mesh()
    glEnableClientState(GL_VERTEX_ARRAY)
    glVertexPointer(3, GL_FLOAT, 0, vertices)
    glEnableClientState(GL_NORMAL_ARRAY)
    glNormalPointer(GL_FLOAT, 0, normals)
    glDrawElements(GL_TRIANGLES, len(triangles) * 3, GL_UNSIGNED_INT, triangles)
    glDisableClientState(GL_VERTEX_ARRAY)
    glDisableClientState(GL_NORMAL_ARRAY)

    # 交换缓冲区
    glutSwapBuffers()

def main():
    global config
    config = UI_Config()

    glutInit()
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
    glutInitWindowSize(config.win_w, config.win_h)
    glutCreateWindow("PyOpenGL with Open3D and UI Config")
    glutDisplayFunc(display)
    glutMainLoop()

if __name__ == "__main__":
    main()