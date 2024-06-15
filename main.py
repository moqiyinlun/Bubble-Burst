import open3d as o3d
import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from SkyboxRenderer import *
class UI_Config:
    def __init__(self):
        # Window size
        self.win_w = 1280
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
    mesh = o3d.io.read_triangle_mesh("assets/sample_mesh/6bubbles.obj")
    vertices = np.asarray(mesh.vertices)
    if not mesh.has_triangle_normals():
        mesh.compute_triangle_normals()
    normals = np.asarray(mesh.triangle_normals)
    triangles = np.asarray(mesh.triangles)
    return vertices, normals, triangles
def renderBitmapString(x, y, z, s):
    glColor3f(0, 0, 0)
    glRasterPos3f(x, y, z)
    for i in range(len(s)):
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, ord(s[i]))
def display():
    global config,skybox
    glClearColor(1, 1, 1, 1)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, float(config.win_w) / float(config.win_h), 0.001, 50)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    gluLookAt(0, -config.view_dist, 0, 0, 0, 0, 0, 0, 1)
    glRotated(config.view_alpha, 1, 0, 0)
    glRotated(config.view_theta, 0, 0, 1)

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
    # A = SkyboxRender("","textures/beach")
    skybox.render()
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    glOrtho(0, config.win_w, 0, config.win_h, -1, 1)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    
    renderBitmapString(20, config.win_h - 20, 0, "T = 0")
    
    glutSwapBuffers()

def main():
    global config,skybox 
    config = UI_Config()
    glutInit()
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
    glutInitWindowSize(config.win_w, config.win_h)
    glutCreateWindow("PyOpenGL with Open3D and UI Config")
    skybox = SkyboxRender("","textures/beach")
    glutDisplayFunc(display)
    glutMainLoop()

if __name__ == "__main__":
    main()