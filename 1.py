import open3d as o3d
import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from OpenGL.GL import shaders
from Shader import SZH_shader as sshader
from PIL import Image
import os 
def check_shader_errors(shader, shader_type):
    status = glGetShaderiv(shader, GL_COMPILE_STATUS)
    if not status:
        info_log = glGetShaderInfoLog(shader)
        print(f"Compile {shader_type} error: {info_log}")
        return False
    return True

class SkyboxRender:
    def __init__(self, mesh, env_map):
        self.mesh = mesh
        self.env_map = env_map
        self.m_tex_env = 0
        self.shader_bubble = sshader()  # 假设sshader()是正确初始化shader程序的函数
        if not self.shader_bubble.program:
            print("Shader program not initialized correctly.")
            return
        # 简化的着色器代码
        vertex_shader_code = """
        #version 330 core
        layout(location = 0) in vec3 aPos;
        void main() {
            gl_Position = vec4(aPos, 1.0);
        }
        """
        fragment_shader_code = """
        #version 330 core
        out vec4 FragColor;
        void main() {
            FragColor = vec4(1.0, 0.5, 0.2, 1.0);
        }
        """

        # 创建着色器并编译
        vs = glCreateShader(GL_VERTEX_SHADER)
        fs = glCreateShader(GL_FRAGMENT_SHADER)
        glShaderSource(vs, vertex_shader_code)
        glShaderSource(fs, fragment_shader_code)
        glCompileShader(vs)
        glCompileShader(fs)
        if not check_shader_errors(vs, "vertex shader"):
            return
        if not check_shader_errors(fs, "fragment shader"):
            return

        # 附加着色器并链接程序
        glAttachShader(self.shader_bubble.program, vs)
        glAttachShader(self.shader_bubble.program, fs)
        glBindAttribLocation(self.shader_bubble.program, 0, 'aPos')
        glLinkProgram(self.shader_bubble.program)
        link_status = glGetProgramiv(self.shader_bubble.program, GL_LINK_STATUS)
        if not link_status:
            info_log = glGetProgramInfoLog(self.shader_bubble.program)
            print(info_log)
            print("Link error:", info_log)
        else:
            print("Shader program linked successfully.")

# 假设有合适的mesh和env_map
skybox_renderer = SkyboxRender(None, None)