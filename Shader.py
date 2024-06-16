import os
from OpenGL.GL import *
from OpenGL.GL.shaders import compileProgram, compileShader
def check_shader_errors(shader, shader_type):
    status = glGetShaderiv(shader, GL_COMPILE_STATUS)
    if not status:
        info_log = glGetShaderInfoLog(shader)
        print(f"Compile {shader_type} error: {info_log}")
        return False
    return True
class SZH_shader:
    _current_shader = None

    def __init__(self):
        self.program = glCreateProgram()
        if not self.program:
            raise Exception("Failed to create shader program")
        self.vertex_shader = None
        self.fragment_shader = None
        self.uniform_locations = {}
        self.vertex_attributes = {}
    def activate(self):
        glUseProgram(self.program)
        SZH_shader._current_shader = self

    @staticmethod
    def deactivate():
        glUseProgram(0)
        SZH_shader._current_shader = None
    def loadFromCode(self,vs_code,fs_code,attribute_list):
        vs = glCreateShader(GL_VERTEX_SHADER)
        fs = glCreateShader(GL_FRAGMENT_SHADER)
        glShaderSource(vs,vs_code)
        glShaderSource(fs,fs_code)
        glCompileShader(vs)
        glCompileShader(fs)
        if not check_shader_errors(vs, "vertex shader"):
            return
        if not check_shader_errors(fs, "fragment shader"):
            return
        glAttachShader(self.program, vs)
        glAttachShader(self.program, fs)
        for i in range(len(attribute_list)):

            glBindAttribLocation(self.program, i, attribute_list[i])
        # Link the program
        glLinkProgram(self.program)
        link_status = glGetProgramiv(self.program, GL_LINK_STATUS)
        if not link_status:
            info_log = glGetProgramInfoLog(self.program)
            print(info_log)
            print(f"Link error: {info_log}")
        else:
            print("Link success")
    def set_uniform(self, name, value):
        location = self.get_uniform_location(name)
        if isinstance(value, int):
            glUniform1i(location, value)
        elif isinstance(value, float):
            glUniform1f(location, value)
        elif isinstance(value, list) and len(value) == 3:
            print("This is glUniform3f!")
            glUniform3f(location, value[0],value[1],value[2])
        elif isinstance(value, list) and len(value) == 4:
            glUniform4f(location, *value)
        elif isinstance(value, list) and len(value[0]) == 3:
            glUniform3fv(location, 1, GL_FALSE, [x for vec in value for x in vec])
        elif isinstance(value, list) and len(value[0]) == 4:
            glUniform4fv(location, 1, GL_FALSE, [x for vec in value for x in vec])

    def get_uniform_location(self, name):
        if name in self.uniform_locations:
            return self.uniform_locations[name]
        location = glGetUniformLocation(self.program, name)
        self.uniform_locations[name] = location
        return location

    def set_vertex_attrib_name(self, name, index):
        self.vertex_attributes[name] = index

    def check_program_link_status(self):
        linked = glGetProgramiv(self.program, GL_LINK_STATUS)
        if not linked:
            info_log = glGetProgramInfoLog(self.program)
            print("Shader Linking Failed:", info_log)
        return linked