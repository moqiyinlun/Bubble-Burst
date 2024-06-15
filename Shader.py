import os
from OpenGL.GL import *
from OpenGL.GL.shaders import compileProgram, compileShader
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
        Shader._current_shader = self

    @staticmethod
    def deactivate():
        glUseProgram(0)
        Shader._current_shader = None

    def set_uniform(self, name, value):
        location = self.get_uniform_location(name)
        if isinstance(value, int):
            glUniform1i(location, value)
        elif isinstance(value, float):
            glUniform1f(location, value)
        elif isinstance(value, tuple) and len(value) == 3:
            glUniform3f(location, *value)
        elif isinstance(value, tuple) and len(value) == 4:
            glUniform4f(location, *value)
        elif isinstance(value, list) and len(value[0]) == 3:
            glUniform3fv(location, len(value), [x for vec in value for x in vec])
        elif isinstance(value, list) and len(value[0]) == 4:
            glUniform4fv(location, len(value), [x for vec in value for x in vec])

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