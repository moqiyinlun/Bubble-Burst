import open3d as o3d
import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from OpenGL.GL import shaders
from PIL import Image
import os 

class SkyboxRender:
    def __init__(self, mesh, env_map):
        self.mesh = mesh
        self.env_map = env_map
        self.m_tex_env = 0
        self.shader_bubble = glCreateProgram()
        # compile vertex shader and fragment shader 
        vertex_shader_code = """
        attribute vec4 a_position;
        attribute vec3 a_normal;

        uniform mat4 u_mat_mvp;
        uniform mat4 u_mat_mv;

        varying vec4 v_position_world;
        varying vec3 v_normal;

        void main()
        {
            v_position_world = a_position;
            v_normal = a_normal;
            
            gl_Position = u_mat_mvp * a_position;
        }
        """
        fragment_shader_code = """ 
        uniform sampler2D u_tex_depth;
        uniform samplerCube u_tex_env;

        uniform mat4 u_mat_mvp;
        uniform mat4 u_mat_mv;

        uniform vec3 u_light_direction;
        uniform vec4 u_light_diffuse;
        uniform vec4 u_light_ambient;
        uniform vec4 u_light_specular;
        uniform float u_light_specularity;
        uniform vec3 u_camera_pos;
        uniform float u_ssao_coef;

        uniform vec3 u_ssao_samples[128];

        varying vec4 v_position_world;
        varying vec3 v_normal;

        void main()
        {
            vec3 normal = normalize(v_normal);

            vec4 shading_diffuse = u_light_diffuse * clamp(dot(normal, -u_light_direction), 0.0, 1.0);

            vec3 viewvec = normalize(u_camera_pos - v_position_world.xyz);
            vec3 lightvec = -u_light_direction;
            vec3 halfway = normalize(viewvec + lightvec);
            vec4 shading_specular = u_light_specular * pow(clamp(dot(halfway, normal), 0.0, 1.0), u_light_specularity);

            vec4 shading_ambient = vec4(0.3, 0.3, 0.3, 0.3);
            
            vec3 reflectiondir = reflect(-viewvec, normal);
            vec4 reflection = textureCube(u_tex_env, reflectiondir);
            
            float angle = abs(dot(viewvec, normal));
            float alpha = 0.3 + 0.9 * pow(1.0 - angle, 2.0);
            
            gl_FragColor = vec4(reflection.xyz * 1.5, alpha);

        }
        """
        bubble_vertex = shaders.compileShader(vertex_shader_code, GL_VERTEX_SHADER) 
        bubble_fragment = shaders.compileShader(fragment_shader_code, GL_FRAGMENT_SHADER)
        glAttachShader(self.shader_bubble, bubble_vertex)
        glAttachShader(self.shader_bubble, bubble_fragment)
        glBindAttribLocation(self.shader_bubble, 0, 'a_position')
        glBindAttribLocation(self.shader_bubble, 1, 'a_normal')
        # Link the program
        glLinkProgram(self.shader_bubble)
        glValidateProgram(self.shader_bubble)
    def create_cube_map(self):
        if not self.env_map_path:
            return False

        # Generate a cube-map texture to hold all the sides
        glActiveTexture(GL_TEXTURE0)
        self.m_tex_env = glGenTextures(1)
        glBindTexture(GL_TEXTURE_CUBE_MAP, self.m_tex_env)

        sides = [
            (GL_TEXTURE_CUBE_MAP_NEGATIVE_Z, 'bottom.png'),
            (GL_TEXTURE_CUBE_MAP_POSITIVE_Z, 'top.png'),
            (GL_TEXTURE_CUBE_MAP_POSITIVE_Y, 'front.png'),
            (GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, 'back.png'),
            (GL_TEXTURE_CUBE_MAP_NEGATIVE_X, 'left.png'),
            (GL_TEXTURE_CUBE_MAP_POSITIVE_X, 'right.png')
        ]

        # Load each image and copy it into a side of the cube-map texture
        for target, suffix in sides:
            image_path = f"{self.env_map_path}_{suffix}"
            if not self.load_cube_map_side(self.m_tex_env,target, image_path):
                return False

        # Set texture parameters
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)

        return True
    def load_cube_map_side(tex, side, filename):
        try:
            with Image.open(filename) as img:
                img = img.convert("RGBA")  # Ensure the image is in RGBA format
                data = np.array(img)  # Convert image to numpy array
                data = data[::-1, :, :]  # Flip the image vertically
            # Bind the texture
            glBindTexture(GL_TEXTURE_CUBE_MAP, tex)
            # Load the image data into the cube map side
            glTexImage2D(side, 0, GL_RGBA, img.width, img.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data)
            
            return True
        except Exception as e:
            print(f"An error occurred while loading {filename}: {e}")
            return False