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
class MyDefinedMesh:
    def __init__(self,data_path,name):
        obj_path = os.path.join(data_path,"{}.obj")
        self.obj = o3d.io.read_triangle_mesh(obj_path)
        self.label_path = os.path.join(data_path,"{}_flabel.txt")
    def create_label_list(self):
        # with open(self.label_path,"w") as f:
        #     f.read()
class SkyboxRender:
    def __init__(self, mesh, env_map):
        self.mesh = mesh
        self.env_map = env_map
        self.m_tex_env = 0
        self.shader_bubble = sshader()
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
        vs = glCreateShader(GL_VERTEX_SHADER)
        fs = glCreateShader(GL_FRAGMENT_SHADER)
        glShaderSource(vs,vertex_shader_code)
        glShaderSource(fs,fragment_shader_code)
        glCompileShader(vs)
        glCompileShader(fs)
        if not check_shader_errors(vs, "vertex shader"):
            return
        if not check_shader_errors(fs, "fragment shader"):
            return
        glAttachShader(self.shader_bubble.program, vs)
        glAttachShader(self.shader_bubble.program, fs)
        glBindAttribLocation(self.shader_bubble.program, 0, 'a_position')
        glBindAttribLocation(self.shader_bubble.program, 1, 'a_normal')
        # Link the program
        glLinkProgram(self.shader_bubble.program)
        link_status = glGetProgramiv(self.shader_bubble.program, GL_LINK_STATUS)
        if not link_status:
            info_log = glGetProgramInfoLog(self.shader_bubble.program)
            print(info_log)
            print(f"Link error: {info_log}")
        else:
            print("Link success")
        self.create_cube_map()
    def create_cube_map(self):
        if not self.env_map:
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
            image_path = f"{self.env_map}_{suffix}"
            if not self.load_cube_map_side(self.m_tex_env,target, image_path):
                return False

        # Set texture parameters
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)

        return True
    def load_cube_map_side(self, tex, side, filename):
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
