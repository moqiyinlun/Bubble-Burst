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
        obj_path = os.path.join(data_path,"{}.obj".format(name))
        self.obj = o3d.io.read_triangle_mesh(obj_path)
        self.label_path = os.path.join(data_path,"{}_flabel.txt".format(name))
        print(self.label_path)
        self.labels = []
        self.create_label_list()
        print(self.labels)
    def create_label_list(self):
        try:
            with open(self.label_path, "r") as file:
                lines = file.readlines()
                n_faces = int(lines[1].strip())  
                if n_faces != len(self.obj.triangles):
                    raise ValueError("Number of labels does not match number of faces")
                for line in lines[2:]:
                    parts = line.strip().split()
                    if len(parts) < 2:
                        continue  
                    label1 = int(parts[0])
                    label2 = int(parts[1])
                    self.labels.append((label1, label2))
        except Exception as e:
            print("Label file not found: {}".format(self.label_path))
            self.labels = [(1, 0) for _ in range(len(self.obj.triangles))]
        # with open(self.label_path,"w") as f:
        #     f.read()
class SkyboxRender:
    def __init__(self, mesh, env_map):
        self.mesh = MyDefinedMesh("assets/sample_mesh","6bubbles")
        self.env_map = env_map
        self.m_tex_env = 0
        self.shader_bubble = sshader()
        self.shader_env = sshader()
        # compile vertex shader and fragment shader 
        bubble_vertex_shader_code = """
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
        bubble_fragment_shader_code = """ 
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
        env_vertex_shader = """
            attribute vec4 a_position;

            uniform mat4 u_mat_mvp;

            varying vec4 v_position_world;

            void main()
            {
                v_position_world = a_position;
                
                gl_Position = u_mat_mvp * a_position;
            }
        """
        env_fragment_shader = """
            uniform samplerCube u_tex_env;

            uniform mat4 u_mat_mvp;
            uniform vec3 u_camera_pos;

            varying vec4 v_position_world;

            void main()
            {
                vec3 viewvec = v_position_world.xyz / v_position_world.w - u_camera_pos;
                gl_FragColor =  textureCube(u_tex_env, viewvec);
                
            }
        """
        bubble_attribute_list = ["a_position","a_normal"]
        env_attribute_list = ["a_position"]
        self.shader_bubble.loadFromCode(bubble_vertex_shader_code,bubble_fragment_shader_code,bubble_attribute_list)
        self.shader_env.loadFromCode(env_vertex_shader,env_fragment_shader,env_attribute_list)
        self.create_cube_map()
        self.vao = glGenVertexArrays(1)
        self.vbo = glGenBuffers(1)
        glBindVertexArray(self.vao)
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo)
        self.env_vb = np.array([
            -10.0,  10.0, -10.0,
            -10.0, -10.0, -10.0,
            10.0, -10.0, -10.0,
            10.0, -10.0, -10.0,
            10.0,  10.0, -10.0,
            -10.0,  10.0, -10.0,
            
            -10.0, -10.0,  10.0,
            -10.0, -10.0, -10.0,
            -10.0,  10.0, -10.0,
            -10.0,  10.0, -10.0,
            -10.0,  10.0,  10.0,
            -10.0, -10.0,  10.0,
            
            10.0, -10.0, -10.0,
            10.0, -10.0,  10.0,
            10.0,  10.0,  10.0,
            10.0,  10.0,  10.0,
            10.0,  10.0, -10.0,
            10.0, -10.0, -10.0,
            
            -10.0, -10.0,  10.0,
            -10.0,  10.0,  10.0,
            10.0,  10.0,  10.0,
            10.0,  10.0,  10.0,
            10.0, -10.0,  10.0,
            -10.0, -10.0,  10.0,
            
            -10.0,  10.0, -10.0,
            10.0,  10.0, -10.0,
            10.0,  10.0,  10.0,
            10.0,  10.0,  10.0,
            -10.0,  10.0,  10.0,
            -10.0,  10.0, -10.0,
            
            -10.0, -10.0, -10.0,
            -10.0, -10.0,  10.0,
            10.0, -10.0, -10.0,
            10.0, -10.0, -10.0,
            -10.0, -10.0,  10.0,
            10.0, -10.0,  10.0
        ], dtype=np.float32)
        
    def create_cube_map(self):
        if not self.env_map:
            return False

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

        for target, suffix in sides:
            image_path = f"{self.env_map}_{suffix}"
            try:
                img = Image.open(image_path)
                img_data = np.array(list(img.getdata()), np.uint8) # 图片数据
                img_data = img_data.reshape((img.size[1], img.size[0], 4)) # 根据图片尺寸调整形状
                img_data = np.flip(img_data, axis=0) # 垂直翻转图像
                glTexImage2D(target, 0, GL_RGBA, img.size[0], img.size[1], 0, GL_RGBA, GL_UNSIGNED_BYTE, img_data)
            except Exception as e:
                print(f"An error occurred while loading {image_path}: {e}")
                return False

        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)

        return True
    def render(self):
        glActiveTexture(GL_TEXTURE0)
        glBindTexture(GL_TEXTURE_CUBE_MAP, self.m_tex_env)
        glClearDepth(1.0)
        glClearColor(1.0, 1.0, 1.0, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        mv = glGetFloatv(GL_MODELVIEW_MATRIX)
        pj = glGetFloatv(GL_PROJECTION_MATRIX)
        mv_matrix = np.transpose(np.array(mv, dtype=np.float32).reshape(4, 4))
        pj_matrix = np.transpose(np.array(pj, dtype=np.float32).reshape(4, 4))
        mvp_matrix = np.dot(pj_matrix, mv_matrix)

        # Camera position from the inverted modelview matrix
        inv_mv_matrix = np.linalg.inv(mv_matrix)
        cam_pos_h = np.dot(inv_mv_matrix, np.array([0.0, 0.0, 1.0, 1.0]))
        cam_pos_h /= cam_pos_h[3]
        cam_pos = cam_pos_h[:3]
        
        for i in range(36):
            self.env_vb[i*3 + 0] += cam_pos[0]
            self.env_vb[i*3 + 1] += cam_pos[1]
            self.env_vb[i*3 + 2] += cam_pos[2]
        glBufferData(GL_ARRAY_BUFFER, self.env_vb.nbytes, self.env_vb, GL_STATIC_DRAW)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, None)
        glEnableVertexAttribArray(0)
        
        glBindBuffer(GL_ARRAY_BUFFER, 0)
        glBindVertexArray(0)
        self.shader_env.activate()
        mvp_location = glGetUniformLocation(self.shader_env.program, "u_mat_mvp")
        if mvp_location == -1:
            print("Failed to get uniform location for 'u_mat_mvp'")
            return

        glUniformMatrix4fv(mvp_location, 1, GL_FALSE, mvp_matrix)
        self.shader_env.set_uniform("u_tex_env", 0)
        self.shader_env.set_uniform("u_camera_pos", cam_pos)

        glBindVertexArray(self.vao)
        glDrawArrays(GL_TRIANGLES, 0, 36)
        glBindVertexArray(0)
        self.shader_env.deactivate()
        face_ordering = []
        # print(len(self.mesh.obj))
        # for i in range()
        
