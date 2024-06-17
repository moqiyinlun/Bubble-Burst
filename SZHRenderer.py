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
class FluidSimulation:
    def __init__(self,data_path,time,env_map):
        # pass 
        self.mesh_list = [MyDefinedMesh(os.path.join(data_path,"mesh_{}.obj".format(i)),os.path.join(data_path,"label_{}.txt".format(i))) for i in range(time)]
        # SkyboxRender
        self.renderer = SZHRenderer(self.mesh_list,env_map)
    def render(self):
        self.renderer.render()
    def step(self):
        # pass 
        self.renderer.update_mesh()
class MyDefinedMesh:
    def __init__(self,obj_path,label_path):
        self.obj = o3d.io.read_triangle_mesh(obj_path)
        self.label_path = label_path
        # print(self.label_path)
        self.vertex_normals = None
        self.labels = []
        self.create_label_list()
        # print(self.labels)
        self.compute_vertex_normals()
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
    def sort_triangles(self, cam_pos):
        vertices = np.asarray(self.obj.vertices)
        triangles = np.asarray(self.obj.triangles)
        centers = vertices[triangles].mean(axis=1)
        depths = np.dot(centers - cam_pos, cam_pos)
        sorted_indices = np.argsort(depths)[::-1]  # Sort from farthest to nearest
        return sorted_indices
    def compute_vertex_normals(self):
        if not self.obj.has_vertex_normals():
            self.obj.compute_vertex_normals()
        self.vertex_normals = np.asarray(self.obj.vertex_normals)

class SZHRenderer:
    def __init__(self, mesh_list, env_map):
        self.mesh_list = mesh_list 
        self.mesh = None
        self.index = 0
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

            vec4 shading_ambient = vec4(0.4, 0.3, 0.3, 0.3);
            
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
                gl_FragColor = textureCube(u_tex_env, viewvec);
                
            }
        """
        bubble_attribute_list = ["a_position","a_normal"]
        env_attribute_list = ["a_position"]
        self.shader_bubble.loadFromCode(bubble_vertex_shader_code,bubble_fragment_shader_code,bubble_attribute_list)
        self.shader_env.loadFromCode(env_vertex_shader,env_fragment_shader,env_attribute_list)
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

                data = data[::-1, :, :] # Flip the image vertically
            # Bind the texture
            glBindTexture(GL_TEXTURE_CUBE_MAP, tex)
            # Load the image data into the cube map side
            glTexImage2D(side, 0, GL_RGBA, img.width, img.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data)
            
            return True
        except Exception as e:
            print(f"An error occurred while loading {filename}: {e}")
            return False
    def update_mesh(self):
        self.index += 1
        print(self.index)
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
        env_vb = np.array([
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
        for i in range(36):
            env_vb[i*3 + 0] += cam_pos[0]
            env_vb[i*3 + 1] += cam_pos[1]
            env_vb[i*3 + 2] += cam_pos[2]
        self.shader_env.activate()
        glDisable(GL_DEPTH_TEST)
        glDepthMask(GL_FALSE)
        glCullFace(GL_FRONT_AND_BACK)
        glEnableVertexAttribArray(0)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, env_vb)
        # row main index 
        glUniformMatrix4fv(self.shader_env.get_uniform_location("u_mat_mvp"), 1, GL_FALSE, mvp_matrix.T)
        self.shader_env.set_uniform("u_tex_env", 0)
        self.shader_env.set_uniform("u_camera_pos", [cam_pos[0],cam_pos[1],cam_pos[2]])
        
        glDrawArrays(GL_TRIANGLES, 0, 36)
        
        self.shader_env.deactivate()
        # face_ordering = []
        self.mesh = self.mesh_list[self.index]
        face_ordering = self.mesh.sort_triangles(cam_pos)
        triangles = np.asarray(self.mesh.obj.triangles)[face_ordering]
        vertex = np.asarray(self.mesh.obj.vertices)
        normals = self.mesh.vertex_normals
        vb = np.zeros((len(triangles) * 9), dtype=np.float32)
        nb = np.zeros((len(triangles) * 9), dtype=np.float32)
        for i,tri in enumerate(triangles):
            for j in range(3):
                vb[i*9 + j*3 : i*9 + (j+1) *3] = vertex[tri[j]]
                nb[i*9 + j*3:i*9 + (j+1)*3] = normals[tri[j]]
        self.shader_bubble.activate()
    
        glDisable(GL_DEPTH_TEST)
        glDepthMask(GL_FALSE)
        glCullFace(GL_FRONT_AND_BACK)
        
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        
        glEnableVertexAttribArray(0)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, vb)
        glEnableVertexAttribArray(1)
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nb)
        self.shader_bubble.set_uniform("u_camera_pos", [cam_pos[0],cam_pos[1],cam_pos[2]])
        glUniformMatrix4fv(self.shader_bubble.get_uniform_location("u_mat_mvp"), 1, GL_FALSE, mvp_matrix.T)
        glUniform3f(self.shader_bubble.get_uniform_location("u_light_direction"), 0.26726124191, 0.53452248382, -0.80178372573)
        glUniform4f(self.shader_bubble.get_uniform_location("u_light_ambient"), 0.3, 0.3, 0.3, 1)
        glUniform4f(self.shader_bubble.get_uniform_location("u_light_diffuse"), 1, 1, 1, 1)
        glUniform4f(self.shader_bubble.get_uniform_location("u_light_specular"), 1, 1, 1, 1)
        glUniform1f(self.shader_bubble.get_uniform_location("u_light_specularity"), 120)
        self.shader_bubble.set_uniform("u_tex_env", 0)
        glDrawArrays(GL_TRIANGLES, 0, len(face_ordering) * 3)
        self.shader_bubble.deactivate()
        glEnable(GL_DEPTH_TEST)
        glDepthMask(GL_TRUE)
        glDisable(GL_BLEND)
        
