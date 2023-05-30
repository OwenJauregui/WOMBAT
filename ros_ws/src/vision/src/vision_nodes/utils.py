#Additional libs
import numpy as np
import cv2 as cv
from pyzbar.pyzbar import decode
import struct

#Messages
from std_msgs.msg import Header
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from custom_msgs.msg import GridMap, Cell

#utilities
from sensor_msgs import point_cloud2

#Functions
def hello():

  print("Hello!")

def pcloud2numpy(p_cloud):

  #extract point cloud
  points = point_cloud2.read_points(p_cloud, skip_nans=True)

  #point cloud to numpy
  points = np.array(list(points))
  
  return points
  
class Vision():

  #Constructor
  def __init__(self, fx, fy, cx, cy, scale, d):

    #Definir matriz de parametros intrinsecos 
    self.K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    self.K_inv = np.linalg.inv(self.K)

    #Escala de nube de puntos
    self.scale = scale

    #Densidad de la nube
    self.d = d

    #Rotacion camara/mundo
    theta = -3.1416/2
    self.R = np.array([[1.0, 0.0, 0.0], [0.0, np.cos(theta), -np.sin(theta)], [0.0, np.sin(theta), np.cos(theta)]])
  
  def decoder(self, image):
    gray_img = cv.cvtColor(image,0)
    barcode = decode(gray_img)
    
    if len(barcode) != 0:
      for obj in barcode:

        #get storaged data from qr
        #(x,y,w,h) = obj.rect
        QRData = obj.data.decode("utf-8")
        #QRType = obj.type

        #these are for seeing the qr highlighted
        #points = obj.polygon
        #pts = np.array(points, np.int32)
        #pts = pts.reshape((-1, 1, 2))
        #cv.polylines(image, [pts], True, (0, 255, 0), 3)

        #string = "Data: " + str(QRData)
        #cv.putText(image, string, (x,y), cv.FONT_HERSHEY_SIMPLEX,0.8,(255,0,0), 2)
        
      return str(QRData)
    else:
      return "X"

  #Metodo 1: Calcular nube de puntos
  ##Input: 
  ##  img: Imagen RGB
  ##  depth: Mapa de profundidad
  ##Output:
  ##  p_cloud: Lista con puntos en 3D
  def point_cloud(self, img, depth):

    #Inicializar nube de puntos
    p_cloud = []

    #Ciclo de filas (V)
    for i in range(img.shape[0]):

      #Densidad
      if (i%self.d == 0):

        #Ciclo de columnas (U)
        for j in range(img.shape[1]):

          #Densidad
          if (j%self.d == 0):

            #Crear vector de punto en 2D
            p_2d = np.array([[j], [i], [1]]) 

            #Obtener z de mapa de profundidad
            z = self.scale*depth[i,j]
            #print(z)
            if (z > 0):

              #Calcular el punto en 3D
              p_3d = z*np.dot(self.K_inv, p_2d)

              #Transformar Camara - Rviz
              p_3d = np.dot(self.R,  p_3d)

              #Extraer color de la imagen (imagen BGR)
              b = img[i, j, 0]
              g = img[i, j, 1]
              r = img[i, j, 2]
              a = 255

              #Construir punto
              rgb = struct.unpack("I", struct.pack('BBBB', b, g, r, a))[0]
              pt = [p_3d[0,0], p_3d[1,0], p_3d[2,0], rgb]
              p_cloud.append(pt)
    
    #Regresar nube de puntos
    return p_cloud

  #Metodo 2: Convertir lista a nube de puntos 
  def plcloud2rviz(p_cloud, frame):

    header = Header()
    header.frame_id = frame

    #Definir campos 
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('rgba', 12, PointField.UINT32, 1)]

    #Crear nube de puntos 
    pc_msg = point_cloud2.create_cloud(header, fields, p_cloud)

    return pc_msg
  
#occupacy map
class OccGrid():

  #constructor
  def __init__(self, x_g=10, y_g=10, d_g=0.1, z_min=0.5, z_max=1.5, z_floor=0.0, p_occ=0.65, p_free=0.35, p_thresh=0.8, thresh_obst=0.5, thresh_angle=10.0*np.pi/180.0):
    #save attributes
    #grid parameters
    self.x_g = x_g
    self.y_g = y_g
    self.d_g = d_g
    self.zmin = z_min
    self.zmax = z_max
    self.z_floor = z_floor

    #algorythm param
    self.p_thresh = p_thresh
    self.p_occ = p_occ
    self.p_free = p_free

    #initialize auxiliary arrays
    self.map = np.zeros([self.x_g, self.y_g])

    #probability relation
    self.l_occ = np.log(self.p_occ/self.p_free)
    self.l_free = np.log(self.p_free/self.p_occ)

    #calculate positions on grid
    self.grid_position = np.array([np.tile(np.arange(-0.5*self.d_g*self.x_g, 0.5*self.d_g*self.x_g, self.d_g)[:,None],(1, self.y_g)),
                                   np.tile(np.arange(-0.5*self.d_g*self.y_g, 0.5*self.d_g*self.y_g, self.d_g)[:,None].T,(self.x_g, 1))])
    
    #save threshholds
    self.thresh_obs = thresh_obst
    self.thresh_angle = thresh_angle
    
    #print(self.grid_position.shape)

  #laser update
  def update_laser(self, state_odom, state_laser):
    
    #create a map copy
    occ_map_pos = self.grid_position.copy()

    #translate positions on grid
    occ_map_pos[0, :, :] -= state_odom[0]
    occ_map_pos[1, :, :] -= state_odom[1]

    #calculate orientation of grid
    occ_map_angle = np.arctan2(occ_map_pos[1, :, :], occ_map_pos[0, :, :])

    #adjust orientation
    occ_map_angle -= state_odom[2]

    #adjust orientation range
    occ_map_angle[occ_map_angle > np.pi] -= 2.0*np.pi
    occ_map_angle[occ_map_angle < -np.pi] += 2.0*np.pi

    #calculate grill distance from robot
    occ_map_dist = np.linalg.norm(occ_map_pos, axis=0)

    #analize laser info
    #print(state_laser.shape)
    #print(state_laser)
    for i in range(state_laser.shape[1]):
      #extract laser info
      angle = state_laser[0, i]
      dist = state_laser[1, i]

      #estimate occupied zones
      occ_zones = (np.abs(occ_map_angle - angle) < self.thresh_angle) & (np.abs(occ_map_dist - dist) <= self.thresh_obs)
      
      #determine free zones
      free_zones = (np.abs(occ_map_angle - angle) <= self.thresh_angle) & (occ_map_dist <= (dist - self.thresh_obs))
      
      #update map
      self.map[occ_zones] += self.l_occ
      self.map[free_zones] += self.l_free

  #instance methods
  def update(self, point_cloud):

    #analize point by point on grid
    for i in range(point_cloud.shape[0]):

      #unstruct point
      x = point_cloud[i, 0]
      y = point_cloud[i, 1]
      z = point_cloud[i, 2]

      #get coordinates from grid
      a, b = self.get_index(x,y)


      #verify point inside region
      if(z < self.zmax and z > self.zmin):

        #map points (camera - world) eventually...

        #verify point inside grid
        if a > 0 and a < self.map.shape[0] and b > 0 and b < self.map.shape[1]:
          #update as obstacle
          self.map[a,b] += self.l_occ


      #floor level
      if(z < self.z_floor):

        #verify point inside grid
        if a > 0 and a < self.map.shape[0] and b > 0 and b < self.map.shape[1]:
          #update as obstacle
          self.map[a,b] += self.l_free
  
  #calculate grid pos
  def get_index(self, x, y):

    a = int(np.ceil(x/self.d_g) + 0.5*self.map.shape[0])
    b = int(np.ceil(y/self.d_g) + 0.5*self.map.shape[1])

    return a, b
  
  #get map
  def get_map(self):

    occ_map = 1.0 - 1.0/(1 + np.exp(self.map))

    return occ_map
  
  #generate personalized msg occupacy grid
  def map_msg(self, occ_map, ros_time):

    #initialize msg
    msg_grid = GridMap()

    #analize occupacy map
    for i in range(occ_map.shape[0]):
      for j in range(occ_map.shape[1]):

        #initialice cell
        cell = Cell()

        #fill attributes
        #grid
        cell.grid.x = i
        cell.grid.y = j
        cell.grid.z = 0
        #position
        cell.position.x = self.d_g*(i -0.5*self.map.shape[0])
        cell.position.y = self.d_g*(j -0.5*self.map.shape[1])
        cell.position.z = 0.0

        #probability
        cell.prob.data = occ_map[i, j]

        #add message
        msg_grid.grid.append(cell)
    
    #stamp msg
    msg_grid.header.stamp = ros_time

    return msg_grid

  #generate occupacy map on RVIZ
  def get_rviz(self, occ_map, ros_time):

    #initialice marker array
    msg_grid = MarkerArray()

    #analice occupacy grid
    for i in range(occ_map.shape[0]):

      for j in range(occ_map.shape[1]):

        #initialize marker
        marker = Marker()

        #set marker
        marker.header.frame_id = "world"
        marker.header.stamp = ros_time
        marker.type = 1
        marker.id = i*occ_map.shape[1] + j

        #define scale
        marker.scale.x = self.d_g
        marker.scale.y = self.d_g
        marker.scale.z = 0.2*self.d_g

        #marker position
        marker.pose.position.x = self.d_g*(i -0.5*self.map.shape[0])
        marker.pose.position.y = self.d_g*(j -0.5*self.map.shape[1])
        marker.pose.position.z = 0.0

        #define marker color
        #obstacle
        if occ_map[i,j] > self.p_occ:
          marker.color.r = 0.0
          marker.color.g = 0.0
          marker.color.b = 0.0
          marker.color.a = 0.5
        
        #unknown point
        if occ_map[i,j] > self.p_free and occ_map[i,j] < self.p_occ:
          marker.color.r = 0.5
          marker.color.g = 0.5
          marker.color.b = 0.5
          marker.color.a = 0.5
        
        #free zone
        if occ_map[i,j] < self.p_free:
          marker.color.r = 1.0
          marker.color.g = 1.0
          marker.color.b = 1.0
          marker.color.a = 0.5
        
        #add marker to list
        msg_grid.markers.append(marker)

    return msg_grid
        

  #static methods
  def st_method():
    pass
