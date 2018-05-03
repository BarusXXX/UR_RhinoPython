import Rhino.Geometry as rg
import rhinoscriptsyntax as rs
from yoUR import ur_standard as ur
from yoUR import comm

script = ''
robot_ip = '169.254.186.40'
accel = 1.0
vel = 0.1
blend = 0.0

curve = rs.GetObject('Select a tracing curve')
points = rs.CurvePoints(curve)

script += ur.set_tcp_by_angles(0, 0, 0, 3.14, 0, 0)

for point in points:
    plane = rg.Plane(point, rg.Vector3d.XAxis, rg.Vector3d.YAxis)
    script += ur.move_l(plane, accel, vel, blend)


script = comm.concatenate_script(script)
print script
comm.send_script(script,robot_ip)

