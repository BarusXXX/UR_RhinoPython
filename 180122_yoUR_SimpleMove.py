import Rhino.Geometry as rg
from yoUR import ur_standard as ur
from yoUR import comm

script = ''
robot_ip = '169.254.186.40'
origin = rg.Point3d(500,500,300)
plane = rg.Plane(origin, rg.Vector3d.XAxis, rg.Vector3d.YAxis)
accel = 1.0
vel = 0.2
blend = 0.0

script += ur.set_tcp_by_angles(0, 0, 0, 3.14, 0, 0)
script += ur.move_l(plane, accel, vel, blend)
plane.Translate(rg.Vector3d(100,0,0))
script += ur.move_l(plane, accel, vel, blend)
plane.Translate(rg.Vector3d(-100,0,0))
script += ur.move_l(plane, accel, vel, blend)


script = comm.concatenate_script(script)
print script
comm.send_script(script,robot_ip)

