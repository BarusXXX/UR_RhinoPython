import Rhino.Geometry as rg
import rhinoscriptsyntax as rs
from yoUR import ur_standard as ur
from yoUR import comm


def get_pose_text():
    packet = comm.listen_to_robot('10.0.0.12')
    pose = packet['pose']
    text = ''
    for element in pose:
        text += str(round(element,10)) + ', '
    text = text[:-2]
    text += '\n'
    return text


rs.MessageBox('Go to the first point and press OK',0)
text = get_pose_text()

print text