import xml.etree.ElementTree as ET
from box import Box
import numpy as np
from dm_control import mjcf

class XMLParser:
    def __init__(self, config):
        self.config = config
        xml_path = config['MJ_MODEL_PATH']
        mjcf_model = mjcf.from_path(xml_path)
        self.root = mjcf_model.worldbody

        self.movable_bodies = []
        self.static_bodies = []
        self.subgoals = []

        self.x_min = np.inf
        self.x_max = -np.inf
        self.y_min = np.inf
        self.y_max = -np.inf

    def parse(self):
        for body in self.root.find_all('body'):
            pos = body.pos
            pos = np.array(pos, dtype=np.float32)

    
            for geom in body.find_all('geom'):
                
                body_name = body.name
                geom_type = geom.type
                geom_name = geom.name
                
                if geom_type == 'box':
                    sizes = geom.size
                    sizes = np.array(sizes, dtype=np.float32)
                    if self.config['MOVABLE_PREFIX'] in geom_name:
                        print(body_name, geom_name)
                        euler = body.euler
                        euler = np.array(euler, dtype=np.float32)
                        self.movable_bodies.append(Box(pos, sizes, euler))

                        extents = self.movable_bodies[-1].get_extents()
                        # todo: duplicated code
                        self.x_min = min(self.x_min, extents[0][0])
                        self.x_max = max(self.x_max, extents[1][0])
                        self.y_min = min(self.y_min, extents[0][1])
                        self.y_max = max(self.y_max, extents[1][1])
                    else:
                        for name in self.config['STATIC_PREFIX']:
                            if name in geom_name:
                                # See if geom has a pos, if it does override
                                if geom.pos is not None:
                                    pos = geom.pos
                                    pos = np.array(pos, dtype=np.float32)
                                self.static_bodies.append(Box(pos, sizes))
                                extents = self.static_bodies[-1].get_extents()
                                self.x_min = min(self.x_min, extents[0][0])
                                self.x_max = max(self.x_max, extents[1][0])
                                self.y_min = min(self.y_min, extents[0][1])
                                self.y_max = max(self.y_max, extents[1][1])