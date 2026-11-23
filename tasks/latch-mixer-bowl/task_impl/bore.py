"""Signed minimum shank clearance through both faces of a flat-sided tab."""
import math

import numpy as np


def validate_parts(spec, actors):
    tab,pin_body=actors[spec['a']],actors[spec['b']]
    indices=spec['wall_parts']
    if (tab is pin_body or len(indices)!=8 or len(set(indices))!=8 or
            any(type(i) is not int or not 0<=i<len(tab['parts']) for i in indices)):
        raise ValueError('Eight distinct physical flat walls are required')
    pi=spec['pin_part']
    if type(pi) is not int or not 0<=pi<len(pin_body['parts']):
        raise ValueError('Missing locating pin')
    pin=pin_body['parts'][pi]
    if pin['type']!='cylinder' or pin['size'][0]!=pin['size'][1] or spec['pb']!=pin['offset']:
        raise ValueError('The registered pin must be the full cylindrical shank')
    centre=np.asarray(spec['pa'])
    wall_angles=[]
    apothems=[]
    heights=[]
    for i in indices:
        wall=tab['parts'][i]
        if wall['type']!='box' or any(wall['euler_deg'][:2]) or abs(wall['offset'][2]-centre[2])>1e-6:
            raise ValueError('Bore walls must have a common local midplane')
        angle=math.radians(wall['euler_deg'][2])
        normal=np.asarray([math.cos(angle),math.sin(angle)])
        delta=np.asarray(wall['offset'][:2])-centre[:2]
        radial=normal@delta
        if radial<0:
            normal=-normal
            radial=-radial
        apothem=radial-wall['size'][0]/2
        if (apothem<=0 or np.linalg.norm(delta-radial*normal)>1e-6 or
                wall['size'][1]/2 < apothem*math.tan(math.pi/8)):
            raise ValueError('Physical bore flats must meet at the octagon corners')
        wall_angles.append(math.atan2(normal[1],normal[0])%(2*math.pi))
        apothems.append(apothem)
        heights.append(wall['size'][2])
    angles=sorted(wall_angles)
    gaps=np.diff([*angles,angles[0]+2*math.pi])
    if np.max(abs(gaps-math.pi/4))>1e-4 or np.ptp(apothems)>1e-6 or np.ptp(heights)>1e-6:
        raise ValueError('The eight finite walls must form the specified regular bore')


def clearance(axis_center, direction, radius, half_length, bore_center, thickness, flats):
    """Inputs are in the tab frame. Each flat is an outward unit XY normal and apothem."""
    direction=np.asarray(direction,dtype=float)
    direction=direction/np.linalg.norm(direction)
    if abs(direction[2])<1e-6:
        return -1.
    center=np.asarray(axis_center)-np.asarray(bore_center)
    result=math.inf
    for z in (-thickness/2,thickness/2):
        travel=(z-center[2])/direction[2]
        axis_xy=center[:2]+travel*direction[:2]
        # The two tab planes must cross the full shank, including the rim
        # of its slanted section. A tip perched on the surface fails.
        end_margin=half_length-abs(travel)-radius*np.linalg.norm(direction[:2])/abs(direction[2])
        result=min(result,end_margin)
        for normal,apothem in flats:
            n=np.asarray(normal)
            extent=radius*math.hypot(1.,np.dot(n,direction[:2])/direction[2])
            result=min(result,apothem-np.dot(n,axis_xy)-extent)
    return float(result)


def measure(spec, frames, actors):
    from scipy.spatial.transform import Rotation
    tab=actors[spec['a']]
    pin=actors[spec['b']]['parts'][spec['pin_part']]
    center,rot,_=frames[tab['name']]
    mount,mount_rot,_=frames[spec['b']]
    pin_rot=Rotation.from_euler('xyz',pin['euler_deg'],degrees=True).as_matrix()
    axis=rot.T@mount_rot@pin_rot[:,2]
    pin_center=rot.T@(mount+mount_rot@pin['offset']-center)
    walls=[tab['parts'][i] for i in spec['wall_parts']]
    bore_center=np.asarray(spec['pa'])
    flats=[]
    for wall in walls:
        normal=Rotation.from_euler('z',wall['euler_deg'][2],degrees=True).as_matrix()[:2,0]
        offset=np.asarray(wall['offset'])[:2]-bore_center[:2]
        if normal@offset<0:normal=-normal
        flats.append((normal,normal@offset-wall['size'][0]/2))
    return clearance(pin_center,axis,pin['size'][0]/2,pin['size'][2]/2,bore_center,walls[0]['size'][2],flats)
