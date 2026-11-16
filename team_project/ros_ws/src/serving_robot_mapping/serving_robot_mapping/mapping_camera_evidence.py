"""Capture existing RGB-D inputs after confirmed Nav2 shutdown; no map edits."""
import json,time
from pathlib import Path
import numpy as np
from PIL import Image as PILImage


def decode_image(message):
    enc=message.encoding.lower()
    types={'rgb8':('u1',3),'bgr8':('u1',3),'mono8':('u1',1),'32fc1':('f4',1),'16uc1':('u2',1)}
    if enc not in types:raise ValueError('Unsupported image encoding: '+enc)
    dtype,channels=types[enc];dt=np.dtype(dtype).newbyteorder('>' if message.is_bigendian else '<')
    width=message.width*channels;stride=message.step//dt.itemsize
    if stride<width or len(message.data)<message.height*message.step:raise ValueError('Invalid camera image stride')
    a=np.frombuffer(bytes(message.data),dtype=dt).reshape(message.height,stride)[:,:width].copy()
    if channels==3:
        a=a.reshape(message.height,message.width,3)
        if enc=='bgr8':a=a[:,:,::-1]
    return a


def collect_stopped_cameras(node, output, seconds=4.):
    import rclpy
    from sensor_msgs.msg import Image,CameraInfo
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from tf2_ros import Buffer,TransformListener
    output=Path(output)/'stopped_camera_evidence';output.mkdir(exist_ok=True)
    received={};subscriptions=[];buffer=Buffer();listener=TransformListener(buffer,node)
    names=('camera_front','camera_rear_left','camera_rear_right')
    for name in names:
        for kind,cls in [('image',Image),('depth',Image),('camera_info',CameraInfo)]:
            key=name+'/'+kind
            subscriptions.append(node.create_subscription(cls,'/robot_01/'+key,
                lambda msg,key=key:received.update({key:(msg,time.monotonic())}),qos_profile_sensor_data))
    end=time.monotonic()+seconds
    try:
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.05)
            if len(received)==len(names)*3 and all(
                    buffer.can_transform('map',pair[0].header.frame_id,Time())
                    for pair in received.values()):break
        result={'capture_phase':'AFTER_CONFIRMED_NAV2_SHUTDOWN','diagnostic_only':True,'camera_used_to_fill_map':False,'cameras':{}}
        for name in names:
            item={}
            for kind in ('image','depth','camera_info'):
                pair=received.get(name+'/'+kind)
                if pair is None:item[kind]={'available':False};continue
                msg,wall=pair
                info=dict(available=True,frame_id=msg.header.frame_id,
                          stamp_s=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                          receipt_age_s=time.monotonic()-wall,width=msg.width,height=msg.height)
                if kind=='camera_info':info.update(k=list(msg.k),distortion=list(msg.d))
                else:
                    info['encoding']=msg.encoding
                    try:
                        data=decode_image(msg)
                        if kind=='image':PILImage.fromarray(data).save(output/(name+'.png'))
                        else:
                            np.save(output/(name+'_depth.npy'),data)
                            info['finite_positive_pixels']=int((np.isfinite(data)&(data>0)).sum())
                    except ValueError as exc:info['decode_error']=str(exc)
                try:
                    tf=buffer.lookup_transform('map',msg.header.frame_id,Time())
                    t,q=tf.transform.translation,tf.transform.rotation
                    info['map_transform']=dict(translation=[t.x,t.y,t.z],rotation=[q.x,q.y,q.z,q.w])
                except Exception as exc:info['map_transform_unverified']=str(exc)
                item[kind]=info
            result['cameras'][name]=item
        (output/'camera_evidence.json').write_text(json.dumps(result,indent=2))
        return result
    finally:
        for sub in subscriptions:node.destroy_subscription(sub)
        listener.unregister()
