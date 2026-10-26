import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
import yaml

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from checks import check,occupancy_signature,speed_config_ok

class OfflineTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        v3=self.root/'v3';out=v3/'generated';out.mkdir(parents=True)
        self.world=self.root/'cafe.world.sdf';self.world.write_text('team test sdf')
        self.map=self.root/'map.yaml';self.raster=self.root/'map.pgm'
        self.raster.write_bytes(b'P5\n32 32\n255\n'+bytes([255])*1024)
        self.map.write_text(yaml.safe_dump({'image':'map.pgm','resolution':.05,'origin':[0,0,0]}))
        self.runtime=self.root/'runtime.yaml'
        self.runtime.write_text(yaml.safe_dump({'max_linear_mps':.25,'max_angular_rps':.8}))
        self.arrival=self.root/'arrival.yaml';self.arrival.write_text(yaml.safe_dump({'arrival':{
            'final_error_m':.01,'detection_radius_m':.05,'yaw_tolerance_rad':.25,'final_verify_timeout_s':10}}))
        self.layout=v3/'layout.yaml'
        self.layout.write_text(yaml.safe_dump({'robot_id':'robot_01','map_revision':2,
            'frame_alignment':{'mode':'sdf_aligned_lab_only'},
            'geometry':{'robot_clearance_radius_m':.35},'serving':{
                'loading_mode':'manual','completion_mode':'manual',
                'approach_selection':'shortest_planned_path',
                'active_goal_timeout_s':240,'planner_timeout_s':12}}))
        poses=[{'entity_id':k,'pose':{'x':.4+i*.1,'y':.4,'yaw':0.}}
            for i,k in enumerate(['kitchen','counter_pickup'])]
        choices={}
        for i in range(1,5):
            table=f'table_{i:02d}'
            item={'id':f'{table}_top','side':'top','pose':{'x':.7,'y':.8,'yaw':0},'enabled':True,'clearance_ok':True}
            choices[table]=[item,{'id':f'{table}_left','side':'left','enabled':False,'clearance_ok':False}]
            poses.append({'entity_id':item['id'],'pose':item['pose']})
        self.wps=out/'active_waypoints.yaml';self.wps.write_text(yaml.safe_dump({'map_revision':2,'waypoints':poses}))
        sha=lambda x:hashlib.sha256(x.read_bytes()).hexdigest()
        snap={'robot_id':'robot_01','map_revision':2,
           'world':str(self.world),'world_sha256':sha(self.world),
           'map':str(self.map),'map_yaml_sha256':sha(self.map),
           'map_raster':str(self.raster),'map_raster_sha256':sha(self.raster),
           'layout_sha256':sha(self.layout),'waypoints_sha256':sha(self.wps),
           'map_info':{'width':32,'height':32,'resolution':.05,'origin':[0,0,0]},
           'occupancy_binary_sha256':occupancy_signature(32,32,bytes([255])*1024),
           'choices':choices}
        self.manifest=out/'manifest.json';self.manifest.write_text(json.dumps(snap))
        self.config=self.root/'config.yaml'
        self.cfg={'schema_version':1,'robot_id':'robot_01','ros_domain_id':99,
              'serving_v3_dir':str(v3),'runtime_limits_yaml':str(self.runtime),
              'arrival_yaml':str(self.arrival),
              'speed':{'max_linear_mps':.25,'max_angular_radps':.8,
                       'default_profile':'normal','topic':'/robot_01/speed_limit',
                       'profiles':{'low':.1,'normal':.15,'high':.2}}}
        self.write_cfg()
    def tearDown(self):self.tmp.cleanup()
    def write_cfg(self):self.config.write_text(yaml.safe_dump(self.cfg))
    def result(self):return check(self.config)
    def test_valid_static_snapshot(self):
        r=self.result();self.assertTrue(r['ready'],r)
        self.assertEqual(len(r['checks']),7)
    def test_tampered_world_rejected(self):
        self.world.write_text('moved table')
        self.assertFalse(self.result()['ready'])
    def test_tampered_waypoint_rejected(self):
        self.wps.write_text(self.wps.read_text()+'\n# changed')
        self.assertFalse(self.result()['ready'])
    def test_map_occupancy_change_rejected(self):
        p=self.manifest;data=json.loads(p.read_text())
        self.raster.write_bytes(b'P5\n32 32\n255\n'+bytes([0])+bytes([255])*1023)
        data['map_raster_sha256']=hashlib.sha256(self.raster.read_bytes()).hexdigest()
        p.write_text(json.dumps(data))
        self.assertFalse(self.result()['ready'])
    def test_out_of_bounds_waypoint_rejected(self):
        p=yaml.safe_load(self.wps.read_text());p['waypoints'][0]['pose']['x']=-1.
        self.wps.write_text(yaml.safe_dump(p));snap=json.loads(self.manifest.read_text())
        snap['waypoints_sha256']=hashlib.sha256(self.wps.read_bytes()).hexdigest()
        self.manifest.write_text(json.dumps(snap))
        self.assertFalse(self.result()['ready'])
    def test_no_enabled_approach_rejected(self):
        snap=json.loads(self.manifest.read_text());snap['choices']['table_02'][0]['enabled']=False
        self.manifest.write_text(json.dumps(snap));self.assertFalse(self.result()['ready'])
    def test_speed_above_safety_rejected(self):
        self.cfg['speed']['profiles']['high']=.3;self.write_cfg()
        self.assertFalse(self.result()['ready'])
    def test_auto_loading_warns(self):
        l=yaml.safe_load(self.layout.read_text());l['serving']['loading_mode']='auto'
        self.layout.write_text(yaml.safe_dump(l));snap=json.loads(self.manifest.read_text())
        snap['layout_sha256']=hashlib.sha256(self.layout.read_bytes()).hexdigest()
        self.manifest.write_text(json.dumps(snap));r=self.result()
        self.assertTrue(r['ready']);self.assertTrue(any('auto-confirm' in x for x in r['warnings']))
    def test_wrong_robot_rejected(self):
        self.cfg['robot_id']='robot_02';self.write_cfg();self.assertFalse(self.result()['ready'])

if __name__=='__main__':unittest.main()
