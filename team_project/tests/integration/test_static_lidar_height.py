"""Only collision surfaces intersecting the LiDAR plane may guide registration."""
from types import SimpleNamespace
from cafe_autonomy.dynamic_obstacles import DynamicObstacles


def test_top_and_floor_geometry_do_not_bias_lidar_registration(tmp_path):
    path = tmp_path / 'world.sdf'
    path.write_text("""<sdf version="1.9"><world name="cafe">
    <model name="furniture"><static>true</static><link name="body">
      <collision name="top"><pose>0 0 .73 0 0 0</pose><geometry><box><size>.8 .8 .06</size></box></geometry></collision>
      <collision name="foot"><pose>0 0 .05 0 0 0</pose><geometry><box><size>.4 .4 .04</size></box></geometry></collision>
      <collision name="leg"><pose>0 0 .35 0 0 0</pose><geometry><cylinder><radius>.09</radius><length>.7</length></cylinder></geometry></collision>
      <collision name="chair"><pose>.65 0 .275 0 0 0</pose><geometry><box><size>.45 .45 .55</size></box></geometry></collision>
    </link></model>
    <model name="person"><static>false</static><link name="body"><collision name="body">
      <pose>2 0 .8 0 0 0</pose><geometry><cylinder><radius>.22</radius><length>1.6</length></cylinder></geometry>
    </collision></link></model></world></sdf>""")
    world = SimpleNamespace(static_shapes=[], static_lidar_height=.30)
    DynamicObstacles.load_static_world(world, path)
    assert len(world.static_shapes) == 2
    assert {s[3] for s in world.static_shapes} == {'box', 'circle'}
    assert all(not (s[3] == 'box' and s[4] == .4) for s in world.static_shapes)
