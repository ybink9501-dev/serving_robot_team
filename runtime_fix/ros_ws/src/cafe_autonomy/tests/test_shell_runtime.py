"""Offline shell startup regression test: mocks ROS setup, not a Gazebo integration test."""
import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[4]

class ShellStartup(unittest.TestCase):
    def test_nounset_does_not_break_ros_setup(self):
        # Mocks both ROS setup files deliberately referencing an unset variable.
        # Before this fix all four scripts exit immediately at setup.bash.
        with tempfile.TemporaryDirectory(prefix='cafe_bootstrap_') as td:
            root = Path(td)
            tools = root / 'tools'
            tools.mkdir()
            setup = root / 'fake_ros_setup.bash'
            setup.write_text('test -z "$AMENT_TRACE_SETUP_FILES"\n')
            inst = root / 'ros_ws/install'
            inst.mkdir(parents=True)
            (inst / 'setup.bash').write_text('test -z "$AMENT_TRACE_SETUP_FILES"\n')
            bin_dir = root / 'bin'
            bin_dir.mkdir()
            def exe(name, code):
                p = bin_dir / name
                p.write_text('#!/usr/bin/env bash\n' + code)
                p.chmod(0o755)
            exe('ros2', '''case "$1 $2" in
  'pkg prefix')
    if [[ "$3" == 'cafe_autonomy' ]];then echo "$MOCK_ROOT/ros_ws/install/cafe_autonomy";fi;;
  'launch cafe_autonomy')
    if [[ "$3" == '01_world.launch.py' ]];then exec sleep 1;fi;;
  'topic info') exit 0;;
  'run nav2_map_server')
    touch "${MOCK_ROOT}/results/maps/$(basename "$5").pgm";;
esac
exit 0
''')
            exe('gz', '''if [[ "$1 $2" == 'model --list' ]];then echo 'cafe_structure';fi
exit 0
''')
            exe('colcon', 'exit 0\n')
            exe('pgrep', 'exit 1\n')
            # check.sh should actually get as far as acceptance; mock its body.
            (tools / 'acceptance.py').write_text("print('ACCEPTANCE_REACHED')\n")
            (root / 'results/maps').mkdir(parents=True)
            env = dict(os.environ)
            env.pop('AMENT_TRACE_SETUP_FILES',None)
            env['MOCK_ROOT'] = str(root)
            env['PATH'] = str(bin_dir) + ':' + env.get('PATH','')
            for name in ('preflight.sh','check.sh','run_all.sh','save_mapping_map.sh'):
                original = (PROJECT / 'tools' / name).read_text()
                mock = original.replace('/opt/ros/jazzy/setup.bash',str(setup))
                (tools/name).write_text(mock)
            for name in ('preflight.sh','check.sh','run_all.sh'):
                with self.subTest(script=name):
                    p = subprocess.run(['bash',str(tools/name)],env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=8)
                    self.assertEqual(p.returncode,0,p.stdout)
                    self.assertNotIn('unbound variable',p.stdout)
            self.assertIn('ACCEPTANCE_REACHED',subprocess.run(['bash',str(tools/'check.sh')],env=env,capture_output=True,text=True).stdout)

if __name__ == '__main__':
    unittest.main()
