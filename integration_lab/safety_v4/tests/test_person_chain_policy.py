from person_chain_policy import evaluate_graph


def decide(src=['/gazebo_person_event_bridge'], direct=[], target=['/robot_01/semantic_safety'],
           existing=[], semantic=['/robot_01/semantic_safety'], final=['/robot_01/collision_monitor']):
    return evaluate_graph(src,direct,target,existing,semantic,final)


def test_relay():
    assert decide()=='RELAY_ALLOWED'


def test_direct():
    assert decide(direct=['/robot_01/semantic_safety'])=='DIRECT_READY'


def test_already_has_input():
    assert decide(existing=['/robot_01/person_detector'])=='EXTERNAL_SOURCE_PRESENT_NO_LAB_RELAY'


def test_reject_missing_safety():
    assert decide(target=[])=='REJECT_SAFETY_INPUT_NOT_FOUND'


def test_reject_competing_velocity_publishers():
    assert decide(semantic=['/robot_01/semantic_safety','/unknown'])=='REJECT_SEMANTIC_VELOCITY_OWNER'
    assert decide(final=['/robot_01/collision_monitor','/unknown'])=='REJECT_FINAL_VELOCITY_OWNER'


def test_reject_missing_bridge():
    assert decide(src=[])=='REJECT_PERSON_BRIDGE_MISSING_OR_DUPLICATE'
