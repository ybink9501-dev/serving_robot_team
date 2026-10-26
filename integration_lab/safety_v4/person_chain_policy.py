"""Pure, offline-testable admission policy for a ROS person-event relay.

Only one existing semantic safety and one existing collision monitor may own the
robot's velocity pipeline. Never spawn new velocity publishers from this LAB.
"""

def evaluate_graph(bridge_publishers, direct_safety_subscribers,
                   target_safety_subscribers, target_publishers,
                   semantic_velocity_publishers, final_velocity_publishers):
    if len(semantic_velocity_publishers) != 1 or 'semantic_safety' not in semantic_velocity_publishers[0]:
        return 'REJECT_SEMANTIC_VELOCITY_OWNER'
    if len(final_velocity_publishers) != 1 or 'collision_monitor' not in final_velocity_publishers[0]:
        return 'REJECT_FINAL_VELOCITY_OWNER'
    if len(bridge_publishers) != 1 or 'gazebo_person_event_bridge' not in bridge_publishers[0]:
        return 'REJECT_PERSON_BRIDGE_MISSING_OR_DUPLICATE'
    if len(direct_safety_subscribers) > 0:
        if len(direct_safety_subscribers) != 1:
            return 'REJECT_DUPLICATE_SAFETY_NODES'
        return 'DIRECT_READY'
    if len(target_safety_subscribers) != 1:
        return 'REJECT_SAFETY_INPUT_NOT_FOUND'
    if target_publishers:
        return 'EXTERNAL_SOURCE_PRESENT_NO_LAB_RELAY'
    return 'RELAY_ALLOWED'
