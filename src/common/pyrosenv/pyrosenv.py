try:
    import rospy
    import roslaunch
    import rosgraph
    import rosnode
except ImportError:
    import pyros_setup
    pyros_setup.configurable_import().configure().activate()
    import rospy
    import roslaunch
    import rosgraph
    import rosnode
