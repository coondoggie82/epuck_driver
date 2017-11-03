#! /usr/bin/env python

#
# Convert epuck proximity range sensors measurements to coarse laser scans.
#
# The virtual laser scanner is position at the top center point of the robot
# at the same height as the proximity sensors. The virtual scanner has 360 
# degree angular field of view.
#
# ROS Parameters:
#   /range_to_laserscan_node/epuck_name
#     - Topic namespace for the e-puck. This node will subscribe to
#       the 8 proximity sensors topics /<epuck_name>/proximity<n>.
#       Default: "epuck".
#   /range_to_laserscan_node/angle_inc
#     - The virtual laser scanner angle increment (radians).
#       Default: 10 degrees in radians.
#

import math
import time
from copy import deepcopy
import rospy
from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan

#
# The e-puck relevant dimensions.
#
EPuckRadius     = 0.03    # e-puck 3cm radius (meters)
ProximityHeight = 0.034   # height of proximity sensors from base (meters)

#
# Sensor identifiers.
#
# Order is important, so a list of keys to proximity sensor dictionary is
# needed.
# 
# For the e-puck, x points forward, y to the left. By sensors ids, the sensors
# are arranged clockwise. We want couter-clockwise.
#
ProximityIds = [
  'proximity7', 'proximity6', 'proximity5', 'proximity4',
  'proximity3', 'proximity2', 'proximity1', 'proximity0',
]

#
# Proximity sensor dictionary.
#   Key: sensorId
#   Data:
#     cob     center of infrared beam (radians)
#     range   most recent distance measurement (meters)
#
# Note that the sensors are not evenly spaced.
#
ProximitySensors = {
    'proximity0': {'cob': math.radians(350.0), 'range': None}, # +30
    'proximity1': {'cob': math.radians(320.0), 'range': None}, # +50
    'proximity2': {'cob': math.radians(270.0), 'range': None}, # +70
    'proximity3': {'cob': math.radians(200.0), 'range': None}, # +40
    'proximity4': {'cob': math.radians(160.0), 'range': None}, # +70
    'proximity5': {'cob': math.radians( 90.0), 'range': None}, # +50
    'proximity6': {'cob': math.radians( 40.0), 'range': None}, # +30
    'proximity7': {'cob': math.radians( 10.0), 'range': None}, # +10
}

##
class RangeToLaserScanNode:
  ##
  def __init__(self, epuck_name = 'epuck', angle_inc = math.radians(10.0)):
    self.epuckName = epuck_name # epuck name and subscribed topic namespace
    self.angleInc  = angle_inc  # scanner angle increment in radians

    # range measurement with r is the range, m is the maximum
    self.meas = lambda r, m: r if (r is not None) else 2.0 * m

    print 'DBG: epuck_name =', self.epuckName
    print 'DBG: angle_inc  =', math.degrees(self.angleInc)

  ##
  def __del__(self):
    pass

  ##
  def subscribeToTopics(self):
    self.sub = []
    for sensorId in ProximityIds:
      topic = '/' + self.epuckName + '/' + sensorId
      self.sub.append(rospy.Subscriber(topic, Range,
        callback=self.updateRangeData, callback_args=sensorId))

  ##
  def advertisePublishers(self):
    self.msgScanFixed = None
    self.proximityToLaserInterp = []
    self.pubLaserScan = rospy.Publisher('/scan', LaserScan, queue_size=5)

  ##
  def updateRangeData(self, msgRange, sensorId):
    #print 'DBG:', sensorId
    if ProximitySensors.has_key(sensorId):
      ProximitySensors[sensorId]['range'] = msgRange.range
      if self.msgScanFixed is None:
        self.rangeMin = msgRange.min_range
        self.rangeMax = msgRange.max_range
        self.initScanFixed(msgRange)
        #print 'DBG: First Range Message'
        #print 'DBG:', msgRange,
        #print 'DBG: Fixed Scan Message'
        #print 'DBG:', self.msgScanFixed

  ##
  def initScanFixed(self, msgRange):
    # First calculate the range sensor to laser scanner interpolation parameters
    for deg in range(0, 360, int(math.degrees(self.angleInc))):
      a = math.radians(float(deg))
      self.proximityToLaserInterp.append(self.calcIterpParams(a))
      #print 'DBG: Iterpolation:', deg, self.proximityToLaserInterp[-1]

    # Now set the quasi-fixed laser scan message
    self.msgScanFixed = LaserScan()

    # initialize quasi-fixed header
    self.msgScanFixed.header.seq          = 0
    self.msgScanFixed.header.stamp.secs   = 0
    self.msgScanFixed.header.stamp.nsecs  = 0
    self.msgScanFixed.header.frame_id     = self.epuckName + '/scan'

    # scanner fixed start, end, and increment angles (radians)
    self.msgScanFixed.angle_min       = 0.0
    self.msgScanFixed.angle_max       = 2.0 * math.pi - self.angleInc
    self.msgScanFixed.angle_increment = self.angleInc

    # scanner fixed measurement and scan intervals (seconds)
    self.msgScanFixed.time_increment  = 0.0
    self.msgScanFixed.scan_time       = 0.1

    # fixed scanner minimum and maximum range (meters)
    self.msgScanFixed.range_min = EPuckRadius + msgRange.min_range
    self.msgScanFixed.range_max = EPuckRadius + msgRange.max_range

    # measurements
    self.msgScanFixed.ranges      = []  # range (meters)
    self.msgScanFixed.intensities = []  # not used

  ##
  def calcIterpParams(self, a):
    n = len(ProximityIds)

    for i in range(0, n):
      j = (i + 1) % n

      id0 = ProximityIds[i]
      id1 = ProximityIds[j]

      cob0 = ProximitySensors[id0]['cob']
      cob1 = ProximitySensors[id1]['cob']

      #
      # Scan angle is coincident to a range sensor's beam center.
      #
      if cob0 == a:
        return {id0: 1.0, id1: 0.0}

      #
      # Scan angle is between two range sensors' beam centers.
      # Example: With a = 20, cob0 = 10, cob1 = 40
      #   gap = 30
      #   wt0 = 1 - 10/30 = 1 - 0.333 = 0.667
      #   wt1 = 1 - 20/30 = 1 - 0.667 = 0.333
      #
      elif cob0 < a and cob1 > a:
        wt0, wt1 = self.weights(a, cob0, cob1)
        return {id0: wt0, id1: wt1}

    #
    # Special case: Sensor wrap around x-axis
    #
    id0 = ProximityIds[0]   # first range sensor
    id1 = ProximityIds[-1]  # last range sensor

    cob0 = ProximitySensors[id0]['cob'] 
    cob1 = ProximitySensors[id1]['cob']

    #
    # Scan angle between 0 and the first sensor's beam center.
    #
    if a < cob0:
      cob1 -= 2.0 * math.pi
      wt0, wt1 = self.weights(a, cob0, cob1)
      return {id0: wt0, id1: wt1}

    #
    # Scan angle between last sensor's beam center and 0.
    #
    else: # a > cob1
      cob0 += 2.0 * math.pi
      wt0, wt1 = self.weights(a, cob0, cob1)
      return {id0: wt0, id1: wt1}

  ##
  def weights(self, val, val0, val1):
    gap = math.fabs(val1 - val0)
    wt0 = 1.0 - math.fabs(val0 - val) / gap
    wt1 = 1.0 - math.fabs(val1 - val) / gap
    return (wt0, wt1)

  ##
  def publish(self):
    # have not received the first proximity range measurement
    if self.msgScanFixed is None:
      return

    # "fixed" laser scan measurement's header hold convenient accounting info
    self.msgScanFixed.header.seq += 1

    # make a deep copy of the fixed message (don't want to alter the source)
    msgScan = deepcopy(self.msgScanFixed)

    # time stamp
    msgScan.header.stamp = rospy.Time.now()

    #
    # "Scan"
    #
    for interp in self.proximityToLaserInterp:
      id0, id1 = interp.iterkeys()      # range sensor ids
      wt0, wt1 = interp.itervalues()    # range sensor weights

      # range sensor most recent measurements
      r0 = self.meas(ProximitySensors[id0]['range'], self.rangeMax)
      r1 = self.meas(ProximitySensors[id1]['range'], self.rangeMax)
      
      # interpolated scan measurement
      msgScan.ranges.append(EPuckRadius + wt0 * r0 + wt1 * r1)

    # and publish
    self.pubLaserScan.publish(msgScan)

  ##
  def run(self):
    self.subscribeToTopics()
    self.advertisePublishers()

    rospy.init_node("range_to_laserscan_node", anonymous=True)

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
      self.publish()
      rate.sleep()

##
if __name__ == '__main__':
  nodenamespace = "/range_to_laserscan_node"
  #
  # ROS Parameter Server parameters for this node
  #
  epuck_name = rospy.get_param(nodenamespace + "/epuck_name", "epuck")
  angle_inc  = rospy.get_param(nodenamespace + "/angle_inc", math.radians(10.0))

  node = RangeToLaserScanNode(epuck_name=epuck_name, angle_inc=angle_inc)

  try:
    node.run()
  except rospy.ROSInterruptException:
    pass

