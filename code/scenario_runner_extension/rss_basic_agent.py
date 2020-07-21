import carla
from agents.navigation.basic_agent import BasicAgent
from scenario_runner_extension.rss_aux import RssSensor

import time
import threading

import matplotlib
import matplotlib.pyplot as plt

# from matplotlib.animation import FuncAnimation

class RssBasicAgent(BasicAgent):
  def __init__(self, vehicle, target_speed, rss_params, restrictor=True):
    
    super(RssBasicAgent, self).__init__(vehicle, target_speed)
    print(rss_params)
    self._rss_sensor = RssSensor(vehicle, rss_params)
    
    self._restrictor = None
    if restrictor:
      self._restrictor = carla.RssRestrictor()

    self._physics_control_static = vehicle.get_physics_control()

  def run_step(self):
    # call basic agent control first
    print("run Rss Agent step")
    control = super(RssBasicAgent, self).run_step()
    
    #-------------------------------------
    if self._restrictor:
      rss_restriction = self._rss_sensor.acceleration_restriction if self._rss_sensor and self._rss_sensor.response_valid else None
      if rss_restriction:

        vehicle_control_rss = self._restrictor.restrict_vehicle_control(control, 
                                        rss_restriction, 
                                        self._rss_sensor.ego_dynamics_on_route, 
                                        self._physics_control_static)

        if not (control == vehicle_control_rss):
         print('RSS restrictor is ON: brake=%.3f, steer=%.3f' % (vehicle_control_rss.brake, vehicle_control_rss.steer))

        control = vehicle_control_rss
    #-------------------------------------
    return control