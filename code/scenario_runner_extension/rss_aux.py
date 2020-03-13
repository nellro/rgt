import carla
import weakref
import inspect
import numpy as np

#==============================================================================
def defineRssParams(x, names):
    rss_params = {}
    for i, name in enumerate(names):
        rss_params[name] = x[i]
    return rss_params
#==============================================================================    
'''
class RssParamsDefault:
    def __init__(self):
        self.alpha_lon_accel_max = 3.5
        self.alpha_lon_brake_max = 8.0
        self.alpha_lon_brake_min = 4.0
        self.alpha_lon_brake_min_correct = 3.0
        self.alpha_lat_accel_max = 0.2
        self.alpha_lat_brake_min = 0.8
        self.lateral_fluctuation_margin = 0.0
        self.response_time = 1.0
'''
#==============================================================================
class RssParamsInit:
    def __init__(self):
        #0
        self.alpha_lon_accel_max_min = 0.0
        self.alpha_lon_accel_max_max = 10.0
        # 1
        self.alpha_lon_brake_max_min = 6
        self.alpha_lon_brake_max_max = 15.0
        # 2
        self.alpha_lon_brake_min_min = 2.0
        self.alpha_lon_brake_min_max = 6.0
        # 3
        self.alpha_lon_brake_min_correct_min = 0.0
        self.alpha_lon_brake_min_correct_max = 2.0
        # 4
        self.alpha_lat_accel_max_min = 0.0
        self.alpha_lat_accel_max_max = 2.0
        # 5
        self.alpha_lat_brake_min_min = 0.0001
        self.alpha_lat_brake_min_max = 3.0
        # 6
        self.lateral_fluctuation_margin_min = 0.0001
        self.lateral_fluctuation_margin_max = 0.5
        # 7
        self.response_time_min = 0.05
        self.response_time_max = 5.0

    def getInit(self, names, **kwargs):
        x = np.array([])
        space = np.empty((0,2))

        for name in names:
            x = np.append(x, kwargs[name])

            val_min = getattr(self, (name + '_min'))
            val_max = getattr(self, (name + '_max'))
            space = np.append(space, np.array([[val_min, val_max]]), axis=0)
        return x, space
# ==============================================================================
def print_dynamics(rss_dynamics):
            print('************************')
            print('RSS DYANMICS:')
            print('Lon accel max: %.3f' % rss_dynamics.alphaLon.accelMax)
            print('Lon brake max: %.3f' % rss_dynamics.alphaLon.brakeMax)
            print('Lon brake min: %.3f' % rss_dynamics.alphaLon.brakeMin)
            print('Lon brake min correct: %.3f' % rss_dynamics.alphaLon.brakeMinCorrect)
            #
            print('Lat accel max: %.3f' % rss_dynamics.alphaLat.accelMax)
            print('Lat brake min: %.3f' % rss_dynamics.alphaLat.brakeMin)
            #
            print('Lat fluct mar: %.3f' % rss_dynamics.lateralFluctuationMargin)
            print('Response time: %.3f' % rss_dynamics.responseTime)
            print('************************')
            
# ==============================================================================
# -- RssSensor --------------------------------------------------------
# ==============================================================================
class RssSensor(object):
    def __init__(self, parent_actor, rss_params):
        self.sensor = None
        self._parent = parent_actor
        self.timestamp = None
        self.response_valid = False
        self.proper_response = None
        self.acceleration_restriction = None
        self.individual_rss_states = None
        self.ego_dynamics_on_route = None
        self.lon_response = None
        self.lat_response_right = None
        self.lat_response_left = None
        self.acceleration_restriction = None
        self.ego_velocity = None
        self.rss_params = rss_params
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.rss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=0.0, z=0.0)), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.

        def check_rss_class(clazz):
            return inspect.isclass(clazz) and "RssSensor" in clazz.__name__

        if not inspect.getmembers(carla, check_rss_class):
            raise RuntimeError('CARLA PythonAPI not compiled in RSS variant, please "make PythonAPI.rss"')
        weak_self = weakref.ref(self)
        self.sensor.visualize_results = True
        self.sensor.listen(lambda event: RssSensor._on_rss_response(weak_self, event))
        self.sensor.routing_target = carla.Transform(carla.Location(x=335, y=153))
        print ("RSS Sensor:")
        print(dir(self.sensor))

        def set_parameters(rss_dynamics, rss_params):
            for key, value in rss_params.items(): 
                if (key == 'alpha_lon_accel_max'):
                    # rss_dynamics.alpha_lon.accel_max = carla.Acceleration(value)
                    rss_dynamics.alphaLon.accelMax = value
                elif (key == 'alpha_lon_brake_max'):
                    # rss_dynamics.alpha_lon.brake_max = carla.Acceleration(value)
                    rss_dynamics.alphaLon.brakeMax = value
                elif (key == 'alpha_lon_brake_min'):
                    # rss_dynamics.alpha_lon.brake_min = carla.Acceleration(value)
                    rss_dynamics.alphaLon.brakeMin = value
                elif (key == 'alpha_lon_brake_min_correct'):
                    # rss_dynamics.alpha_lon.brake_min_correct = carla.Acceleration(value)
                    rss_dynamics.alphaLon.brakeMinCorrect = value
                elif (key == 'alpha_lat_accel_max'):
                    # rss_dynamics.alpha_lat.accel_max = carla.Acceleration(value)
                    rss_dynamics.alphaLat.accelMax = value
                elif (key =='alpha_lat_brake_min'):
                    # rss_dynamics.alpha_lat.brake_min = carla.Acceleration(value)
                    rss_dynamics.alphaLat.brakeMin = value
                elif (key =='lateral_fluctuation_margin'):
                    # rss_dynamics.lateral_fluctuation_margin = carla.Distance(value)
                    rss_dynamics.lateralFluctuationMargin = value
                elif (key =='response_time'):
                    # rss_dynamics.response_time = carla.Duration(value)     
                    rss_dynamics.responseTime = value     
                else:
                    print('WRONG RSS PARAM LABEL')
                    exit()
            return rss_dynamics
        
        rss_dynamics = set_parameters(self.sensor.ego_vehicle_dynamics, self.rss_params)
        self.sensor.ego_vehicle_dynamics = rss_dynamics
        print_dynamics(self.sensor.ego_vehicle_dynamics)

    @staticmethod
    def _on_rss_response(weak_self, response):
        
        self = weak_self()
        if not self:
            return
        self.timestamp = response.timestamp
        self.response_valid = response.response_valid
        self.proper_response = response.proper_response
        # self.lon_response = self.proper_response.longitudinal_response
        # self.lat_response_right = self.proper_response.lateral_response_right
        # self.lat_response_left = self.proper_response.lateral_response_left
        self.acceleration_restriction = response.acceleration_restriction
        self.ego_dynamics_on_route = response.ego_dynamics_on_route

