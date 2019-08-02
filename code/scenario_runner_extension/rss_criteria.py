import py_trees
from tools import dist_aux
from tools import other_aux
from srunner.scenariomanager.atomic_scenario_criteria import Criterion
import math


class RssTest(Criterion):

    def __init__(self, actor, filename, optional=False, name="CheckRss", terminate_on_failure=False):
        super(RssTest, self).__init__(name, actor, 0, None, optional, terminate_on_failure)
        world = self.actor.get_world()
        self.vehicles = world.get_actors().filter('vehicle.*')
        self.filename = filename


    def update(self):
        d = dist_aux.evaluate_dist(self.vehicles)
        
        v_ego = self.vehicles[0].get_velocity()
        speed_ego = 3.6 * math.sqrt(v_ego.x**2 + v_ego.y**2 + v_ego.z**2) #km/h
        
        v_lead = self.vehicles[1].get_velocity()
        speed_lead = 3.6 * math.sqrt(v_lead.x**2 + v_lead.y**2 + v_lead.z**2) #km/h
        # print(vel) #km/h
        
        vals = [str(d), str(speed_ego), str(speed_lead)]
        other_aux.write2csv(self.filename, vals)

        return py_trees.common.Status.RUNNING