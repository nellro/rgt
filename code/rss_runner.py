#!/usr/bin/env python

from __future__ import print_function

import traceback
import argparse
from argparse import RawTextHelpFormatter
from datetime import datetime
from distutils.version import LooseVersion
import importlib
import inspect
import os
import signal
import sys
import time
import pkg_resources

import carla

# import pdb

### SR imports
sys.path.append(os.getenv('ROOT_SCENARIO_RUNNER'))

from srunner.scenarioconfigs.openscenario_configuration import OpenScenarioConfiguration
from srunner.scenarioconfigs.route_scenario_configuration import RouteScenarioConfiguration
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider, CarlaActorPool
from srunner.scenariomanager.scenario_manager import ScenarioManager

from srunner.scenarios.open_scenario import OpenScenario
from srunner.scenarios.route_scenario import RouteScenario
from srunner.tools.scenario_config_parser import ScenarioConfigurationParser
from srunner.tools.route_parser import RouteParser

### SR Scenarios
from srunner.scenarios.control_loss import ControlLoss
from srunner.scenarios.follow_leading_vehicle import FollowLeadingVehicle, FollowLeadingVehicleWithObstacle
from srunner.scenarios.maneuver_opposite_direction import ManeuverOppositeDirection
from srunner.scenarios.no_signal_junction_crossing import NoSignalJunctionCrossing
from srunner.scenarios.object_crash_intersection import VehicleTurningRight, VehicleTurningLeft
from srunner.scenarios.object_crash_vehicle import StationaryObjectCrossing, DynamicObjectCrossing
from srunner.scenarios.opposite_vehicle_taking_priority import OppositeVehicleRunningRedLight
from srunner.scenarios.other_leading_vehicle import OtherLeadingVehicle
from srunner.scenarios.signalized_junction_left_turn import SignalizedJunctionLeftTurn
from srunner.scenarios.signalized_junction_right_turn import SignalizedJunctionRightTurn
from srunner.scenarios.change_lane import ChangeLane
from srunner.scenarios.cut_in import CutIn

### RGT Functions
from tools import annealing
from tools import robustness

from scenario_runner_extension.rss_aux import defineRssParams
from scenario_runner_extension.rss_aux import RssParamsInit
from scenario_runner_extension.rss_config_parser import parse_rss_scenario_configuration 

### RGT Scenarios
from scenarios.rss_opposite_vehicle_taking_priority import RssOppositeVehicleRunningRedLight
from scenarios.rss_lvdad import RssLVDAD
from scenarios.rss_follow_leading_vehicle import RssFollowLeadingVehicle
from scenarios.rss_pov_unprotected_left import RssPovUnprotectedLeft


class ScenarioRunner(object):
  
    """
    This is the core scenario runner module. It is responsible for
    running (and repeating) a single scenario or a list of scenarios.

    Usage:
    scenario_runner = ScenarioRunner(args)
    scenario_runner.run()
    del scenario_runner
    """

    ego_vehicles = []

    # Tunable parameters
    client_timeout = 10.0  # in seconds
    wait_for_world = 20.0  # in seconds
    frame_rate = 20.0      # in Hz

    # CARLA world and scenario handlers
    world = None
    manager = None

    additional_scenario_module = None

    # agent_instance = None
    # module_agent = None

    def __init__(self, args):
        """
        Setup CARLA client and world
        Setup ScenarioManager
        """
        self._args = args

        self.filename_traj = self._args .filename_traj

        if args.timeout:
            self.client_timeout = float(self._args.timeout)

        # First of all, we need to create the client that will send the requests
        # to the simulator. Here we'll assume the simulator is accepting
        # requests in the localhost at port 2000.
        self.client = carla.Client(args.host, int(args.port))
        self.client.set_timeout(self.client_timeout)

        # dist = pkg_resources.get_distribution("carla")
        # if LooseVersion(dist.version) < LooseVersion('0.9.8'):
        #     raise ImportError("CARLA version 0.9.8 or newer required. CARLA version found: {}".format(dist))

        # Load additional scenario definitions, if there are any
        # If something goes wrong an exception will be thrown by importlib (ok here)
        if self._args.additionalScenario != '':
            module_name = os.path.basename(args.additionalScenario).split('.')[0]
            sys.path.insert(0, os.path.dirname(args.additionalScenario))
            self.additional_scenario_module = importlib.import_module(module_name)

        # # Load agent if requested via command line args
        # # If something goes wrong an exception will be thrown by importlib (ok here)
        # if self._args.agent is not None:
        #     module_name = os.path.basename(args.agent).split('.')[0]
        #     sys.path.insert(0, os.path.dirname(args.agent))
        #     self.module_agent = importlib.import_module(module_name)

        # Create the ScenarioManager
        self.manager = ScenarioManager(self._args.debug, self._args.timeout)

        # Create signal handler for SIGINT
        self._shutdown_requested = False
        signal.signal(signal.SIGHUP, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        self._start_wall_time = datetime.now()

    def destroy(self):
        """
        Cleanup and delete actors, ScenarioManager and CARLA world
        """

        self._cleanup()
        if self.manager is not None:
            del self.manager
        if self.world is not None:
            del self.world
        if self.client is not None:
            del self.client

    def _signal_handler(self, signum, frame):
        """
        Terminate scenario ticking when receiving a signal interrupt
        """
        self._shutdown_requested = True
        if self.manager:
            self.manager.stop_scenario()
            self._cleanup()
            if not self.manager.get_running_status():
                raise RuntimeError("Timeout occured during scenario execution")

    def _get_scenario_class_or_fail(self, scenario):
        """
        Get scenario class by scenario name
        If scenario is not supported or not found, exit script
        """

        if scenario in globals():
            return globals()[scenario]

        for member in inspect.getmembers(self.additional_scenario_module):
            if scenario in member and inspect.isclass(member[1]):
                return member[1]

        print("Scenario '{}' not supported ... Exiting".format(scenario))
        sys.exit(-1)

    def _cleanup(self):
        """
        Remove and destroy all actors
        """

        # Reset to asynchronous mode
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        self.world.apply_settings(settings)

        self.client.stop_recorder()
        self.manager.cleanup()

        CarlaDataProvider.cleanup()
        CarlaActorPool.cleanup()

        for i, _ in enumerate(self.ego_vehicles):
            if self.ego_vehicles[i]:
                if not self._args.waitForEgo:
                    print("Destroying ego vehicle {}".format(self.ego_vehicles[i].id))
                    self.ego_vehicles[i].destroy()
                self.ego_vehicles[i] = None
        self.ego_vehicles = []

        # if self.agent_instance:
        #     self.agent_instance.destroy()
        #     self.agent_instance = None

    def _prepare_ego_vehicles(self, ego_vehicles):
        """
        Spawn or update the ego vehicles
        """

        if not self._args.waitForEgo:
            for vehicle in ego_vehicles:
                self.ego_vehicles.append(CarlaActorPool.setup_actor(vehicle.model,
                                                                    vehicle.transform,
                                                                    vehicle.rolename,
                                                                    True,
                                                                    color=vehicle.color,
                                                                    actor_category=vehicle.category))
        else:
            ego_vehicle_missing = True
            while ego_vehicle_missing:
                self.ego_vehicles = []
                ego_vehicle_missing = False
                for ego_vehicle in ego_vehicles:
                    ego_vehicle_found = False
                    carla_vehicles = CarlaDataProvider.get_world().get_actors().filter('vehicle.*')
                    for carla_vehicle in carla_vehicles:
                        if carla_vehicle.attributes['role_name'] == ego_vehicle.rolename:
                            ego_vehicle_found = True
                            self.ego_vehicles.append(carla_vehicle)
                            break
                    if not ego_vehicle_found:
                        ego_vehicle_missing = True
                        break

            for i, _ in enumerate(self.ego_vehicles):
                self.ego_vehicles[i].set_transform(ego_vehicles[i].transform)

        # sync state
        CarlaDataProvider.get_world().tick()

    def _analyze_scenario(self, config):
        """
        Provide feedback about success/failure of a scenario
        """

        current_time = str(datetime.now().strftime('%Y-%m-%d-%H-%M-%S'))
        junit_filename = None
        config_name = config.name
        if self._args.outputDir != '':
            config_name = os.path.join(self._args.outputDir, config_name)
        if self._args.junit:
            junit_filename = config_name + current_time + ".xml"
        filename = None
        if self._args.file:
            filename = config_name + current_time + ".txt"

        if not self.manager.analyze_scenario(self._args.output, filename, junit_filename):
            print("All scenario tests were passed successfully!")
        else:
            print("Not all scenario tests were successful")
            if not (self._args.output or filename or junit_filename):
                print("Please run with --output for further information")

    def _load_and_wait_for_world(self, town, ego_vehicles=None):
        """
        Load a new CARLA world and provide data to CarlaActorPool and CarlaDataProvider
        """

        if self._args.reloadWorld:
            self.world = self.client.load_world(town)
        else:
            # if the world should not be reloaded, wait at least until all ego vehicles are ready
            ego_vehicle_found = False
            if self._args.waitForEgo:
                while not ego_vehicle_found and not self._shutdown_requested:
                    vehicles = self.client.get_world().get_actors().filter('vehicle.*')
                    for ego_vehicle in ego_vehicles:
                        ego_vehicle_found = False
                        for vehicle in vehicles:
                            if vehicle.attributes['role_name'] == ego_vehicle.rolename:
                                ego_vehicle_found = True
                                break
                        if not ego_vehicle_found:
                            print("Not all ego vehicles ready. Waiting ... ")
                            time.sleep(1)
                            break

        self.world = self.client.get_world()

        CarlaActorPool.set_client(self.client)
        CarlaActorPool.set_world(self.world)
        CarlaDataProvider.set_world(self.world)

        # Wait for the world to be ready
        if self.world.get_settings().synchronous_mode:
            self.world.tick()
        else:
            self.world.wait_for_tick()

        if CarlaDataProvider.get_map().name != town and CarlaDataProvider.get_map().name != "OpenDriveMap":
            print("The CARLA server uses the wrong map: {}".format(CarlaDataProvider.get_map().name))
            print("This scenario requires to use map: {}".format(town))
            return False

        return True

    def _prepare_camera(self, config):
        spectator = self.world.get_spectator()
        spectator.set_transform(config.camera.transform)

    def load_and_run_scenario(self, args, config, scenario):
        """
        Load and run the given scenario
        """
        try:
            # Load scenario and run it
            self.manager.load_scenario(scenario)
            self.manager.run_scenario()
            print("Ok Run scenario")

            # Provide outputs if required
            self._analyze_scenario(config)

            # Stop scenario and cleanup
            self.manager.stop_scenario()
            scenario.remove_all_actors()
            result = True
        except Exception as e:              # pylint: disable=broad-except
            traceback.print_exc()
            print(e)
            result = False
        
        self._cleanup()
        return result

    #### same as _load_and_run_scenario
    def _simulate(self, config, args, rss_params):
        # Prepare scenario
        print("Preparing scenario: " + config.name)
        
        # file = open(self.filename_traj, 'w')  
        # file.close()
        result = False
        scenario_class = self._get_scenario_class_or_fail(config.type)

        while not result:
            try:
                # self.load_world(args, config.town)
                self._load_and_wait_for_world(config.town)
                self.manager = ScenarioManager(self.world, args.debug)   
                CarlaActorPool.set_world(self.world)
                self._prepare_ego_vehicles(config.ego_vehicles)
                self._prepare_camera(config) 
                scenario = scenario_class(self.world, 
                                            rss_params, 
                                            self.filename_traj, 
                                            self.ego_vehicles, 
                                            config, 
                                            args.randomize, 
                                            args.debug)

                result = True
            except Exception as exception:
                print("The scenario cannot be loaded")
                traceback.print_exc()
                print(exception)
                self._cleanup()
                pass
        

        # # Set the appropriate weather conditions
        self.world.set_weather(config.weather)
        self.load_and_run_scenario(args, config, scenario)

        # rob = robustness.getRobustness(self.filename_traj)

        return 

    ### same as _run_scenarios
    def run(self, args): 
        scenario_config_file = ScenarioConfigurationParser.find_scenario_config(args.scenario, 
                                                                                args.configFile) # xml file 

        if scenario_config_file is None:
            print("Configuration for scenario {} cannot be found!".format(self._args.scenario))
            return                                                                  

        scenario_configurations = parse_rss_scenario_configuration(scenario_config_file, args.scenario)
        config = scenario_configurations[0] # since we work only with one scenario!

        search_names = ['alpha_lon_accel_max', 
                        'alpha_lon_brake_max', 
                        'alpha_lon_brake_min', 
                        'response_time', 
                        'alpha_lat_accel_max', 
                        'alpha_lat_brake_min', 
                        'lateral_fluctuation_margin', 
                        'alpha_lon_brake_min_correct']

        ## initial values
        alpha_lon_accel_max = 3.5
        response_time       = 1.0
        alpha_lon_brake_max = 6.0
        alpha_lon_brake_min = 3.5 
        alpha_lat_accel_max = 0.1
        alpha_lat_brake_min = 0.1
        lateral_fluctuation_margin = 0.1
        alpha_lon_brake_min_correct = 1.0

        x0, searchSpace = RssParamsInit().getInit(search_names,
                                                alpha_lon_accel_max = alpha_lon_accel_max,
                                                response_time = response_time, 
                                                alpha_lon_brake_max = alpha_lon_brake_max, 
                                                alpha_lon_brake_min = alpha_lon_brake_min, 
                                                alpha_lat_accel_max = alpha_lat_accel_max, 
                                                alpha_lat_brake_min = alpha_lat_brake_min,
                                                lateral_fluctuation_margin = lateral_fluctuation_margin,
                                                alpha_lon_brake_min_correct = alpha_lon_brake_min_correct)


        rss_params = defineRssParams(x0, search_names)
        rss_params['alpha_lon_brake_min_correct'] = 0.1
        print('RSS params: %s' % rss_params)
        
        self._simulate(config, args, rss_params)

        self._cleanup()
        return 


        # reproduce trajs
        '''
        for i in range(10):
            self.filename_traj = os.path.join(RES_FOLDER, ('trajectory'+str(i)+'.csv'))
            ff(x0)
        '''
        # best_x_history, best_f_history, x_history, f_history, accept_x_history, accept_flags = annealing.runFunc(ff, X0, searchSpace, nruns, num_simult_runs, RES_FOLDER) 




if __name__ == '__main__':

    DESCRIPTION = ("CARLA RSS exploration\n")
    PARSER = argparse.ArgumentParser(description=DESCRIPTION,
                                     formatter_class=RawTextHelpFormatter)
    PARSER.add_argument('--host', default='127.0.0.1', help='IP of the host server (default: localhost)')
    PARSER.add_argument('--timeout', default="30.0",
                        help='Set the CARLA client timeout value in seconds')
    PARSER.add_argument('--port', default='2000', help='TCP port to listen to (default: 2000)')
    PARSER.add_argument('--debug', action="store_true", help='Run with debug output')
    PARSER.add_argument('--output', action="store_true", help='Provide results on stdout')
    PARSER.add_argument('--file', action="store_true", help='Write results into a txt file')
    PARSER.add_argument('--junit', action="store_true", help='Write results into a junit file')
    PARSER.add_argument('--outputDir', default='', help='Directory for output files (default: this directory)')
    PARSER.add_argument('--repetitions', default=1, help='Number of scenario executions')
    PARSER.add_argument('--waitForEgo', action="store_true", help='Connect the scenario to an existing ego vehicle')
    PARSER.add_argument('--configFile', default='', help='Provide an additional scenario configuration file (*.xml)')
    PARSER.add_argument('--additionalScenario', default='', help='Provide additional scenario implementations (*.py)')
    PARSER.add_argument('--reloadWorld', action="store_true",
                        help='Reload the CARLA world before starting a scenario (default=True)')
    PARSER.add_argument('--randomize', action="store_true", help='Scenario parameters are randomized')

    PARSER.add_argument('--openscenario', help='Provide an OpenSCENARIO definition')
    PARSER.add_argument(
        '--route', help='Run a route as a scenario (input: (route_file,scenario_file,[number of route]))', nargs='+', type=str)

    ARGUMENTS = PARSER.parse_args()
    ARGUMENTS.reloadWorld = True
    ARGUMENTS.configFile = os.path.join(os.getcwd(), 'rss.xml') # do not change this line
    

    ###############################################################
    # CHOOSE THE SCENARIO:
    ###############################################################
    # 1. Rss_LVS: Leading Vehicle Stopped 
    #    (One of the Automatic Emergency breaking scenarios (AEB))
    #       EV velocity:  40.2 km/h = 11.17 m/s
    #       POV velocity: 0 km/h    = 0 m/s
    # 2. Rss_LVM1: Leading Vehicle Moving, scenario #1 
    #    (One of the Automatic Emergency breaking scenarios (AEB))
    #       EV velocity:  40.2 km/h = 11.17 m/s
    #       POV velocity: 16.1 km/h = 4.47 m/s
    # 3. Rss_LVM2: Leading Vehicle Moving, scenario #2 
    #    (One of the Automatic Emergency breaking scenarios (AEB))
    #       EV velocity:  72.4 km/h = 20.1 m/s
    #       POV velocity: 32.2 km/h = 8.9 m/s
    # 4. Rss_LVD: Leading Vehicle Decelerating
    #    (One of the Automatic Emergency breaking scenarios (AEB))
    #       EV velocity:  56.3 km/h = 15.6 m/s
    #       POV velocity: 56.3 km/h = 15.6 m/s
    # 5. Rss_LVDAD: Leading Vehicle Decelerates, Accelerates, then Decelerates
    #    (Traffic Jam Assist scenarios (TJA))
    #       EV velocity:  40.2 km/h = 11.17 m/s
    #       POV velocity: 40.2 km/h = 11.17 m/s
    # 6. Rss_OppositeVehicleRunningRedLight
    #    (Intersection scenarios)
    # 7. Rss_PovUnprotectedLeft
    #    (Intersection scenarios)
    ###############################################################
    #ARGUMENTS.scenario = 'Rss_LVS'
    #ARGUMENTS.scenario = 'Rss_LVM1'
    #ARGUMENTS.scenario = 'Rss_LVM2'
    # ARGUMENTS.scenario = 'Rss_LVD'
    ARGUMENTS.scenario = 'Rss_LVDAD'
    # ARGUMENTS.scenario = 'Rss_OppositeVehicleRunningRedLight'
    # ARGUMENTS.scenario = 'Rss_PovUnprotectedLeft'
    ###############################################################

    RES_FOLDER = '../results-' + ARGUMENTS.scenario + '-' + time.strftime("%Y-%m-%d-%H-%M-%S")
    if not os.path.exists(RES_FOLDER):
        os.makedirs(RES_FOLDER)
    TRAJ_FILENAME = os.path.join(RES_FOLDER, 'trajectory.csv')

    ARGUMENTS.filename_traj = TRAJ_FILENAME


    SCENARIORUNNER = None
    try:
        SCENARIORUNNER = ScenarioRunner(ARGUMENTS)
        SCENARIORUNNER.run(ARGUMENTS)
    finally:
        if SCENARIORUNNER is not None:
            del SCENARIORUNNER
