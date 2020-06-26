from __future__ import print_function

import traceback
import argparse
from argparse import RawTextHelpFormatter
from datetime import datetime
import carla
import sys
import os
import time
import numpy as np

sys.path.append(os.getenv('ROOT_SCENARIO_RUNNER'))
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider


import matplotlib.pyplot as plt 
import matplotlib.animation as animation


def _on_rss_response(response, safety_metrics_vector):
  ### get Rss Sensor safety metrics
  # safety_metrics_vector = []
  safety_metrics_vector.append(response.safety_metrics)

  # print("in listen type", len(safety_metrics_vector))

def save_metrics_history(metrics_hist_dict, safety_metrics_vector):
  active_situations = []
  
  for sm in safety_metrics_vector[-1]:
    situation_id = sm.get_situation_id()
    active_situations.append(situation_id)

    if situation_id not in metrics_hist_dict:
      metrics_hist_dict[situation_id] = []

    metrics_hist_dict[situation_id].append(sm)

  return active_situations

def retrieve_wold(client):

  world = client.get_world()
  world_id = world.id

  settings = world.get_settings()
  # settings.synchronous_mode = True
  world.apply_settings(settings)

  CarlaActorPool.set_client(client)
  CarlaActorPool.set_world(world)
  CarlaDataProvider.set_world(world)

  return world


def figure_settings():
  
  fig, axs = plt.subplots(6, figsize=(8, 10))

  SMALL_SIZE = 8
  MEDIUM_SIZE = 10
  BIGGER_SIZE = 12

  plt.rc('font', size=SMALL_SIZE)          # controls default text sizes
  plt.rc('axes', titlesize=SMALL_SIZE)     # fontsize of the axes title
  plt.rc('axes', labelsize=SMALL_SIZE)     # fontsize of the x and y labels
  plt.rc('xtick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
  plt.rc('ytick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
  plt.rc('legend', fontsize=SMALL_SIZE)    # legend fontsize
  plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title
  
  plt.rc('lines', linewidth=1.2)            # linewidth of the figure
  # plt.subplots_adjust(bottom=0.1)

  return fig, axs

# ==============================================================================
# -- run_loop() ---------------------------------------------------------------
# ==============================================================================


def run_loop(host='127.0.0.1', port=2000):

  
  # Tunable parameters
  client_timeout = 30.0  # in seconds
  wait_for_world = 20.0  # in seconds
  world = None

  safety_metrics_vector = []
  metrics_hist_dict = {}

  # try:
  
  # First of all, we need to create the client that will send the requests
  # to the simulator. Here we'll assume the simulator is accepting
  # requests in the localhost at port 2000.
  client = carla.Client(host, int(port))
  client.set_timeout(client_timeout)
  
  # Once we have a client we can retrieve the world that is currently
  # running.
  world = retrieve_wold(client)
  world_id = world.id
  current_world_id = world_id
  
  fig, axs = figure_settings()


  rss_sensor_missing = True
  rss_sensor = None
  ego_vehicle_missing = True
  new_episode = False

  ego_found = False

  while ego_found == False:
    print("Waiting for ego vehicle")
    ego_vehicle = CarlaDataProvider.get_hero_actor()
    if ego_vehicle != None:
      ego_found = True
        
  
  # for actor in world.get_actors():
  #     if actor.attributes and 'role_name' in actor.attributes and actor.attributes['role_name'] == 'hero':
  #         ego_found = True
  #         print("Ego vehicle found")
  #         break

  #  get the RssSensor
  while rss_sensor_missing:
    sensors = CarlaDataProvider.get_world().get_actors().filter('sensor.other.rss')
    if sensors is not None and len(sensors) > 0 :
      rss_sensor = sensors[0]
      rss_sensor_missing = False
      print("RSS sensor found")
      break
    else:
      print("waiting for RSS sensor")

  ## need stopping condition
  while actor_id_exists == True:


    ### listen to RSS sensor attached to ego_vehicle
    rss_sensor.listen(lambda event: _on_rss_response(event, safety_metrics_vector))


    if len(safety_metrics_vector) > 0:

      active_situations = save_metrics_history(metrics_hist_dict, safety_metrics_vector)
      print("active situations size", len(active_situations))

      # create a color palette
      palette = plt.get_cmap('Set1')

      if len(active_situations) > 0:
        
        for ax in axs:
          ax.cla()

        for i in active_situations:

          situation_id = i
          metrics = metrics_hist_dict[situation_id]
          hist_cnt = len(metrics)

          time = []

          msdf_lon = []
          msdf_latr = []
          msdf_latl = []

          msdv = []
          d_lon = []
          d_min_lon = []
          pra = []
        
          ttc = []
          ttc_v = []
          ttc_thr = []

          mttc = []      
          mttc_v = []
          mttc_thr = []

          for j in range(0, hist_cnt):
            metrics_hist = metrics[j]
        
            # print("------")
            time.append(metrics_hist.get_timestamp().elapsed_seconds)
            # print(len(time), metrics_hist.get_timestamp().elapsed_seconds)

            msdf_lon.append(metrics_hist.get_msdf().msdf_lon)
            msdf_latr.append(metrics_hist.get_msdf().msdf_lat_r)
            msdf_latl.append(metrics_hist.get_msdf().msdf_lat_l)            
            # print(len(msdf_lon), metrics_hist.get_msdf().msdf_lon)
            # print(metrics_hist.get_msdf().msdf_lat_r)
            # print(metrics_hist.get_msdf().msdf_lat_l)
            
            msdv.append(metrics_hist.get_msdv().msd_violation)
            
            d_lon.append(metrics_hist.d_lon)
            d_min_lon.append(metrics_hist.d_min_lon)

            # print("msdv", metrics_hist.get_msdv().msd_violation)

            pra.append(metrics_hist.get_pra().pra)
            # print("pra", metrics_hist.get_pra().pra)

            ttc.append(metrics_hist.get_ttc().ttc)
            ttc_v.append(metrics_hist.get_ttc().ttc_violation)
            ttc_thr.append(metrics_hist.get_ttc().ttc_threshold)            
            # print(metrics_hist.get_ttc().ttc)
            # print(metrics_hist.get_ttc().ttc_violation)
            # print(metrics_hist.get_ttc().ttc_threshold)

            mttc.append(metrics_hist.get_mttc().mttc)
            mttc_v.append(metrics_hist.get_mttc().mttc_violation)
            mttc_thr.append(metrics_hist.get_mttc().mttc_threshold)
            # print(metrics_hist.get_mttc().mttc)
            # print(metrics_hist.get_mttc().mttc_violation)
            # print(metrics_hist.get_mttc().mttc_threshold)

            # print("------")

          # print("d_min_lon", d_min_lon, "d_lon", d_lon)


          label = "car ID: " + str(situation_id)

          ## D_lon anf D_min_lon
          axs[0].plot(time, d_lon, color='g', label = r'$d^{long}_{curr}$' + " - " + label)
          axs[0].plot(time, d_min_lon, color='y', label = r'$d^{long}_{min}$' + " - " + label )
          axs[0].set(xlabel='time [s]', ylabel='LV ' + r'$d^{long}_{curr}$' + " and RSS " + r'$d^{long}_{min}$')
          axs[0].legend(loc="upper right")
          axs[0].grid()
          
          ## PRA and MSDV
          axs[1].plot(time, pra, color=palette(situation_id), label = "PRA - " + label)
          axs[1].plot(time, msdv, color=palette(situation_id+1), label = "MSDV - " + label)
          axs[1].set(xlabel='time [s]', ylabel = 'PRA and MSDV')
          axs[1].legend(loc="upper right")
          axs[1].grid()

          ## MSDF Lon
          axs[2].plot(time, msdf_lon, color=palette(situation_id),  label = label) 
          axs[2].set(xlabel='time [s]', ylabel = 'MSDF Lon [m]')
          axs[2].plot(time, [1]*len(time), '--r')
          axs[2].legend(loc="upper right")
          axs[2].grid()
          
          ## MSDF Lat
          # axs[1].plot(time, msdf_latr, color=palette(situation_id), label = label)
          # # axs[1].plot(time, msdf_latl, 'g' , label = label)
          # axs[1].plot(time, [1]*len(time), '--r')
          # axs[1].set(xlabel='time [s]', ylabel='MSDF Lat R [m]')
          # axs[1].legend(loc="upper right")


          ## TTC
          axs[3].plot(time, ttc, color=palette(situation_id), label = label)
          # axs[3].plot(time, ttc_v, 'g' , label = label + ' - TTC Violation')
          axs[3].plot(time, ttc_thr, '--r')
          axs[3].set(xlabel='time [s]', ylabel='TTC [s]')
          axs[3].legend(loc="upper right")
          axs[3].grid()

          ## MTTC
          axs[4].plot(time, mttc, color=palette(situation_id), label = label)
          axs[4].plot(time, mttc_thr, '--r')
          axs[4].set(xlabel='time [s]', ylabel='MTTC [s]')
          axs[4].legend(loc="upper right")
          axs[4].grid()

          ## TTCV and MTTCV
          axs[5].plot(time, ttc_v, color=palette(situation_id), label = label + " - TTCV")
          axs[5].plot(time, mttc_v, color=palette(situation_id+1), label = label + " - MTTCV")
          axs[5].set(xlabel='time [s]', ylabel='TTCV and MTTCV')
          axs[5].legend(loc="upper right")
          axs[5].grid()


          # Format plot
          fig.suptitle('Safety Metrics')
          # plt.tight_layout()
        plt.pause(0.00001)
      
      plt.savefig("safety_metrics.png")


    ego_found = False
    for actor in world.get_actors():
        if actor.attributes and 'role_name' in actor.attributes and actor.attributes['role_name'] == 'hero':
            ego_found = True
            print("Ego vehicle found")
            break
    
    if not ego_found:
      print("---------")
      print("new episode")
      print("--------")
      sys.exit(-1)


    # '''
    # As the world is re-loaded for every scenario, no ego exists so far
    # '''
    # ego_vehicle_found = False
    # carla_vehicles = CarlaDataProvider.get_world().get_actors().filter('vehicle.*')
    # if len(carla_vehicles) > 0 :
    #   for carla_vehicle in carla_vehicles:
    #     if carla_vehicle.attributes['role_name'] == 'hero':
    #         ego_vehicle_found = True
    #         print("vehicle found")

    # if ego_vehicle_found == False:
    #   print("---------")
    #   print("new episode")
    #   print("--------")
    #   current_episode = False
    #   # break

    # curr_world = CarlaDataProvider.get_world()

    # # current_world_id = world.id

    # if curr_world != world:
      
    #   print("starting new episode")
    #   print("exiting")
    #   sys.exit(-1)
    #   # break

    # if CarlaDataProvider.get_world() != world:
    #   print("EXITING")
    #   break
    
  print("exiting")
  sys.exit(-1)

if __name__ == '__main__':

  try:
    run_loop()

  except KeyboardInterrupt:
      print('\nCancelled by user. Bye!')





