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
from srunner.scenariomanager.carla_data_provider import CarlaActorPool, CarlaDataProvider

import matplotlib.pyplot as plt 
import matplotlib.animation as animation


def _on_rss_response(response, safety_metrics_vector):
  ### get Rss Sensor safety metrics
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

  try:
    # First of all, we need to create the client that will send the requests
    # to the simulator. Here we'll assume the simulator is accepting
    # requests in the localhost at port 2000.
    client = carla.Client(host, int(port))
    client.set_timeout(client_timeout)

    # Once we have a client we can retrieve the world that is currently
    # running.
    world = client.get_world()

    settings = world.get_settings()
    #settings.synchronous_mode = True
    world.apply_settings(settings)

    # CarlaActorPool.set_world(world)
    CarlaDataProvider.set_world(world)


    fig, axs = plt.subplots(6, figsize=(9, 12))
    # safety_metrics = []

    rss_sensor_missing = True
    rss_sensor = None

    while True:
      #  get the RssSensor

      while rss_sensor_missing:
        # print(CarlaDataProvider.get_world().get_actors().filter("vehicle*")
        sensors = CarlaDataProvider.get_world().get_actors().filter('sensor.other.rss')
        if sensors is not None and len(sensors) > 0 :
          rss_sensor = sensors[0]
          rss_sensor_missing = False
          break
        else:
          print("waiting for RSS sensor")

      # print("pre-listen", rss_sensor.safety_metrics)
      rss_sensor.listen(lambda event: _on_rss_response(event, safety_metrics_vector))
      # print("after listen type", type(safety_metrics_vector))
      # print("safety_metrics_vector size", len(safety_metrics_vector))

      if len(safety_metrics_vector) > 0:

        active_situations = save_metrics_history(metrics_hist_dict, safety_metrics_vector)
        print("active situations size", len(active_situations))
      
        # print("safety_metrics_vector[-1] size", len(safety_metrics_vector[-1]))
        # safety_metrics_hist = np.array([])
        # safety_metrics_hist = np.asarray(safety_metrics_vector)
        # print("safety_metrics_hist shape", safety_metrics_hist.shape)

        # situations_cnt = safety_metrics_hist[-1].shape[0]
        situations_cnt = len(active_situations)

        # print("situations_cnt", situations_cnt)

        # create a color palette
        palette = plt.get_cmap('Set1')

        for ax in axs:
          ax.cla()

        # for i in range(0, situations_cnt):
        for i in active_situations:


          # metrics = safety_metrics_hist[:,i]
          

          # situation_id = metrics[-1].get_situation_id()
          situation_id = i

          print("situation id", situation_id)

          metrics = metrics_hist_dict[situation_id]
          hist_cnt = len(metrics)

          time = []

          msdf_lon = []
          msdf_latr = []
          msdf_latl = []

          msdv = []
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
            # print(len(msdv), metrics_hist.get_msdv().msd_violation)

            pra.append(metrics_hist.get_pra().pra)
            # print(metrics_hist.get_pra().pra)

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

      
          axs[0].plot(time, msdf_lon, color=palette(situation_id), linewidth=1.9,  label = situation_id) 
          axs[0].set(xlabel='time [s]', ylabel = 'MSDF Lon [m]')
          axs[0].plot(time, [1]*len(time), '--r')
          axs[0].legend(loc="upper right")
          
          # axs[1].cla()
          axs[1].plot(time, msdf_latr, color=palette(situation_id), linewidth=1.9, label = situation_id)
          # axs[1].plot(time, msdf_latl, 'g' , label = situation_id)
          axs[1].plot(time, [1]*len(time), '--r')
          axs[1].set(xlabel='time [s]', ylabel='MSDF Lat R [m]')
          axs[1].legend(loc="upper right")

          # axs[2].cla()
          axs[2].plot(time, pra, color=palette(situation_id), linewidth=1.9, label = situation_id)
          axs[2].set(xlabel='time [s]', ylabel='PRA')
          axs[2].legend(loc="upper right")

          # axs[3].cla()
          axs[3].plot(time, msdv, color=palette(situation_id), linewidth=1.9, label = situation_id)
          axs[3].set(xlabel='time [s]', ylabel='MSDV')
          axs[3].legend(loc="upper right")

          # axs[4].cla()
          axs[4].plot(time, ttc, color=palette(situation_id), linewidth=1.9, label = situation_id)
          # axs[4].plot(time, ttc_v, 'g' , label = 'TTC Violation')
          axs[4].plot(time, ttc_thr, '--r')
          axs[4].set(xlabel='time [s]', ylabel='TTC [s]')
          axs[4].legend(loc="upper right")

          # axs[5].cla()
          axs[5].plot(time, mttc, color=palette(situation_id), linewidth=1.9, label = situation_id)
          # axs[5].plot(time, mttc_v, 'g' , label = situation_id)
          axs[5].plot(time, mttc_thr, '--r')
          axs[5].set(xlabel='time [s]', ylabel='MTTC [s]')
          axs[5].legend(loc="upper right")

          # Format plot
          # plt.subplots_adjust(bottom=0.20)
          fig.suptitle('Safety Metrics')
          # plt.tight_layout()
        plt.pause(0.001)

        plt.savefig("safety_metrics.png")
        

  finally:

    # settings = world.get_settings()
    # settings.synchronous_mode = False
    # world.apply_settings(settings)

    CarlaDataProvider.cleanup()
    CarlaActorPool.cleanup()



if __name__ == '__main__':

  try:
    run_loop()

  except KeyboardInterrupt:
      print('\nCancelled by user. Bye!')





