# JLL, 2022.6.30
# from /home/jinn/OP/laika/examples/Kalman.ipynb
#   or https://github.com/commaai/laika/blob/master/examples/Kalman.ipynb

# In this example we will show the difference between fixes computed with laika
# from raw data of the ublox receiver vs the fixes the ublox receiver computes

import numpy as np
from pathlib import Path

base_dir = Path('example_data')
raw_ublox_t = np.load(base_dir / 'raw_gnss_ublox/t')
raw_ublox = np.load(base_dir / 'raw_gnss_ublox/value')
fixes_ublox_t = np.load(base_dir / 'live_gnss_ublox/t')
fixes_ublox = np.load(base_dir / 'live_gnss_ublox/value')

  # We get the raw data into our format from the log array format
from laika.raw_gnss import normal_meas_from_array
measurements = np.array([normal_meas_from_array(arr) for arr in raw_ublox])

  # initialize an astrodog with dgps corrections
from laika import AstroDog
dog = AstroDog(dgps=True)

  # Building this cache takes forever, just copy it from repo
from shutil import copyfile

cache_directory = Path(dog.cache_dir) / 'cors_coord'
cache_directory.mkdir(parents=True, exist_ok=True)
copyfile('cors_station_positions', cache_directory / 'cors_station_positions')

from laika.raw_gnss import process_measurements, correct_measurements, calc_pos_fix
from tqdm.auto import tqdm

  # We want to group measurements by measurement epoch
  # this makes the kalman filter faster and is easier
  # to reason about
grouped_t = np.unique(raw_ublox_t)
grouped_meas_processed = []
corrected_meas_arrays = []

  # process measurement groups
for t in grouped_t:
    meas = measurements[raw_ublox_t == t]
    grouped_meas_processed.append(process_measurements(meas, dog))

  # correct measurement groups with an estimate position
  # that was computes with weighted-least-squares on
  # the first epoch
  # WARNING: can take up to 10min
wls_estimate = calc_pos_fix(grouped_meas_processed[0])
est_pos = wls_estimate[0][:3]
for proc in tqdm(grouped_meas_processed):
    corrected = correct_measurements(proc, est_pos, dog)
    corrected_meas_arrays.append(np.array([c.as_array() for c in corrected]))

    for proc in tqdm(grouped_meas_processed):
        corrected = correct_measurements(proc, est_pos, dog)
        corrected_meas_arrays.append(np.array([c.as_array() for c in corrected]))

          # Generate and build a CFFI library from the Kalman filter defined symbolically in SymPy
        from kalman.models.gnss_kf import GNSSKalman

        GNSSKalman.generate_code()
        !g++ kalman/generated/gnss.cpp -Wall -fPIC -shared -o kalman/generated/libgnss.so

          # We run the Kalman filter
        from kalman.kalman_helpers import run_car_ekf_offline, ObservationKind

        ekf = GNSSKalman()
        init_state = ekf.x
        init_state[:3] = est_pos
        ekf.init_state(init_state)
        ekf_data = {}
        ekf_data[ObservationKind.PSEUDORANGE_GPS] = (grouped_t, corrected_meas_arrays)
        ekf_data[ObservationKind.PSEUDORANGE_RATE_GPS] = (grouped_t, corrected_meas_arrays)
        ekf_outputs = run_car_ekf_offline(ekf, ekf_data)

        import laika.lib.coordinates as coord

        laika_positions_t = ekf_outputs[4]
        laika_positions_ecef = ekf_outputs[0][:, :3]
        laika_positions_geodetic = coord.ecef2geodetic(laika_positions_ecef)

          # By looking at the map, we can see that the two paths compared.
          # If you want to regenerate the gmplot you will need a google maps API key

        import gmplot

        gmap = gmplot.GoogleMapPlotter(*laika_positions_geodetic[0])
          #gmap.apikey='...'
        gmap.plot([x[0] for x in laika_positions_geodetic], [x[1] for x in laika_positions_geodetic], 'blue', edge_width=5)
        gmap.plot([x[0] for x in ublox_positions_geodetic], [x[1] for x in ublox_positions_geodetic], 'red', edge_width=5)
        gmap.draw("laika_quality_check.html")

        import webbrowser
        import os

        webbrowser.open('file://' + os.path.realpath("laika_quality_check.html"));
