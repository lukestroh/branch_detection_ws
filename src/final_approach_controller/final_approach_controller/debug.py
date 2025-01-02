def old_fit_func():

    # self.d_tof0_readings_filtered = np.where(np.isclose(self.d_tof0_readings, 255.0, atol=100.0), np.nan, self.d_tof0_readings) # TODO: Get VL6180 far plane for tolerances?

    #################################################33
    # self.d_tof0_readings_filtered = np.where(np.asarray(self.d_tof0_readings) < self.vl6180_far_plane, self.d_tof0_readings, np.nan)
    # # self.d_tof1_readings_filtered = np.where(np.as_array(self.d_tof1_readings) < vl6180_far_plane, self.d_tof1_readings, np.nan)

    # self.timestamps_tof0_filtered = np.where(np.isnan(self.d_tof0_readings_filtered), np.nan, self.timestamp_readings)

    # self.d_tof0_readings_filtered = self.d_tof0_readings_filtered[~np.isnan(self.d_tof0_readings_filtered)]
    # self.timestamps_tof0_filtered = self.timestamps_tof0_filtered[~np.isnan(self.timestamps_tof0_filtered)]
    # normalized_timestamps_tof0_filtered = self.timestamps_tof0_filtered - self.timestamps_tof0_filtered[0]

    # # np.where(np.isclose(d_tof0_readings, 255.0, atol=100.0), np.nan, d_tof0_readings)
    # # self.info(self.d_tof0_readings_filtered)
    # # self.info(self.timestamps_tof0_filtered)

    # fit_params_tof0, fit_cov_tof0 = so.curve_fit(cf.parabola, xdata=normalized_timestamps_tof0_filtered, ydata=self.d_tof0_readings_filtered, nan_policy='omit', maxfev=5000, sigma=0.001, absolute_sigma=True) # sigma=y-data uncertainty, need from VL6180
    #####################################################

    fit_tof0 = self.get_branch_fit(self.timestamp_readings, self.d_tof0_readings)
    if fit_tof0 is not None:
        fit_params_tof0, fit_covs_tof0 = fit_tof0

    #############################
    t_fit = np.linspace(
        min(normalized_timestamps_tof0_filtered),
        max(normalized_timestamps_tof0_filtered),
        len(normalized_timestamps_tof0_filtered),
    )
    a, b, c = fit_params_tof0
    # self.error(fit_params_tof0)
    tof0_fit = cf.parabola(t_fit, a, b, c)

    idx_min = np.argmin(tof0_fit)
    # self.warn(idx_min)

    timestamp_min_tof0 = self.timestamps_tof0_filtered[idx_min]
    # self.warn(timestamp_min_tof0)
    split_time_tof0 = np.modf(timestamp_min_tof0)
    time_tof0_center = Time(seconds=int(split_time_tof0[1]), nanoseconds=split_time_tof0[0] * 1e9)
    self.warn(time_tof0_center)

    fig = go.Figure()
    fig.add_trace(go.Scatter(x=self.timestamp_readings - self.timestamp_readings[0], y=self.d_tof0_readings))
    fig.add_trace(go.Scatter(x=normalized_timestamps_tof0_filtered, y=self.d_tof0_readings_filtered))

    fig.add_trace(go.Scatter(x=t_fit, y=tof0_fit))
    fig.show()

    self.destroy_node()
    rclpy.shutdown()

    ###################################

    # TODO: Get zero point, find index, relate back to raw timesteps

    self.warn(f"fit params:\n{fit_params_tof0}")
    self.info(f"covs:\n{fit_cov_tof0}")
    # fit_params_tof1, fit_cov_tof1 = so.curve_fit(cf.parabola, xdata=self.timestamp_readings, ydata=self.d_tof1_readings_filtered)
    return
