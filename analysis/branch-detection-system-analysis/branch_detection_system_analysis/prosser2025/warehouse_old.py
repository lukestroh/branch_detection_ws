def slice_dfs_by_transition_event(big_df: pd.DataFrame, transition_event_df: pd.DataFrame) -> list[pd.DataFrame]:
    # Grab the ToF, wrench, etc. data between transition events
    trial_dfs = []

    logger.info("Slicing dataframes by data channel")

    df_fpc_transitions = transition_event_df.loc[
        transition_event_df["fpc_transition_start_state"] == TransitionStates.TRANSITION_STATE_DEACTIVATING.value
    ].reset_index()

    # Get start and end timestamps from transition events
    i = 0

    # TODO THis doesn't get the last one
    while True:
        try:
            start_time = df_fpc_transitions.at[i * 2, "fpc_transition_events_ts"]
            end_time = df_fpc_transitions.at[i * 2 + 2, "fpc_transition_events_ts"]

            df_tof0_raw_trial = big_df.loc[
                get_bin_mask(big_df, start_time, end_time, "tof0_raw_ts"), ["tof0_raw_ts", "tof0_raw_data"]
            ]
            df_tof1_raw_trial = big_df.loc[
                get_bin_mask(big_df, start_time, end_time, "tof1_raw_ts"), ["tof1_raw_ts", "tof1_raw_data"]
            ]
            df_tof0_filtered_trial = big_df.loc[
                get_bin_mask(big_df, start_time, end_time, "tof0_filtered_ts"),
                ["tof0_filtered_ts", "tof0_filtered_data"],
            ]
            df_tof1_filtered_trial = big_df.loc[
                get_bin_mask(big_df, start_time, end_time, "tof1_filtered_ts"),
                ["tof1_filtered_ts", "tof1_filtered_data"],
            ]
            df_wrench_trial = big_df.loc[
                get_bin_mask(big_df, start_time, end_time, "wrench_ts"),
                ["wrench_ts", "wrench_fx", "wrench_fy", "wrench_fz", "wrench_tx", "wrench_ty", "wrench_tz"],
            ]
            df_joint_states_trial = big_df.loc[
                get_bin_mask(big_df, start_time, end_time, "joint_angle_ts"),
                ["joint_angle_ts", "joint_angle_pos"],
            ]

            trial_dfs.append(
                {
                    "trial_num": i,
                    "tof0_raw": df_tof0_raw_trial,
                    "tof1_raw": df_tof1_raw_trial,
                    "tof0_filtered": df_tof0_filtered_trial,
                    "tof1_filtered": df_tof1_filtered_trial,
                    "wrench": df_wrench_trial,
                    "joint_states": df_joint_states_trial,
                }
            )

            # print(trial_dfs)

            # break

            i += 1
        except (KeyError, ValueError):
            break

    return trial_dfs


def warehouse_trial_dfs(trial_dfs: list[dict[str, pd.DataFrame]], filename: str):
    name = Path(Path(filename).stem).stem
    trial_path = os.path.join(warehouse_path, name)
    os.mkdir(trial_path)
    for trial in trial_dfs:
        for topic_name, df in trial.items():
            if topic_name.endswith("trial_num"):
                continue
            else:
                df.to_hdf(
                    os.path.join(trial_path, name + f"__{topic_name}__{str(trial['trial_num']).zfill(3)}.h5"),
                    key=name,
                )

    return
